#include "tp.h"



using Eigen::MatrixXd;
using Eigen::VectorXd;

TrajectoryPlanning::TrajectoryPlanning(double car_x_, double car_y_, double car_yaw_, double car_s_, double ref_vel_, int lane_,
                   nlohmann::json previous_path_x_, nlohmann::json previous_path_y_,
                   vector<double> map_waypoints_s_, vector<double> map_waypoints_x_, vector<double> map_waypoints_y_)
{
    car_x = car_x_;
    car_y = car_y_,
    car_yaw = car_yaw_;
    car_s = car_s_;
    ref_vel = mph2ms(ref_vel_); // m/s
    lane = lane_;

    previous_path_x = previous_path_x_;
    previous_path_y = previous_path_y_;

    map_waypoints_s = map_waypoints_s_;
    map_waypoints_x = map_waypoints_x_;
    map_waypoints_y = map_waypoints_y_;
}
vector<vector<double>> TrajectoryPlanning::spline_trajectory_generation(void)
{
    vector<double> grid_x;
    vector<double> grid_y;
    double lane_change_duration = 4;  // time take to lane change [s]
    double car_speed;  // [m/s]
    double offset_s = 10;
    double static pre_car_s;
    int prev_size = previous_path_x.size();

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // ptsx[0],ptsx[1] are for tangent of previous path
    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        grid_x.push_back(prev_car_x);
        grid_x.push_back(car_x);

        grid_y.push_back(prev_car_y);
        grid_y.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        grid_x.push_back(previous_path_x[prev_size - 2]);
        grid_x.push_back(previous_path_x[prev_size - 1]);
        grid_y.push_back(previous_path_y[prev_size - 2]);
        grid_y.push_back(previous_path_y[prev_size - 1]);
        car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / SAMPLING_T) ; // m/s
    }
    // ptsx[2],[3],[4] are netx wp goal

    double passing_idx = 50 - previous_path_x.size();
    double car_sdot = (car_s - pre_car_s) / (SAMPLING_T * passing_idx);
    printf("car_sdot: %lf\n", car_sdot);
    if (car_sdot <5)
    {
        car_sdot = 5; // threshold
    }
    pre_car_s = car_s;

    vector<double> next_wp0 = getXY(car_s + offset_s + car_sdot*lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 2*offset_s+ car_sdot*lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 3*offset_s+ car_sdot*lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    grid_x.push_back(next_wp0[0]);
    grid_x.push_back(next_wp1[0]);
    grid_x.push_back(next_wp2[0]);

    grid_y.push_back(next_wp0[1]);
    grid_y.push_back(next_wp1[1]);
    grid_y.push_back(next_wp2[1]);

    // call by reference, global 2 local coordinate conversion
    global2local_coord_conversion(grid_x, grid_y, ref_x, ref_y, ref_yaw);

    // spline fitting
    tk::spline s;
    s.set_points(grid_x, grid_y);
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }


    double target_x = 2*offset_s + car_sdot*lane_change_duration; //offset_s+car_speed*lane_change_duration;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {

        if (ref_vel > car_speed)
        {
            car_speed += 0.1;
        }
        else if (ref_vel < car_speed)
        {
            car_speed -= 0.1;
        }

        double N = (target_dist / (SAMPLING_T * car_speed ));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return {next_x_vals, next_y_vals};
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}

double CalQuintic(vector<double> coeff, double T)
{
    double result = coeff[0] + coeff[1] * T + coeff[2] * T * T + coeff[3] * T * T * T + coeff[4] * T * T * T * T + coeff[5] * T * T * T * T * T;
    return result;
}


void global2local_coord_conversion(vector<double> &ptx, vector<double> &pty, double ref_x, double ref_y, double ref_yaw)
{
    // local coorinate conversion, after shift, ref_x and ref_y is coordinate (0, 0)
    for (int i = 0; i < ptx.size(); i++)
    {
        //shift car reference angle to 0 degrees
        double shift_x = ptx[i] - ref_x;
        double shift_y = pty[i] - ref_y;
        ptx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        pty[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
}

vector<vector<double>> quintic_polynomial_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double car_d, double ref_vel, int lane, double end_path_s, double end_path_d,
                                              nlohmann::json previous_path_x, nlohmann::json previous_path_y, double car_speed,
                                              vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int behavior)
{
    static double q_pre_car_s;
    static double q_pre_car_sdot;
    static double q_pre_car_d;
    static double q_pre_car_ddot;
    double time_to_goal = 1;
    int prev_size = previous_path_x.size();
    vector<double> ptsx;
    vector<double> ptsy;
    double ref_x;
    double ref_y;
    double ref_yaw;
    // polynomial generation
    // Add polynomial start point and goal point

    // ptsx[0],ptsx[1] are for tangent of previous path
    if (prev_size < 2)
    {
        ref_x = car_x;
        ref_y = car_y;
        ref_yaw = deg2rad(car_yaw);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    }
    printf("prev_size: %d\n",prev_size);
    vector<double> ptpx;
    vector<double> ptpy;
    for (int i = 0; i < 5; i++)
    {
    }
    // Choose 5 way points
    double ref_s = end_path_s;
    double ref_d = end_path_d;

    double passing_idx = 50 - previous_path_x.size();
    double car_sdot = (car_s - q_pre_car_s) / (SAMPLING_T * passing_idx);
    double car_sdotdot = (car_sdot - q_pre_car_sdot) / (SAMPLING_T * passing_idx);
    double car_ddot = (car_d - q_pre_car_d) / (SAMPLING_T * passing_idx);
    double car_ddotdot = (car_ddot - q_pre_car_ddot) / (SAMPLING_T * passing_idx);
    // if (car_sdot <5)
    // {
    //     car_sdot = 5; // threshold
    // }
    q_pre_car_s = car_s;
    q_pre_car_sdot = car_sdot;
    q_pre_car_d = car_d;
    q_pre_car_ddot = car_ddot;
    

    // // Generate polynomial function
    double poly_gen_time = 1;
    vector<double> start_s = {0, car_sdot, 0};
    vector<double> start_d = {0, car_ddot, 0};
    vector<double> goal_s = {10, 3, 0};
    vector<double> goal_d;
    // if (behavior == kLaneChangeRight)
    // {
    //     goal_d = {4, 0, 0};
    // }
    // else if (behavior == kLaneChangeLeft)
    // {
    //      goal_d = {-4, 0, 0};
    // }
    // else
    // {
    //     goal_d = {0, 0, 0};
    // }
    goal_d = {0, 0, 0};
    vector<double> s_coeff = JMT(start_s, goal_s, poly_gen_time);
    vector<double> d_coeff = JMT(start_d, goal_d, poly_gen_time);
    // for (int i = 0; i< s_coeff.size(); i++)
    // {
    //     printf("s_coeff: %lf\n",s_coeff[i]);
    //     printf("d_coeff: %lf\n",d_coeff[i]);

    // }
    
    printf("car_sdot: %lf\n",car_sdot);
    printf("car_sdotdot: %lf\n",car_sdotdot);
    printf("car_ddot: %lf\n",car_ddot);
    printf("car_ddotdot: %lf\n",car_ddotdot);

    vector<double> local_s_trajectory;
    vector<double> local_d_trajectory;
    for (double t = 0.02; t <= 0.02*50; t+=0.02)
    {
      double next_s = CalQuintic(s_coeff, t);
      double next_d = CalQuintic(d_coeff, t);
      local_s_trajectory.push_back(next_s);
      local_d_trajectory.push_back(next_d);
      printf("next_s: %.2lf next_d: %.2lf, t: %.2lf \n", next_s, next_d, t);
    }
    vector<double> local_refx;
    vector<double> local_refy;
    vector<double> local_refs;
    for (int i = 0; i<=20; i++)
    {
        local_refx.push_back(10*i);
        local_refy.push_back(0);
        local_refs.push_back(20*i);
    }
    vector<double> local_x_trajectory;
    vector<double> local_y_trajectory;    
    for (int i = 0; i < local_s_trajectory.size(); i ++)
    {
        vector<double> local_xy = getXY(local_s_trajectory[i],local_d_trajectory[i],local_refs,local_refx,local_refy);
        local_x_trajectory.push_back(local_xy[0]);
        local_y_trajectory.push_back(local_xy[1]);
        // printf("local x: %lf, local y: %lf\n",local_xy[0], local_xy[1]);
    }

    
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 0; i< local_x_trajectory.size(); i++)
    {
        double x_point = (local_x_trajectory[i] * cos(ref_yaw) - local_y_trajectory[i] * sin(ref_yaw));
        double y_point = (local_x_trajectory[i] * sin(ref_yaw) + local_y_trajectory[i] * cos(ref_yaw));
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);

    }
    return {next_x_vals, next_y_vals};



           
        

    // for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    // {

    //     if (ref_vel > car_speed)
    //     {
    //         car_speed += 0.1;
    //     }
    //     else if (ref_vel < car_speed)
    //     {
    //         car_speed -= 0.1;
    //     }

    //   double t_inc = SAMPLING_T;
    //   t_add_on = t_add_on + t_inc;
    //   double next_s = CalQuintic(s_coeff, t_add_on);
    //   double next_d = CalQuintic(d_coeff, t_add_on);
    //   next_s = next_s + ref_s;
    //   next_d = next_d + ref_d;
    //   printf("next_s: %lf next_d: %lf \n", next_s, next_d);
    // }
    // printf("------------------------------\n");
}
// vector<double> PTG(vector<double> start_s,
//                    vector<double> start_d,
//                    Vehicle target_vehicle,
//                    vector<double> delta,
//                    double T)
// {

//     double timestep = SAMPLING_T;
//     double t;
//     vector<double> goal_s(3);
//     vector<double> goal_d(3);
//     vector<double> target_state;
//     vector<double> s_coefficients(3);
//     vector<double> d_coefficients(3);
//     vector<double> ptg_coefficients(6);
//     t = T - 4 * timestep;
//     while (t <= T + 4 * timestep)
//     {
//         target_state = target_vehicle.state_in(t);
//         goal_s[0] = target_state[0];
//         goal_s[1] = target_state[1];
//         goal_s[2] = target_state[2];
//         goal_d[0] = target_state[3];
//         goal_d[1] = target_state[4];
//         goal_d[2] = target_state[5];
//     }

//     s_coefficients = JMT(start_s, goal_s, T + 4 * timestep);
//     d_coefficients = JMT(start_d, goal_d, T + 4 * timestep);
//     ptg_coefficients[0] = s_coefficients[0];
//     ptg_coefficients[1] = s_coefficients[1];
//     ptg_coefficients[2] = s_coefficients[2];
//     ptg_coefficients[3] = d_coefficients[0];
//     ptg_coefficients[4] = d_coefficients[1];
//     ptg_coefficients[5] = d_coefficients[2];
//     return ptg_coefficients;

//     // target = predictions[target_vehicle]
//     //     # generate alternative goals
//     //     all_goals = []
//     //     timestep = 0.5
//     //     t = T - 4 * timestep
//     //     while t <= T + 4 * timestep:
//     //         target_state = np.array(target.state_in(t)) + np.array(delta)
//     //         goal_s = target_state[:3]
//     //         goal_d = target_state[3:]
//     //         goals = [(goal_s, goal_d, t)]
//     //         for _ in range(N_SAMPLES):
//     //             perturbed = perturb_goal(goal_s, goal_d)
//     //             goals.append((perturbed[0], perturbed[1], t))
//     //         all_goals += goals
//     //         t += timestep

//     //     # find best trajectory
//     //     trajectories = []
//     //     for goal in all_goals:
//     //         s_goal, d_goal, t = goal
//     //         s_coefficients = JMT(start_s, s_goal, t)
//     //         d_coefficients = JMT(start_d, d_goal, t)
//     //         trajectories.append(tuple([s_coefficients, d_coefficients, t]))

//     //     best = min(trajectories, key=lambda tr: calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
//     //     calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
//     //     return best
// }