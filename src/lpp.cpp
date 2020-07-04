#include "lpp.h"
#include "spline.h"

#define SAMPLING_T 0.2

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> Vehicle::state_in(double t)
{
    vector<double> s(3);
    vector<double> d(3);
    s[0] = start_state[0];
    s[1] = start_state[1];
    s[2] = start_state[2];
    d[0] = start_state[3];
    d[1] = start_state[4];
    d[2] = start_state[5];

    vector<double> state = {
        s[0] + (s[1] * t) + s[2] * t * t / 2.0,
        s[1] + s[2] * t,
        s[2],
        d[0] + (d[1] * t) + d[2] * t * t / 2.0,
        d[1] + d[2] * t,
        d[2],
    };
    return state;
}

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T)
{
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
   */
    MatrixXd A(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;
    // std::cout << A << std::endl;
    MatrixXd B(3, 1);
    B << end[0] - start[0] - start[1] * T - start[2] / 2 * T * T,
        end[1] - start[1] - start[2] * T,
        end[2] - start[2];
    // std::cout << B << std::endl;
    MatrixXd Ai(3, 3);
    Ai = A.inverse();
    MatrixXd Coeff(3, 1);
    Coeff = Ai * B;
    std::cout << Coeff << std::endl;
    vector<double> result = {start[0], start[1], start[2] / 2, Coeff.coeff(0, 0), Coeff.coeff(1, 0), Coeff.coeff(2, 0)};
    return result;
}
double CalPoly(vector<double> coeff, double T)
{
    double result = coeff[0] + coeff[1] * T + coeff[2] * T * T + coeff[3] * T * T * T + coeff[4] * T * T * T * T + coeff[5] * T * T * T * T * T;
    return result;
}
vector<double> PTG(vector<double> start_s,
                   vector<double> start_d,
                   Vehicle target_vehicle,
                   vector<double> delta,
                   double T)
{

    double timestep = SAMPLING_T;
    double t;
    vector<double> goal_s(3);
    vector<double> goal_d(3);
    vector<double> target_state;
    vector<double> s_coefficients(3);
    vector<double> d_coefficients(3);
    vector<double> ptg_coefficients(6);
    t = T - 4 * timestep;
    while (t <= T + 4 * timestep)
    {
        target_state = target_vehicle.state_in(t);
        goal_s[0] = target_state[0];
        goal_s[1] = target_state[1];
        goal_s[2] = target_state[2];
        goal_d[0] = target_state[3];
        goal_d[1] = target_state[4];
        goal_d[2] = target_state[5];
    }

    s_coefficients = JMT(start_s, goal_s, T + 4 * timestep);
    d_coefficients = JMT(start_d, goal_d, T + 4 * timestep);
    ptg_coefficients[0] = s_coefficients[0];
    ptg_coefficients[1] = s_coefficients[1];
    ptg_coefficients[2] = s_coefficients[2];
    ptg_coefficients[3] = d_coefficients[0];
    ptg_coefficients[4] = d_coefficients[1];
    ptg_coefficients[5] = d_coefficients[2];
    return ptg_coefficients;

    // target = predictions[target_vehicle]
    //     # generate alternative goals
    //     all_goals = []
    //     timestep = 0.5
    //     t = T - 4 * timestep
    //     while t <= T + 4 * timestep:
    //         target_state = np.array(target.state_in(t)) + np.array(delta)
    //         goal_s = target_state[:3]
    //         goal_d = target_state[3:]
    //         goals = [(goal_s, goal_d, t)]
    //         for _ in range(N_SAMPLES):
    //             perturbed = perturb_goal(goal_s, goal_d)
    //             goals.append((perturbed[0], perturbed[1], t))
    //         all_goals += goals
    //         t += timestep

    //     # find best trajectory
    //     trajectories = []
    //     for goal in all_goals:
    //         s_goal, d_goal, t = goal
    //         s_coefficients = JMT(start_s, s_goal, t)
    //         d_coefficients = JMT(start_d, d_goal, t)
    //         trajectories.append(tuple([s_coefficients, d_coefficients, t]))

    //     best = min(trajectories, key=lambda tr: calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
    //     calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
    //     return best
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

void quintic_polynomial_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double car_d, double ref_vel, int lane, double end_path_s, double end_path_d,
                                                    nlohmann::json previous_path_x, nlohmann::json previous_path_y, double car_speed,
                                                    vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
              static double pre_car_s;
          static double pre_car_sdot;
          static double pre_car_d;
          static double pre_car_ddot;
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
          vector<double> ptpx;
          vector<double> ptpy;
          for (int i = 0; i < 5; i++)
          {
          }
          // Choose 5 way points
          double ref_s = end_path_s;
          double ref_d = end_path_d;

          vector<double> prev_wp0 = getXY(ref_s - 2 * time_to_goal * car_speed, ref_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> prev_wp1 = getXY(ref_s - time_to_goal * car_speed, ref_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> ref_wp = getXY(ref_s, ref_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp0 = getXY(ref_s + time_to_goal * car_speed, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ref_s + 2 * time_to_goal * car_speed, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ref_s + 3 * time_to_goal * car_speed, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          printf("ref_x: %lf ref_y: %lf\n", ref_x, ref_y);
          printf("ref_x: %lf ref_y: %lf\n", ref_wp[0], ref_wp[1]);

          printf("-----------------------\n");

          ptpx.push_back(prev_wp0[0]);
          ptpx.push_back(prev_wp1[0]);
          ptpx.push_back(ref_x);
          ptpx.push_back(next_wp0[0]);
          ptpx.push_back(next_wp1[0]);
          ptpx.push_back(next_wp2[0]);

          ptpy.push_back(prev_wp0[1]);
          ptpy.push_back(prev_wp1[1]);
          ptpy.push_back(ref_y);
          ptpy.push_back(next_wp0[1]);
          ptpy.push_back(next_wp1[1]);
          ptpy.push_back(next_wp2[1]);
          global2local_coord_conversion(ptpx, ptpy, ref_x, ref_y, ref_yaw);
          printf(" ptpx0: %lf, ptpy0: %lf\n", ptpx[0], ptpy[0]);
          printf(" ptpx1: %lf, ptpy1: %lf\n", ptpx[1], ptpy[1]);
          printf(" ptpx2: %lf, ptpy2: %lf\n", ptpx[2], ptpy[2]);
          printf(" ptpx3: %lf, ptpy3: %lf\n", ptpx[3], ptpy[3]);
          printf(" ptpx4: %lf, ptpy4: %lf\n", ptpx[4], ptpy[4]);
          vector<vector<double>> local_frenet;
          double tmp_yaw;
          for (int i = 0; i < ptpx.size() - 1; i++)
          {
            tmp_yaw = atan2(ptpy[i + 1] - ptpy[i], ptpx[i + 1] - ptpx[i]);
            printf("tmp_yaw: %lf\n", tmp_yaw);
            vector<double> ptp_frenet = getFrenet(ptpx[i], ptpy[i], tmp_yaw, ptpx, ptpy);
            local_frenet.push_back(ptp_frenet);
            //printf("ptp_fs: %lf ptp_fd: %lf\n", ptp_frenet[0], ptp_frenet[1]);
          }

          for (int i = 0; i < local_frenet.size(); i++)
          {
            vector<double> tmp = local_frenet[i];
            
          }
          printf("----------------------------\n");
          double passing_idx = 50 - previous_path_x.size();
          double car_sdot = (car_s - pre_car_s) / (0.02 * passing_idx);
          double car_sdotdot = (car_sdot - pre_car_sdot) / (0.02 * passing_idx);
          double car_ddot = (car_d - pre_car_d) / (0.02 * passing_idx);
          double car_ddotdot = (car_ddot - pre_car_ddot) / (0.02 * passing_idx);


          // Generate polynomial function
          double poly_gen_time = 4;
          vector<double> start_s = {local_frenet[2][0], car_sdot, car_sdotdot};
          vector<double> start_d = {local_frenet[2][1], car_ddot, car_ddotdot};
          vector<double> goal_s = {local_frenet[3][0], car_sdot, 0};
          vector<double> goal_d = {local_frenet[3][1], 0, 0};
          printf("start_s: %lf start_d: %lf\n", local_frenet[2][0], local_frenet[2][1]);
          printf("goal_s: %lf goal_d: %lf\n", local_frenet[3][0], local_frenet[3][1]);
          // printf("tmp_s: %lf tmp_d: %lf\n", tmp[0], tmp[1]);
          // printf("tmp_s: %lf tmp_d: %lf\n", tmp[0], tmp[1]);

          vector<double> s_coeff = JMT(start_s, goal_s, poly_gen_time);
          vector<double> d_coeff = JMT(start_d, goal_d, poly_gen_time);

          // vector<double> next_x_vals_poly;
          // vector<double> next_y_vals_poly;
          // double start_x = ptpx[1];
          // double start_y = ptpy[1];
          // double goal_x = ptpx[2];
          // double goal_y = ptpy[2];
          // printf(" ptpx0: %lf, ptpy0: %lf\n", ptpx[0], ptpy[0]);
          // printf(" ptpx1: %lf, ptpy1: %lf\n", ptpx[1], ptpy[1]);
          // printf(" ptpx2: %lf, ptpy2: %lf\n", ptpx[2], ptpy[2]);
          // global2local_coord_conversion(ptpx, ptpy, ref_x, ref_y, ref_yaw);
          // double start_yaw = atan2(ptpy[2] - ptpy[1], ptpx[2] - ptpx[1]);
          // double goal_yaw = atan2(ptpy[3] - ptpy[2], ptpx[3] - ptpx[2]);
          // printf("start yaw: %lf\n", start_yaw);
          // printf("goal_yaw: %lf\n", goal_yaw);
          // vector<double> start_frenet = getFrenet(start_x, start_y, start_yaw, ptpx, ptpy);
          // vector<double> goal_frenet = getFrenet(goal_x, goal_y, goal_yaw, ptpx, ptpy);

          // // Since simulator passing just 1 index, it can be passing 3, or 4 index!
          // // We need to calculate how much passing index
          // double passing_idx = 50 - previous_path_x.size();
          // double car_sdot = (car_s - pre_car_s) / (0.02 * passing_idx);
          // double car_sdotdot = (car_sdot - pre_car_sdot) / (0.02 * passing_idx);
          // double car_ddot = (car_d - pre_car_d) / (0.02 * passing_idx);
          // double car_ddotdot = (car_ddot - pre_car_ddot) / (0.02 * passing_idx);
          // // printf("1. car_sdot: %lf, car_sdotdot: %lf, car_ddot: %lf, car_ddotdot: %lf\n",car_sdot,car_sdotdot,car_ddot,car_ddotdot);
          // // printf("2. car_s: %lf, pre_car_s: %lf, car_d: %lf, pre_car_d: %lf\n",car_s, pre_car_s,car_d, pre_car_d);
          // pre_car_s = car_s;
          // pre_car_sdot = car_sdot;
          // pre_car_d = car_d;
          // pre_car_ddot = car_ddot;

          // //coordinate move: start s -> 0, d->0
          // double local_start_car_s = start_frenet[0] - start_frenet[0];
          // double local_start_car_d = start_frenet[1] - start_frenet[1];
          // double local_goal_car_s = goal_frenet[0] - start_frenet[0];
          // double local_goal_car_d = goal_frenet[1] - start_frenet[1];

          // // Generate polynomial function
          // double poly_gen_time = 4;
          // vector<double> start_s = {local_start_car_s, car_sdot, car_sdotdot};
          // vector<double> start_d = {local_start_car_d, car_ddot, car_ddotdot};
          // vector<double> goal_s = {local_goal_car_s, car_sdot, 0};
          // vector<double> goal_d = {local_goal_car_d, 0, 0};

          // vector<double> s_coeff = JMT(start_s, goal_s, poly_gen_time);
          // vector<double> d_coeff = JMT(start_d, goal_d, poly_gen_time);

          // for (int i = 0; i < previous_path_x.size(); i++)
          // {
          //   next_x_vals_poly.push_back(previous_path_x[i]);
          //   next_y_vals_poly.push_back(previous_path_y[i]);
          // }
          // double t_add_on = 0;
          // printf("car_s: %lf \n", car_s);
          // for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          // {
          //   double t_inc = 0.02;
          //   t_add_on = t_add_on + t_inc;
          //   double next_s = CalPoly(s_coeff, t_add_on);
          //   double next_d = CalPoly(d_coeff, t_add_on);
          //   next_s = next_s + start_frenet[0];
          //   next_d = next_d + start_frenet[1];

          //   printf("next_s: %lf next_d: %lf \n", next_s, next_d);
          // }
          // printf("------------------------------\n");

}
vector<vector<double>> spline_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double ref_vel, int lane,
                                                    nlohmann::json previous_path_x, nlohmann::json previous_path_y,
                                                    vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    vector<double> ptsx;
    vector<double> ptsy;
    double car_speed;
    int prev_size = previous_path_x.size();

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // ptsx[0],ptsx[1] are for tangent of previous path
    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(previous_path_x[prev_size - 2]);
        ptsx.push_back(previous_path_x[prev_size - 1]);
        ptsy.push_back(previous_path_y[prev_size - 2]);
        ptsy.push_back(previous_path_y[prev_size - 1]);
        car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / .02) * 2.237;
    }
    // ptsx[2],[3],[4] are netx wp goal
    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // call by reference, global 2 local coordinate conversion
    global2local_coord_conversion(ptsx, ptsy, ref_x, ref_y, ref_yaw);

    // spline fitting
    tk::spline s;
    s.set_points(ptsx, ptsy);
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {

        if (ref_vel > car_speed)
        {
            car_speed += .224;
        }
        else if (ref_vel < car_speed)
        {
            car_speed -= .224;
        }

        double N = (target_dist / (.02 * car_speed / 2.24));
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

// void local2global_coord_conversion()
// {
//                 x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//             y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

//             x_point += ref_x;
//             y_point += ref_y;
// }

double speed_planning()
{
    
}