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

void quintic_polynomial_trajectory_generation()
{
}
vector<vector<double>> spline_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double ref_vel, int lane,
                                                    nlohmann::json previous_path_x, nlohmann::json previous_path_y,
                                                     vector<double> map_waypoints_s,vector<double> map_waypoints_x, vector<double> map_waypoints_y)
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