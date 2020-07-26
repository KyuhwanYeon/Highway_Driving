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
    double lane_change_duration = 4; // time take to lane change [s]
    double car_speed;                // [m/s]
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
        car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / SAMPLING_T); // m/s
    }
    // ptsx[2],[3],[4] are netx wp goal

    double passing_idx = NUMTRJSIZE - previous_path_x.size();
    double car_sdot = (car_s - pre_car_s) / (SAMPLING_T * passing_idx);
    if (car_sdot < 5)
    {
        car_sdot = 5; // threshold
    }
    pre_car_s = car_s;

    vector<double> next_wp0 = getXY(car_s + offset_s + car_sdot * lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 2 * offset_s + car_sdot * lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 3 * offset_s + car_sdot * lane_change_duration, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    grid_x.push_back(next_wp0[0]);
    grid_x.push_back(next_wp1[0]);
    grid_x.push_back(next_wp2[0]);

    grid_y.push_back(next_wp0[1]);
    grid_y.push_back(next_wp1[1]);
    grid_y.push_back(next_wp2[1]);

    // call by reference, global 2 local coordinate conversion
    // global2local_coord_conversion(grid_x, grid_y, ref_x, ref_y, ref_yaw);
    vector<vector<double>> local_xy = global2local_coord_conversion(grid_x, grid_y, ref_x, ref_y, ref_yaw);
    vector<double> local_x = local_xy[0];
    vector<double> local_y = local_xy[1];
    // spline fitting
    tk::spline s;
    s.set_points(local_x, local_y);
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 2 * offset_s + car_sdot * lane_change_duration; //offset_s+car_speed*lane_change_duration;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;

    for (int i = 1; i <= NUMTRJSIZE - previous_path_x.size(); i++)
    {

        if (ref_vel > car_speed)
        {
            car_speed += 0.1;
        }
        else if (ref_vel < car_speed)
        {
            car_speed -= 0.1;
        }

        double N = (target_dist / (SAMPLING_T * car_speed));
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
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    MatrixXd A = MatrixXd(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;

    MatrixXd B = MatrixXd(3, 1);
    B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai * B;

    vector<double> result = {start[0], start[1], .5 * start[2]};

    for (int i = 0; i < C.size(); ++i)
    {
        result.push_back(C.data()[i]);
    }

    return result;
}

double CalQuintic(vector<double> coeff, double T)
{
    double result = coeff[0] + coeff[1] * T + coeff[2] * T * T + coeff[3] * T * T * T + coeff[4] * T * T * T * T + coeff[5] * T * T * T * T * T;
    return result;
}
vector<vector<double>> global2local_coord_conversion(vector<double> global_x, vector<double> global_y, double cur_x, double cur_y, double cur_yaw)
{
    vector<double> local_x;
    vector<double> local_y;
    // local coorinate conversion, after shift, ref_x and ref_y is coordinate (0, 0)
    for (int i = 0; i < global_x.size(); i++)
    {
        //shift car reference angle to 0 degrees
        double shift_x = global_x[i] - cur_x;
        double shift_y = global_y[i] - cur_y;
        local_x.push_back(shift_x * cos(0 - cur_yaw) - shift_y * sin(0 - cur_yaw));
        local_y.push_back(shift_x * sin(0 - cur_yaw) + shift_y * cos(0 - cur_yaw));
    }
    return {local_x, local_y};
}
vector<vector<double>> quintic_polynomial_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double car_d, double ref_vel, int lane, double end_path_s, double end_path_d,
                                                                nlohmann::json previous_path_x, nlohmann::json previous_path_y, double car_speed,
                                                                vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, int behavior)
{
    ref_vel =mph2ms(ref_vel);
    // 1. crop global map to local map to reduce the map size
    printf("car_yaw: %lf \n ", car_yaw);
    vector<vector<double>> crop_map = crop_local_map(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s);
    vector<double> crop_x = crop_map[0];
    vector<double> crop_y = crop_map[1];
    vector<double> crop_s = crop_map[2];
    vector<double> crop_dx = crop_map[3];
    vector<double> crop_dy = crop_map[4];
    // 2. interpolate the crop map.  local map = interpolated crop map
    double dist_inc = 0.01;
    int num_inter_points = (crop_s[crop_s.size() - 1] - crop_s[0]) / dist_inc;
    vector<double> local_map_s, local_map_x, local_map_y, local_map_dx, local_map_dy;
    local_map_s.push_back(crop_s[0]);
    for (int i = 1; i < num_inter_points; i++)
    {
        local_map_s.push_back(crop_s[0] + i * dist_inc);
    }
    // for (int i = 0; i < crop_s.size(); i++)
    // {
    //     printf("crop_s: %lf \n ", crop_s[i]);
    // }

    local_map_x = interp_based_interval(crop_s, crop_x, dist_inc, num_inter_points);
    local_map_y = interp_based_interval(crop_s, crop_y, dist_inc, num_inter_points);
    local_map_dx = interp_based_interval(crop_s, crop_dx, dist_inc, num_inter_points);
    local_map_dy = interp_based_interval(crop_s, crop_dy, dist_inc, num_inter_points);
    printf("22222222222222222222222222222\n");

    // 3. Generate candidate trajectories

    // 4. Choose best trajectory

    int prev_size = previous_path_x.size();
    double ref_s_prev;
    double ref_x, ref_y, ref_yaw, ref_s, ref_d;
    double ref_acc_x, ref_acc_y, ref_vel_x, ref_vel_x_prev, ref_vel_y, ref_vel_y_prev, ref_x_prev, ref_x_prev_prev, ref_y_prev, ref_y_prev_prev;
    double s_dot, s_ddot, d_dot, d_ddot;
    double dx, dy, sx, sy;
    // polynomial generation
    // Add polynomial start point and goal point
    // ptsx[0],ptsx[1] are for tangent of previous path
    if (prev_size < 4)
    {
        ref_x = car_x;
        ref_y = car_y;
        ref_s = car_s;
        ref_d = car_d;
        ref_yaw = deg2rad(car_yaw);
        ref_s_prev = ref_s - 1;
        ref_x_prev = ref_x - 1 * cos(ref_yaw);
        ref_y_prev = ref_y - 1 * sin(ref_yaw);
        s_dot = 0;
        d_dot = 0;
        s_ddot = 0;
        d_ddot = 0;
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_x_prev = previous_path_x[prev_size - 2];
        ref_x_prev_prev = previous_path_x[prev_size - 3];
        ref_y = previous_path_y[prev_size - 1];
        ref_y_prev = previous_path_y[prev_size - 2];
        ref_y_prev_prev = previous_path_y[prev_size - 3];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        vector<double> frenet = getFrenet(ref_x, ref_y, ref_yaw, local_map_x, local_map_y);
        //vector<double> frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y, map_waypoints_s);
        ref_s = frenet[0];
        ref_d = frenet[1];
        // calculate s_dot & d_dot
        ref_vel_x = (ref_x - ref_x_prev) / SAMPLING_T;
        ref_vel_y = (ref_y - ref_y_prev) / SAMPLING_T;
        // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
        // and D are unit vectors this is simply the dot products of V with S and V with D
        dx = local_map_dx[num_inter_points - 1];
        dy = local_map_dy[num_inter_points - 1];
        // sx,sy vector is perpendicular to dx,dy
        sx = -dy;
        sy = dx;
        s_dot = ref_vel_x * sx + ref_vel_y * sy;
        d_dot = ref_vel_x * dx + ref_vel_y * dy;
        ref_vel_x_prev = (ref_x_prev - ref_x_prev_prev) / SAMPLING_T;
        ref_vel_y_prev = (ref_y_prev - ref_y_prev_prev) / SAMPLING_T;
        ref_acc_x = (ref_vel_x - ref_vel_x_prev) / SAMPLING_T;
        ref_acc_y = (ref_vel_y - ref_vel_y_prev) / SAMPLING_T;
        s_ddot = ref_acc_x * sx + ref_acc_y * sy;
        d_ddot = ref_acc_x * dx + ref_acc_y * dy;

        ref_s_prev = ref_s - s_dot * SAMPLING_T;
    }
    printf("car_s: %lf, ref_s: %lf, car_d: %lf, ref_d: %lf\n", car_s, ref_s, car_d, ref_d);
    
    // if (car_sdot <5)
    // {
    //     car_sdot = 5; // threshold
    // }

    // // Generate polynomial function
    double goal_sdot = 10;
    double trajectory_duration = 4;
    double duration = NUMTRJSIZE * SAMPLING_T - prev_size * SAMPLING_T;

    double target_v = s_dot;
    if (ref_vel > s_dot)
    {
        target_v += 0.1;
    }
    else if (ref_vel < s_dot)
    {
        target_v -= 0.1;
    }

    vector<double> start_s = {ref_s, s_dot, s_ddot};
    vector<double> start_d = {ref_d, d_dot, d_ddot};
    vector<double> goal_s = {ref_s + target_v * SAMPLING_T * (NUMTRJSIZE - prev_size), target_v, 0};
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
    goal_d = {(double)(2 + 4 * lane), 0, 0};
    vector<double> s_coeff = JMT(start_s, goal_s, duration);
    vector<double> d_coeff = JMT(start_d, goal_d, duration);
    // for (int i = 0; i< s_coeff.size(); i++)
    // {
    //     printf("s_coeff: %lf\n",s_coeff[i]);
    //     printf("d_coeff: %lf\n",d_coeff[i]);

    // }

    printf("duration: %lf, ref_d: %lf , goal_d: %lf, lane: %d\n", duration, ref_d, (double)(2 + 4 * lane), lane);

    vector<double> best_s_trajectory, best_d_trajectory, best_x_trajectory, best_y_trajectory;
    for (double t = 1; t <= duration; t ++)
    {
        double next_s = CalQuintic(s_coeff, t);
        double next_d = CalQuintic(d_coeff, t);
        // printf("next_s: %lf, next_d: %lf \n", next_s, next_d);
        best_s_trajectory.push_back(next_s);
        best_d_trajectory.push_back(next_d);
    }

    printf("------------------------------------------------- \n");
    printf("car_x: %lf , car_y: %lf, car_s: %lf \n", car_x, car_y, car_s);
    printf("ref_x: %lf , ref_y: %lf, ref_s: %lf \n", ref_x, ref_y, ref_s);
    printf("------------------------------------------------- \n");

    for (int i = 0; i < best_s_trajectory.size(); i++)
    {
        vector<double> best_xy_trajectory = getXY(best_s_trajectory[i], best_d_trajectory[i], local_map_s, local_map_x, local_map_y);
        best_x_trajectory.push_back(best_xy_trajectory[0]);
        best_y_trajectory.push_back(best_xy_trajectory[1]);
        printf("best_x: %lf , best_y: %lf, best_s: %lf, best_d: %lf \n", best_x_trajectory[i], best_y_trajectory[i], best_s_trajectory[i], best_d_trajectory[i]);
    }
    printf("estimate car yaw: %lf \n", atan2(best_y_trajectory[1] - car_y, best_x_trajectory[0] - car_x));
    // first two points of coarse trajectory, to ensure spline begins smoothly
    vector<double> trajectory_s;
    vector<double> trajectory_x;
    vector<double> trajectory_y;

    // trajectory_s.push_back(ref_s_prev);
    // trajectory_x.push_back(ref_x_prev);
    // trajectory_y.push_back(ref_y_prev);
    // trajectory_s.push_back(ref_s);
    // trajectory_x.push_back(ref_x);
    // trajectory_y.push_back(ref_y);
    // for (int i = 0; i < trajectory_s.size(); i++)
    // {
    //     printf("trajectory_s: %lf\n", trajectory_s[i]);
    // }
    printf("-------------------------------------------------\n");
    for (int i = 0; i < best_x_trajectory.size(); i++)
    {
        trajectory_x.push_back(best_x_trajectory[i]);
        trajectory_y.push_back(best_y_trajectory[i]);
        trajectory_s.push_back(best_s_trajectory[i]);
    }

    // for (int i = 0; i < trajectory_s.size(); i++)
    // {
    //     printf("trajectory_s: %lf\n", trajectory_s[i]);
    // }

    double target_s_dot = ref_vel;
    double current_s = ref_s;
    double current_v = s_dot;
    double current_a = s_ddot;
    vector<double> interp_s_traj, interp_x_traj, interp_y_traj;

    for (int i = 0; i < NUMTRJSIZE; i++)
    {

        if (ref_vel > current_v)
        {
            current_v += 0.1;
        }
        else if (ref_vel < current_v)
        {
            current_v -= 0.1;
        }

        current_s += current_v * SAMPLING_T;
        interp_s_traj.push_back(current_s);

        // // DEBUG
        // cout << a_incr << "\t\t" << current_a << "\t\t" << v_incr << "\t\t" << current_v << "\t\t" << interpolated_s_traj[i] << endl;
    }

    for (int i = 0; i < interp_s_traj.size(); i++)
    {
        //  printf("interp_s_traj: %lf \n", interp_s_traj[i]);
    }
    printf("4\n");
    interp_x_traj = interp_based_eval_x(trajectory_s, trajectory_x, interp_s_traj);
    interp_y_traj = interp_based_eval_x(trajectory_s, trajectory_y, interp_s_traj);
    for (int i = 0; i < interp_s_traj.size(); i++)
    {
        printf("interp_x: %lf,   interp_y: %lf,   interp_s: %lf \n", interp_x_traj[i], interp_y_traj[i], interp_s_traj[i]);
    }

    printf("5\n");
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    // add previous path, if any, to next path
    for (int i = 0; i < prev_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
        //   printf("next_x_vals: %lf\n", next_x_vals[i]);
    }
    // add xy points from newly generated path
    for (int i = 0; i < interp_x_traj.size(); i++)
    {
        //if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
        next_x_vals.push_back(best_x_trajectory[i]);
        next_y_vals.push_back(best_y_trajectory[i]);
        //  printf("next_x_vals: %lf\n", next_x_vals[i]);
    }
    return {next_x_vals, next_y_vals};
}

vector<vector<double>> crop_local_map(double car_x, double car_y, double car_yaw, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, vector<double> map_waypoints_s)
{
    const int NUM_PREV_WAYPNTS = 5;
    const int NUM_AHEAD_WAYPNTS = 5;
    int num_waypoints = map_waypoints_x.size();
    printf("car_x: %lf, car_y: %lf \n", car_x, car_y);
    int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
    printf("next_waypoint_index: %d \n", next_waypoint_index);
    vector<double> local_waypoints_s, local_waypoints_x, local_waypoints_y,
        local_waypoints_dx, local_waypoints_dy;
    for (int i = -NUM_PREV_WAYPNTS; i < NUM_AHEAD_WAYPNTS; i++)
    {
        // for smooting, take so many previous and so many subsequent waypoints
        int idx = (next_waypoint_index + i) % num_waypoints;
        // printf("idx: %d \n", idx);
        // correct for wrap in s for spline interpolation (must be continuous)
        local_waypoints_s.push_back(map_waypoints_s[idx]);
        local_waypoints_x.push_back(map_waypoints_x[idx]);
        local_waypoints_y.push_back(map_waypoints_y[idx]);
        local_waypoints_dx.push_back(map_waypoints_dx[idx]);
        local_waypoints_dy.push_back(map_waypoints_dy[idx]);
    }
    return {local_waypoints_x, local_waypoints_y, local_waypoints_s, local_waypoints_dx, local_waypoints_dy};
}
vector<double> interp_based_interval(vector<double> x, vector<double> y,
                                     double interval, int output_size)
{
    // interpolation y according to x by using spline
    tk::spline s;
    s.set_points(x, y); // currently it is required that X is already sorted
    vector<double> output;
    for (int i = 0; i < output_size; i++)
    {
        output.push_back(s(x[0] + i * interval));
    }
    return output;
}

vector<double> interp_based_eval_x(vector<double> x, vector<double> y,
                                   vector<double> eval_at_x)
{
    // uses the spline library to interpolate points connecting a series of x and y values
    // output is spline evaluated at each eval_at_x point

    tk::spline s;
    s.set_points(x, y); // currently it is required that X is already sorted
    vector<double> output;
    for (double x : eval_at_x)
    {
        output.push_back(s(x));
    }
    return output;
}
