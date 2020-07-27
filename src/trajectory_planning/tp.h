#ifndef LPP_H
#define LPP_H

#include <cmath>
#include <iostream>
#include "../util/Eigen-3.3/Eigen/Dense"
#include <vector>
#include "../util/helpers.h"
#include "../util/spline.h"
#include "../util/json.hpp"
#include "../behavior_planning/bp.h"
using nlohmann::json;
using std::vector;

#define NUMTRJSIZE 50

class TrajectoryPlanning
{

public:
    TrajectoryPlanning(double car_x_, double car_y_, double car_yaw_, double car_s_, double ref_vel_, int lane_,
                       nlohmann::json previous_path_x_, nlohmann::json previous_path_y_,
                       vector<double> map_waypoints_s_, vector<double> map_waypoints_x_, vector<double> map_waypoints_y_);

    //cubic spline based trajectory
    vector<vector<double>> spline_trajectory_generation(void);

private:
    double car_x; // m
    double car_y; // m
    double car_yaw;
    double car_s;   // m
    double ref_vel; // m/s
    int lane;
    nlohmann::json previous_path_x;
    nlohmann::json previous_path_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
};

class QuinticPolynomial
{
public:
    QuinticPolynomial(vector<double> _start, vector<double> _end, double _T);

    double cal_poly(double t);
    double cal_first_derivative(double t);
    double cal_second_derivative(double t);
    double cal_third_derivative(double t);

private:
    vector<double> start;
    vector<double> end;
    vector<double> coeff;
    double T;
};

vector<double> JMT(vector<double> &start, vector<double> &end, double T);
// vector<double> PTG(vector<double> start_s,
//                    vector<double> start_d,
//                    Vehicle target_vehicle,
//                    vector<double> delta,
//                    double T);
double CalQuintic(vector<double> coeff, double T);
vector<vector<double>> spline_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double ref_vel, int lane,
                                                    nlohmann::json previous_path_x, nlohmann::json previous_path_y,
                                                    vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
vector<vector<double>> global2local_coord_conversion(vector<double> global_x, vector<double> global_y, double cur_x, double cur_y, double cur_yaw);
vector<vector<double>> quintic_polynomial_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double car_d, double ref_vel, int lane, double end_path_s, double end_path_d,
                                                                nlohmann::json previous_path_x, nlohmann::json previous_path_y, double car_speed,
                                                                vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, int behavior);
vector<vector<double>> crop_local_map(double car_x, double car_y, double car_yaw, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, vector<double> map_waypoints_s);
vector<double> interp_based_interval(vector<double> x, vector<double> y,
                                     double interval, int output_size);
vector<double> interp_based_eval_x(vector<double> x, vector<double> y,
                                   vector<double> eval_at_x);
#endif // LPP_H