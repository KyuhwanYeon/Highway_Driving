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
    double car_s; // m
    double ref_vel; // m/s
    int lane;
    nlohmann::json previous_path_x;
    nlohmann::json previous_path_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
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
void global2local_coord_conversion(vector<double> &ptx, vector<double> &pty, double ref_x, double ref_y, double ref_yaw);

void quintic_polynomial_trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, double car_d, double ref_vel, int lane, double end_path_s, double end_path_d,
                                              nlohmann::json previous_path_x, nlohmann::json previous_path_y, double car_speed,
                                              vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int behavior);
#endif // LPP_H