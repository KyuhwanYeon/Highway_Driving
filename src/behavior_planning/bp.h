#ifndef BP_H
#define BP_H

#include <cmath>
#include <iostream>
#include "../util/json.hpp"
#include "../util/helpers.h"
using nlohmann::json;
using std::string;
using std::vector;

class BehaviorPlanning
{

public:
    BehaviorPlanning(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane) : car_s(car_s), car_d(car_d), cur_lane(cur_lane), sensor_fusion(sensor_fusion), target_lane(cur_lane), target_vel(49.5), close_status(0), safe_lane(cur_lane){};
    void cal_close_obs_status(void);
    void cal_safe_lane(void);
    double get_target_vel(void);
    int get_target_lane(void);
    void obtain_behavior(void);  
    int get_behavior(void);  

private:

    double car_s; // m
    double car_d; // m
    int cur_lane;
    nlohmann::json sensor_fusion;
    int target_lane;
    double target_vel;
    int close_status;
    double close_obs_v;
    int safe_lane;
    int behavior;
    
};


enum CloseVehicleSyntax
{
    kFar = 0,
    kClose = 1,
};

enum BehaviorLists
{
    kKeepLane = 0,
    kLaneChangeLeft = 1,
    kLaneChangeRight,
};


// int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d);
// int check_safety_lane(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane);
// void next_ego_vehicle_status(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane);
// int get_lane(double d);

#endif // BP_H