#ifndef BP_H
#define BP_H

#include <cmath>
#include <iostream>
#include "json.hpp"
#define LANE_WIDTH 4
using nlohmann::json;
enum BehaviorSyntax
{
    kStraight = 0,
    kRightLaneChange = 1,
    kLeftLaneChange = 2,
};
enum LaneList
{
    kLane1 = 0,
    kLane2 = 1,
    kLane3 = 2,    
    kLaneOut = 3,      // Lane out means that vehicle is out of lane 1, 2, 3. for example, it is in left of the center lane
};

enum CloseVehicleSyntax
{
    kFar = 0,
    kClose = 1,
};
int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d);
int check_safety_lane(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane);
int get_next_lane(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane);
int get_lane(double d);
#endif // BP_H