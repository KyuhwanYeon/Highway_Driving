#ifndef BP_H
#define BP_H

#include <cmath>
#include <iostream>
#include "json.hpp"
using nlohmann::json;
enum BehaviorSyntax
{
    kStraight = 0,
    kRightLaneChange = 1,
    kLeftLangeChange = 2,
};

enum CloseVehicleSyntax
{
    kFar = 0,
    kClose = 1,
};
int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d);
#endif // BP_H