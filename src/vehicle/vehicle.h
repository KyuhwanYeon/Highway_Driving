#ifndef VEHICLE_H
#define VEHICLE_H

#include <cmath>
#include <iostream>
#include "../util/Eigen-3.3/Eigen/Dense"
#include <vector>
#include "../util/helpers.h"
#include "../util/json.hpp"
using nlohmann::json;
using std::vector;

class Vehicle
{
public:
    Vehicle();
    Vehicle(int lane, float s,float d, float v, float a, string state = "CS");

private:
};

#endif