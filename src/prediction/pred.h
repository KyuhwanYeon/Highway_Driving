#ifndef PRED_H
#define PRED_H

#include <cmath>
#include <iostream>
#include "../util/Eigen-3.3/Eigen/Dense"
#include <vector>
#include "../util/helpers.h"
#include "../util/json.hpp"
#include "../vehicle/vehicle.h"
using nlohmann::json;
using std::vector;

class Prediction
{
public:
    Prediction(double pos_s, double pos_d, double vel) : s(pos_s), d(pos_d), v(vel){};
    vector<vector<double>> generate_predictions(int horizon);

private:
    double s;
    double d;
    double v;
};

#endif
