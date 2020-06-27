#ifndef LPP_H
#define LPP_H

#include <cmath>
#include <iostream>
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

using std::vector;
vector<double> JMT(vector<double> &start, vector<double> &end, double T);


class Vehicle
{
    // constant acceleration vehicle
public:
    Vehicle(double *start) : start_state{(*start)}{};

private:
    double start_state[6];
    double *state_in(double t);
};

#endif // LPP_H