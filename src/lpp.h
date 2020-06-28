#ifndef LPP_H
#define LPP_H

#include <cmath>
#include <iostream>
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

using std::vector;
class Vehicle
{
    // constant acceleration vehicle
public:
    Vehicle(vector<double> start)  {start_state = start;};
    vector<double> state_in(double t);
private:
    vector<double> start_state;
    
};

vector<double> JMT(vector<double> &start, vector<double> &end, double T);
vector<double> PTG(vector<double> start_s,
                   vector<double> start_d,
                   Vehicle target_vehicle,
                   vector<double> delta,
                   double T
                   );


#endif // LPP_H