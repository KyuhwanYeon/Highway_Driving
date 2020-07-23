#include "pred.h"
vector<vector<double>> Prediction::generate_predictions(int horizon)
{
    // Generate prediction based on constant velocity model
    vector<double> pred_s;
    vector<double> pred_d;
    for (int i = 0; i < horizon; ++i)
    {
        double next_s = SAMPLING_T*v;
        double next_d = 0;
        pred_s.push_back(next_s);
        pred_d.push_back(next_d);


    }
    return {pred_s,pred_d};
}
