#include "bp.h"
// for convenience
using std::string;
using std::vector;

int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d)
{
    int minimum_distance = 30;
    int close_status;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float obs_s = sensor_fusion[i][5];
        float obs_d = sensor_fusion[i][6];
        
        if (fabs(car_d-obs_d) <2)
        {
            if (obs_s - car_s <minimum_distance && obs_s - car_s > 0)
            {
                close_status = kClose;
                break;
            }
        }
        printf("obstacle %d, s:%f,  d: %f\n", i, obs_s - car_s, car_d-obs_d);
    }
    return close_status;
}
int check_safety_lane()
{

}
int set_lane()
{
}