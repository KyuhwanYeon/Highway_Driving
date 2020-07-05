#include "bp.h"

#define MINIMUM_DIST 30 // Lane change will be start at minimum distance
// Check close obstacle on the ego lane

double close_obs_v;
double ref_vel; //mph
int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d)
{
    int close_status;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float obs_s = sensor_fusion[i][5];
        float obs_d = sensor_fusion[i][6];
        double obs_vx = sensor_fusion[i][3];
        double obs_vy = sensor_fusion[i][4];
        double obs_v = sqrt(obs_vx * obs_vx + obs_vy * obs_vy);
        if (fabs(car_d - obs_d) < 2) // if obstacle is in the ego-lane
        {
            if (obs_s - car_s < MINIMUM_DIST && obs_s - car_s > 0)
            {
                close_obs_v = obs_v* 2.237;
                close_status = kClose;
                printf("------------Lane change------------\n");
                break;
            }
        }
    }
    return close_status;
}
int check_safety_lane(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane)
{
    vector<int> unsafe_lane_list;
    int safe_lane;
    int right_lane = cur_lane + 1;
    int left_lane = cur_lane - 1;

    int cnt_unsafe_left_lane = 0;
    int cnt_unsafe_right_lane = 0;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float obs_s = sensor_fusion[i][5];
        float obs_d = sensor_fusion[i][6];

        if (obs_s - car_s < MINIMUM_DIST + 20 && car_s - obs_s < MINIMUM_DIST-25)
        {
            unsafe_lane_list.push_back(get_lane(obs_d));
        }
    }    
    for (int unsafe_lane : unsafe_lane_list)
    {
        printf("unsafe_lane: %d, cur_lane: %d \n", unsafe_lane, cur_lane);
        if (unsafe_lane == right_lane)
        {
            cnt_unsafe_right_lane++;
        }
        if (unsafe_lane == left_lane)
        {
            cnt_unsafe_left_lane++;
        }
    }
    printf("-----------------------------------\n");
    // if current lane is lane 1, it only can move to right
    if (cur_lane == kLane1)
    {
        if (cnt_unsafe_right_lane == 0)
        {
            safe_lane = right_lane;
        }
        else
        {
            safe_lane = cur_lane;
        }
    }
    // if current lane is lane 3, it only can move to left
    else if (cur_lane == kLane3)
    {
        if (cnt_unsafe_left_lane == 0)
        {
            safe_lane = left_lane;
        }
        else
        {
            safe_lane = cur_lane;
        }
    }
    // else, vehicle can move right or left. but left has priority
    else
    {

        if (cnt_unsafe_left_lane == 0)
        {
            safe_lane = left_lane;
        }
        else if (cnt_unsafe_right_lane == 0)
        {
            safe_lane = right_lane;
        }
        else
        {
            safe_lane = cur_lane;
        }
    }
    printf("safe_lane: %d \n", safe_lane);
    printf("-----------------------------------\n");
    return safe_lane;
}
vector<double> next_ego_vehicle_status(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane)
{
    double next_vel;
    int next_lane;
    vector<double> next_status;
    if (check_close_obstacle(sensor_fusion, car_s, car_d) == kClose)
    {
        next_lane = check_safety_lane(sensor_fusion, car_s, car_d, cur_lane);
        next_vel = close_obs_v;
    }
    else
    {
        next_lane = cur_lane;
        next_vel = 49.5 ;
    }
    return {(double)next_lane, next_vel};
}
int get_lane(double d)
{
    int lane;
    if (d > 0 && d <= LANE_WIDTH)
    {
        lane = kLane1;
    }
    else if (d > LANE_WIDTH && d <= 2 * LANE_WIDTH)
    {
        lane = kLane2;
    }
    else if (d > 2 * LANE_WIDTH && d <= 3 * LANE_WIDTH)
    {
        lane = kLane3;
    }
    else
    {
        lane = kLaneOut;
    }
    return lane;
}

double get_ref_vel (void)
{
   
    return ref_vel;    
}