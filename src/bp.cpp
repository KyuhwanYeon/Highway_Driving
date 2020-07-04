#include "bp.h"

using std::string;
using std::vector;
#define MINIMUM_DIST 50
// Check close obstacle on the ego lane
int check_close_obstacle(nlohmann::json sensor_fusion, double car_s, double car_d)
{
    int close_status;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float obs_s = sensor_fusion[i][5];
        float obs_d = sensor_fusion[i][6];

        if (fabs(car_d - obs_d) < 2) // if obstacle is in the ego-lane
        {
            if (obs_s - car_s < MINIMUM_DIST && obs_s - car_s > 0)
            {
                close_status = kClose;
                printf("---------Lane change--------\n");
                break;
            }
        }
    }
    return close_status;
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

        if (obs_s - car_s < MINIMUM_DIST && car_s - obs_s < MINIMUM_DIST)
        {
            unsafe_lane_list.push_back(get_lane(obs_d));
        }
    }
    printf("cur_lane: %d\n", cur_lane);

    printf("-----------------------------------\n");
    for (int unsafe_lane : unsafe_lane_list)
    {
        if (unsafe_lane  == right_lane)
        {
            cnt_unsafe_right_lane++;
        }
        if (unsafe_lane  == left_lane)
        {
            cnt_unsafe_left_lane++;
        }
    }
    printf("cnt_unsafe_left_lane: %d\n", cnt_unsafe_left_lane);
    printf("cnt_unsafe_right_lane: %d\n", cnt_unsafe_right_lane);
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
    // else, vehicle can move right or left. but left has prioty
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
int get_next_lane(nlohmann::json sensor_fusion, double car_s, double car_d, int cur_lane)
{
    int next_lane;
    if (check_close_obstacle(sensor_fusion, car_s, car_d) == kClose)
    {
        next_lane = check_safety_lane(sensor_fusion, car_s, car_d, cur_lane);
    }
    else
    {
        next_lane = cur_lane;
    }

    return next_lane;
}