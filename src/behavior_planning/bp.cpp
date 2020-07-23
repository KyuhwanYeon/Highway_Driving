#include "bp.h"

#define MINIMUM_DIST 30 // Lane change will be start at minimum distance \
                        // Check close obstacle on the ego lane

void BehaviorPlanning::cal_close_obs_status(void)
{
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
                close_obs_v = obs_v * 2.237;
                close_status = kClose;
                printf("- Lane change\n");
                break;
            }
        }
        else
        {
            close_status = kFar;
        }
    }
}
void BehaviorPlanning::cal_safe_lane(void)
{
    vector<int> unsafe_lane_list;
    int right_lane = cur_lane + 1;
    int left_lane = cur_lane - 1;

    int cnt_unsafe_left_lane = 0;
    int cnt_unsafe_right_lane = 0;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float obs_s = sensor_fusion[i][5];
        float obs_d = sensor_fusion[i][6];

        if (obs_s - car_s < MINIMUM_DIST + 5 && car_s - obs_s < MINIMUM_DIST - 25)
        {
            unsafe_lane_list.push_back(cal_lane(obs_d));
        }
    }
    for (int unsafe_lane : unsafe_lane_list)
    {

        if (unsafe_lane == right_lane)
        {
            cnt_unsafe_right_lane++;
        }
        if (unsafe_lane == left_lane)
        {
            cnt_unsafe_left_lane++;
        }
    }
    if (cnt_unsafe_left_lane > 0)
    {
        printf("- Left lane is unsafe\n");
    }
    if (cnt_unsafe_right_lane > 0)
    {
        printf("- Right lane is unsafe\n");
    }

    if (close_status == kClose)
    {
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
    }
    else
    {
        safe_lane = cur_lane;
    }
    if (safe_lane == left_lane)
    {
        behavior = kLaneChangeLeft;
    }
    else if (safe_lane == right_lane)
    {
        behavior = kLaneChangeRight;
    }
    else
    {
        behavior = kKeepLane;
    }
    
    printf("- safe_lane: %d \n\n", safe_lane + 1);
}
void BehaviorPlanning::obtain_behavior(void)
{
    cal_close_obs_status();
    cal_safe_lane();

    if (close_status == kClose)
    {
        target_lane = safe_lane;
        target_vel = close_obs_v;
    }
    else
    {
        target_lane = cur_lane;
        target_vel = 49.5;
    }
}

double BehaviorPlanning::get_target_vel(void)
{
    return target_vel;
}
int BehaviorPlanning::get_target_lane(void)
{
    return target_lane;
}
int BehaviorPlanning::get_behavior(void)
{
    return behavior;
}

