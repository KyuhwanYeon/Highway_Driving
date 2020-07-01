#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "lpp.h"
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          static double pre_car_s;
          static double pre_car_sdot;
          static double pre_car_d;
          static double pre_car_ddot;
          //printf("car_s: %lf" , car_s);
          

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: 
           * 1. Spline path planning generation
           * 2. polynomial path generation 
           */
          // ptsx, ptsy are for spline path
          // ptpx, ptpy are for polynomial path
          int lane = 1;
          double ref_vel = 49.5; //mph
          int next_wp = -1;
          int time_to_goal = 1;


          vector<vector<double>> spline_trajectory =  spline_trajectory_generation(car_x, car_y, car_yaw, car_s, ref_vel, lane,
                                                   previous_path_x, previous_path_y,
                                                      map_waypoints_s, map_waypoints_x,  map_waypoints_y);


          
          int prev_size = previous_path_x.size();
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          // polynomial generation
                    // Add polynomial start point and goal point
          vector<double> ptpx;
          vector<double> ptpy;
          vector<double> next_wp_polynomial0 = getXY(car_s, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_polynomial1 = getXY(car_s + time_to_goal * car_speed, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_polynomial2 = getXY(car_s + 2 * time_to_goal * car_speed, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptpx.push_back(next_wp_polynomial0[0]);
          ptpy.push_back(next_wp_polynomial0[1]);
          ptpx.push_back(ref_x);
          ptpy.push_back(ref_y);
          ptpx.push_back(next_wp_polynomial1[0]);
          ptpy.push_back(next_wp_polynomial1[1]);          
          ptpx.push_back(next_wp_polynomial2[0]);
          ptpy.push_back(next_wp_polynomial2[1]);              
          vector<double> next_x_vals_poly;
          vector<double> next_y_vals_poly;
          double start_x = ptpx[1];
          double start_y = ptpy[1];
          double goal_x = ptpx[2];
          double goal_y = ptpy[2];
          printf(" ptpx0: %lf, ptpy0: %lf\n", ptpx[0], ptpy[0]);
          printf(" ptpx1: %lf, ptpy1: %lf\n", ptpx[1], ptpy[1]);
          printf(" ptpx2: %lf, ptpy2: %lf\n", ptpx[2], ptpy[2]);
          global2local_coord_conversion(ptpx, ptpy, ref_x, ref_y, ref_yaw);
          double start_yaw = atan2(ptpy[2] - ptpy[1], ptpx[2] - ptpx[1]);
          double goal_yaw = atan2(ptpy[3] - ptpy[2], ptpx[3] - ptpx[2]);
          printf("start yaw: %lf\n", start_yaw);
          printf("goal_yaw: %lf\n", goal_yaw);
          vector<double> start_frenet = getFrenet(start_x, start_y,start_yaw,ptpx,ptpy );
          vector<double> goal_frenet = getFrenet(goal_x, goal_y,goal_yaw,ptpx,ptpy );
          
          // Since simulator passing just 1 index, it can be passing 3, or 4 index!
          // We need to calculate how much passing index
          double passing_idx = 50 - previous_path_x.size();
          double car_sdot = (car_s - pre_car_s)/(0.02*passing_idx);
          double car_sdotdot = (car_sdot - pre_car_sdot)/(0.02*passing_idx);
          double car_ddot = (car_d-pre_car_d)/(0.02*passing_idx);
          double car_ddotdot = (car_ddot - pre_car_ddot)/(0.02*passing_idx);
          // printf("1. car_sdot: %lf, car_sdotdot: %lf, car_ddot: %lf, car_ddotdot: %lf\n",car_sdot,car_sdotdot,car_ddot,car_ddotdot);
          // printf("2. car_s: %lf, pre_car_s: %lf, car_d: %lf, pre_car_d: %lf\n",car_s, pre_car_s,car_d, pre_car_d);
          pre_car_s = car_s;
          pre_car_sdot = car_sdot;
          pre_car_d = car_d;
          pre_car_ddot = car_ddot;

          //coordinate move: start s -> 0, d->0
          double local_start_car_s = start_frenet[0]-start_frenet[0];
          double local_start_car_d = start_frenet[1]-start_frenet[1];
          double local_goal_car_s = goal_frenet[0]-start_frenet[0];
          double local_goal_car_d = goal_frenet[1]-start_frenet[1];

          // Generate polynomial function
          double poly_gen_time = 4;
          vector<double> start_s ={local_start_car_s, car_sdot, car_sdotdot};
          vector<double> start_d ={local_start_car_d, car_ddot, car_ddotdot};
          vector<double> goal_s = {local_goal_car_s, car_sdot, 0};
          vector<double> goal_d = {local_goal_car_d, 0, 0};


          vector<double> s_coeff = JMT(start_s,goal_s,poly_gen_time);
          vector<double> d_coeff = JMT(start_d,goal_d,poly_gen_time);


          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals_poly.push_back(previous_path_x[i]);
            next_y_vals_poly.push_back(previous_path_y[i]);
          }
          double t_add_on = 0;
          printf("car_s: %lf \n", car_s);
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double t_inc = 0.02;
            t_add_on = t_add_on + t_inc;
            double next_s = CalPoly(s_coeff,t_add_on);
            double next_d = CalPoly(d_coeff, t_add_on);
            next_s = next_s + start_frenet[0];
            next_d = next_d + start_frenet[1];

            
            printf("next_s: %lf next_d: %lf \n", next_s, next_d);
          }
          printf("------------------------------\n");

          msgJson["next_x"] = spline_trajectory[0];
          msgJson["next_y"] = spline_trajectory[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}