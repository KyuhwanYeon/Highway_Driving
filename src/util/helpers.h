#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

#define LANE_WIDTH 4
#define SAMPLING_T 0.02
// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x) ;
double mph2ms(double x);
double ms2mph(double x);
// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) ;
vector<double> getXY_local(double s, double d, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) ;                     
int cal_lane(double d);

enum LaneList
{
    kLane1 = 0,
    kLane2 = 1,
    kLane3 = 2,
    kLaneOut = 3, // Lane out means that vehicle is out of lane 1, 2, 3. for example, it is in left of the center lane
};


#endif  // HELPERS_H