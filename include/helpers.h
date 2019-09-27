#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;



// Max velocity
const double MAX_VEL = 49.0;

// gap vector index
const int FRONT = 0;
const int BACK = 1;
const int CURRENTLANE = 1;
const int LEFTLANE = 0;
const int RIGHTLANE = 2;

const double DANGERGAP = 0.0001;
const double MAXGAP = 200;
const double DANGERVELOCITY = 60.0;
const double DANGERCOST = 10000.0;

// Thresh for car in front and back that count as safe
const double SAFE_FRONT_GAP = 30.0;
const double SAFE_BACK_GAP = 15.0;

// Thresh for car in front and back that we should check
const double LOOKAHEAD_DISTANCE = 80.0;
const double LOOKABEHIND_DISTANCE = 30.0;

// Action distance check
const double ACTION_FRONT_DISTANCE = 50.0;
const double ACTION_BACK_DISTANCE = 20.0;

const double FOLLOW_DISTANCE = 15.0;

// next s point
const double DELTA_S = 45.0;

// the number of points to predict
const int NUM_PREDICT_PT = 50;


// // cost function parameters
// const double KEEP_MID_FACTOR = 0.35; //must be 0 < x < 1
// const double BACK_GAP_FACTOR = 0.4; // must be less than FRONT_GAP_FACTOR
// const double FRONT_GAP_FACTOR = 1.0;
// const double TURN_PENALTY_FACTOR = 1.4; // must be x > 1

// cost parameters
const double RELATIVE_VEL_COST = 1.0;
const double MAX_VEL_COST = 1.0;
const double DIS_COST = 100.0;

const double TIME_STEP = 0.02;
const double MS2MPH = 2.24; // 10m/s



enum class LaneType {NONE, LEFT, MID, RIGHT, INVALID};
enum class BehaviorType {PREP_CHANGE_LANE, LANECLEAR, FOLLOW, TURNLEFT, TURNRIGHT};



//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// // Calculate closest waypoint to current x, y position
// int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
//                     const vector<double> &maps_y) {
//   double closestLen = 100000; //large number
//   int closestWaypoint = 0;

//   for (int i = 0; i < maps_x.size(); ++i) {
//     double map_x = maps_x[i];
//     double map_y = maps_y[i];
//     double dist = distance(x,y,map_x,map_y);
//     if (dist < closestLen) {
//       closestLen = dist;
//       closestWaypoint = i;
//     }
//   }

//   return closestWaypoint;
// }

// // Returns next waypoint of the closest waypoint
// int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
//                  const vector<double> &maps_y) {
//   int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

//   double map_x = maps_x[closestWaypoint];
//   double map_y = maps_y[closestWaypoint];

//   double heading = atan2((map_y-y),(map_x-x));

//   double angle = fabs(theta-heading);
//   angle = std::min(2*pi() - angle, angle);

//   if (angle > pi()/2) {
//     ++closestWaypoint;
//     if (closestWaypoint == maps_x.size()) {
//       closestWaypoint = 0;
//     }
//   }

//   return closestWaypoint;
// }

// // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// vector<double> getFrenet(double x, double y, double theta, 
//                          const vector<double> &maps_x, 
//                          const vector<double> &maps_y) {
//   int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

//   int prev_wp;
//   prev_wp = next_wp-1;
//   if (next_wp == 0) {
//     prev_wp  = maps_x.size()-1;
//   }

//   double n_x = maps_x[next_wp]-maps_x[prev_wp];
//   double n_y = maps_y[next_wp]-maps_y[prev_wp];
//   double x_x = x - maps_x[prev_wp];
//   double x_y = y - maps_y[prev_wp];

//   // find the projection of x onto n
//   double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//   double proj_x = proj_norm*n_x;
//   double proj_y = proj_norm*n_y;

//   double frenet_d = distance(x_x,x_y,proj_x,proj_y);

//   //see if d value is positive or negative by comparing it to a center point
//   double center_x = 1000-maps_x[prev_wp];
//   double center_y = 2000-maps_y[prev_wp];
//   double centerToPos = distance(center_x,center_y,x_x,x_y);
//   double centerToRef = distance(center_x,center_y,proj_x,proj_y);

//   if (centerToPos <= centerToRef) {
//     frenet_d *= -1;
//   }

//   // calculate s value
//   double frenet_s = 0;
//   for (int i = 0; i < prev_wp; ++i) {
//     frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
//   }

//   frenet_s += distance(0,0,proj_x,proj_y);

//   return {frenet_s,frenet_d};
// }

// // Transform from Frenet s,d coordinates to Cartesian x,y
// vector<double> getXY(double s, double d, const vector<double> &maps_s, 
//                      const vector<double> &maps_x, 
//                      const vector<double> &maps_y) {
//   int prev_wp = -1;

//   while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
//     ++prev_wp;
//   }

//   int wp2 = (prev_wp+1)%maps_x.size();

//   double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
//                          (maps_x[wp2]-maps_x[prev_wp]));
//   // the x,y,s along the segment
//   double seg_s = (s-maps_s[prev_wp]);

//   double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
//   double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

//   double perp_heading = heading-pi()/2;

//   double x = seg_x + d*cos(perp_heading);
//   double y = seg_y + d*sin(perp_heading);

//   return {x,y};
// }

#endif  // HELPERS_H