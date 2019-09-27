#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "../include/helpers.h"
#include "../include/vehicle.h"
#include "../include/behavior_planner.h"
#include "../include/trajectory_planner.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void print_lane(LaneType lane){
  if (lane == LaneType::LEFT) {
    cout << "LEFT       " << endl;
  } else if (lane == LaneType::MID) {
    cout << "MID        " << endl;
  } else if (lane == LaneType::RIGHT) {
    cout << "RIGHT      " << endl;
  } else if (lane == LaneType::NONE) {
    cout << "NONE       " << endl;
  } else if (lane == LaneType::INVALID) {
    cout << "INVALID    " << endl;
  }
}

void print_behavior(BehaviorType behavior){
  if (behavior == BehaviorType::PREP_CHANGE_LANE) {
    cout << "PREP_CHANGE_LANE  " << endl;
  } else if (behavior == BehaviorType::LANECLEAR) {
    cout << "LANECLEAR         " << endl;
  } else if (behavior == BehaviorType::FOLLOW) {
    cout << "FOLLOW            " << endl;
  } else if (behavior == BehaviorType::TURNLEFT) {
    cout << "TURNLEFT          " << endl;
  } else if (behavior == BehaviorType::TURNRIGHT) {
    cout << "TURNRIGHT         " << endl;
  }
}


int main() {
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
  while (getline(in_map_, line)) {
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

  Behavior_planner bp;
  double reference_vel = 0;
  BehaviorType state = BehaviorType::LANECLEAR;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &bp, &reference_vel, &state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>(); 
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          cout << "-----------------------" << endl;
          // Main car's localization Data
          double car_x = j[1]["x"]; //The car's x position in map coordinates
          double car_y = j[1]["y"]; //The car's y position in map coordinates
          double car_s = j[1]["s"]; //The car's s position in frenet coordinates
          double car_d = j[1]["d"]; //The car's d position in frenet coordinates
          double car_yaw = j[1]["yaw"]; //The car's yaw angle in the map, in degree
          double car_speed = j[1]["speed"]; //The car's speed in MPH

          // Previous path data given to the Planner, vector type, with processed points removed
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // cout << "****" << car_yaw << endl;
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // cout << "****" << end_path_s << endl;

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road. 0,1,2,3,4,5,6->id,x,y,vx,vy,s,d
          auto sensor_fusion = j[1]["sensor_fusion"];

          // int prev_size = previous_path_x.size(); 
          // if(prev_size > 0){
          //   car_s = end_path_s;
         
          //   // shorten the previous path to make the trajectory more responsive
          //   if((prev_size > 20)) { 
          //     prev_size = 20;
          //   }
          // }

          Vehicle mycar(222);
          mycar.get_position(car_x, car_y, car_s, car_d, car_yaw);
          mycar.get_velocity(car_speed);
          mycar.assign_adjacent_lane();
          cout << "car num: " << mycar.id << " s: " << car_s << ", d: " << car_d << endl;
          cout << "car num: " << mycar.id << " x: " << car_x << ", y: " << car_y << endl;
          std::cout << "current lane: ";
          print_lane(mycar.current_lane);
          cout << "" << endl;

          vector<Vehicle> otherCars;
          for (int i=0; i<sensor_fusion.size(); i++) {
            int id = sensor_fusion[i][0];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            Vehicle car(id);
            car.get_position(s, d);
            car.get_velocity(sqrt(vx*vx+vy*vy));
            otherCars.push_back(car);
            // cout << "car num: " << i << " s: " << s << ", d: " << d << ", lane: ";
            // print_lane(car.current_lane); 
            // cout << endl;
          }
          // // Print for debugging
          // cout << "---------------------------------" << endl;
          // cout << "STATE: s, d --- x, y ---yaw, v:" << endl;
          // cout << car_s << " , "
          //      << car_d << " --- "
          //      << car_x << " , "
          //      << car_y  << " --- "
          //      << car_yaw << " , "
          //      << car_speed << ":" << endl;

          // cout << "---------------------------------" << endl;
          // cout << "our left:  our lane:   our right:" << endl;
          // print_lane(mycar.lane_at_left);
          // print_lane(mycar.current_lane);
          // print_lane(mycar.lane_at_right);
          // cout << endl;

          // cout << "---------------------------------" << endl;

          bp.get_gap(mycar, otherCars);
          bp.get_cost(mycar);
          LaneType target_lane = bp.update_state(mycar, otherCars, reference_vel, state);
          std::cout << "target lane: ";
          print_lane(target_lane);
          print_behavior(state);

          // cout << "front left gap: " << bp.gap[0][0] << " front current gap: " << bp.gap[0][1] << " front right gap: " << bp.gap[0][2] << endl;
          // cout << "back left gap: " << bp.gap[1][0] << " back current gap: " << bp.gap[1][1] << " back right gap: " << bp.gap[1][2] << endl;

          // cout << "front left vel: " << bp.vel[0][0] << " front current vel: " << bp.vel[0][1] << " front right vel: " << bp.vel[0][2] << endl;
          // cout << "back left vel: " << bp.vel[1][0] << " back current vel: " << bp.vel[1][1] << " back right vel: " << bp.vel[1][2] << endl;
          
          std::cout << "vel: " << reference_vel << std::endl;
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // std::cout << "pre size: " << previous_path_x.size() << std::endl;
          Trajectory_planner planner(mycar.v, target_lane, previous_path_x, previous_path_y);
          planner.add_smooth_pts(mycar, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          planner.getSpline();
          planner.generate_next_Pts(next_x_vals, next_y_vals, reference_vel);

          std::ofstream x_path;
          std::ofstream y_path;
          x_path.open("/home/han/Autonomous_driving_ws/src/CarND-Path-Planning-Project/xpath.txt",std::ios::out | std::ios::app);
          y_path.open("/home/han/Autonomous_driving_ws/src/CarND-Path-Planning-Project/ypath.txt",std::ios::out | std::ios::app);

          for (int i=0; i<next_x_vals.size(); i++) {
            // cout << "next" << i << ": " << next_x_vals[i] << endl;
            // cout << "next" << i << ": " << next_y_vals[i] << endl;
            if (i != next_x_vals.size()-1) {
              x_path << next_x_vals[i] << " "; 
              y_path << next_y_vals[i] << " "; 
            }
            else {
              x_path << next_x_vals[i]; 
              y_path << next_y_vals[i];
            }

          }
          x_path << "\n";
          y_path << "\n";
          x_path.close();
          y_path.close();

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}