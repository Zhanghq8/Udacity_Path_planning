#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <math.h>
#include <iostream>
#include <vector>
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "vehicle.h"
#include "helpers.h"

class Trajectory_planner {

public:

	Trajectory_planner(const double speed, const LaneType lane_, const vector<double> &prev_path_x,
        			const vector<double> &prev_path_y); 
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, 
                 	const vector<double> &maps_y);
	vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                 	const vector<double> &maps_x, const vector<double> &maps_y);
	void add_smooth_pts(const Vehicle& mycar_, const vector<double> &maps_s, const vector<double> &maps_x, 
					const vector<double> &maps_y);
	void set_next_lane(const LaneType lane_);
	void getSpline();
	void generate_next_Pts(vector<double> &next_x_vals, vector<double> &next_y_vals, 
				const double reference_vel);
	double cal_Spline_y(double x);
	vector<double> global2local(double yaw, double x, double y);
	vector<double> local2global(double yaw, double x, double y);
	void updateStep(double &speed, const double reference_vel);
	double deg2rad(double x);
	double rad2deg(double x);
	double distance(double x1, double y1, double x2, double y2);
	~Trajectory_planner();
	// double velocity(double vx, double vy);

private:
	tk::spline f_spline;
	// lane number to travel in the future
	double next_lane;
	LaneType lane;
	// previous trajectory (x,y) path points
	vector<double> pre_pts_x;
	vector<double> pre_pts_y;
	// trajectory (x,y) path points
	vector<double> path_x;
	vector<double> path_y;

	// in cartesian coordinates
	double reference_x;
	double reference_y;
	double reference_yaw;

	// in local cartesian coordinates
	double local_x;
	double local_y;
	double pre_local_x;
	double pre_local_y;

	// step to take 
	double step;
	double mycar_speed;

};

#endif