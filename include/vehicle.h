#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include "helpers.h"

// store the states of the car, including cars around

class Vehicle
{
public:

	int id;
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double v;
	LaneType current_lane;
	LaneType lane_at_left;
	LaneType lane_at_right;

	Vehicle(const int id_);
	void get_position(const double x_, const double y_, const double s_, const double d_, const double yaw_);
	void get_position(const double s_, const double d_);
	void get_velocity(const double v_);
	LaneType d2lanetype(const double d_);
	void assign_adjacent_lane();

	~Vehicle();
};


#endif //_VEHICLE_H_
