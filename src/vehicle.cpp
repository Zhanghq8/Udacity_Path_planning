#include "../include/vehicle.h"

Vehicle::Vehicle(const int id_) {
	id = id_;
}

void Vehicle::get_position(const double x_, const double y_, const double s_, const double d_, const double yaw_) {
	x = x_;
	y = y_;
	s = s_;
	d = d_;
	yaw = yaw_;
	current_lane = d2lanetype(d_);
}

void Vehicle::get_position(const double s_, const double d_) {
	s = s_;
	d = d_;
	current_lane = d2lanetype(d_);
}

void Vehicle::get_velocity(const double v_) {
	v = v_;
}

LaneType Vehicle::d2lanetype(const double d_) {
	LaneType lane = LaneType::NONE;
	if (d_>0 && d_<4.0) {
		lane = LaneType::LEFT;
	}
	else if (d_>=4.0 && d_<8.0) {
		lane = LaneType::MID;
	}
	else if (d_>=8.0 && d_<12.0) {
		lane = LaneType::RIGHT;
	}
	else {
		lane = LaneType::INVALID;
	}
	return lane;
}

void Vehicle::assign_adjacent_lane() {
	if (d>0 && d<4.0) {
		lane_at_left = LaneType::INVALID;
		lane_at_right = LaneType::MID;
	}
	else if (d>=4.0 && d<8.0) {
		lane_at_left = LaneType::LEFT;
		lane_at_right = LaneType::RIGHT;
	}
	else if (d>=8.0 && d<12.0) {
		lane_at_left = LaneType::MID;
		lane_at_right = LaneType::INVALID;	
	}
}

Vehicle::~Vehicle() {

}