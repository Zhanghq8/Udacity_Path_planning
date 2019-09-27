#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

#include "helpers.h"
#include "vehicle.h"

class Behavior_planner
{
public:
	// gap, size = 2*3
	// ||front_left || front_cur || front_right
	// ||back_left || back_cur || back_right
	std::vector<std::vector<double>> gap;
	LaneType next_lane;

	bool is_safe_left;
	bool is_safe_right;
	// BehaviorType state;

	//size = 1*3
	// left || cur || right||
	std::vector<double> cost;

	// velocity, size = 2*3
	// ||front_left || front_cur || front_right
	// ||back_left || back_cur || back_right
	std::vector<std::vector<double>> vel;

	// velocity difference, size = 2*3
	// ||front_left || front_cur || front_right
	// ||back_left || back_cur || back_right
	std::vector<std::vector<double>> diff_vel;


	Behavior_planner();
	void init_gap();

	// update the gap and vel vector
	void get_gap(const Vehicle &myCar, const std::vector<Vehicle>& otherCars);
	// update the cost vec;
	void get_cost(const Vehicle &myCar);

	double cal_cost(const Vehicle &myCar, const double diff_velocity, const double velocity, const double gap_value);

	LaneType update_state(const Vehicle &myCar, const std::vector<Vehicle>& otherCars, double& reference_vel, 
						BehaviorType &state);

	double follow_speed(); 
	double d_center(const LaneType &nextlane);

	~Behavior_planner();

// private:
};

#endif