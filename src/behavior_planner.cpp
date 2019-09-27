#include "../include/behavior_planner.h"

Behavior_planner::Behavior_planner() {
	gap = std::vector<std::vector<double>> (2, std::vector<double> (3, 0));
	init_gap();
	vel = std::vector<std::vector<double>> (2, std::vector<double> (3, 0));
	diff_vel = std::vector<std::vector<double>> (2, std::vector<double> (3, 0));
	cost = std::vector<double> (3, 0);
	is_safe_left = true;
	is_safe_right = true;
	next_lane = LaneType::NONE;
	// state = BehaviorType::LANECLEAR;

}
void Behavior_planner::init_gap() {
	for (int i=0; i<gap[0].size(); i++) {
		gap[FRONT][i] = MAXGAP;
		gap[BACK][i] = -MAXGAP;
	}
}

LaneType Behavior_planner::update_state(const Vehicle &myCar, const std::vector<Vehicle>& otherCars, 
	double& reference_vel, BehaviorType &state) {

	get_gap(myCar, otherCars);
	get_cost(myCar);
	// update safe gap flag
	// std::cout << "*********************" << std::endl;
	// std::cout << "gap back left " << gap[BACK][LEFTLANE] << std::endl;
	// std::cout << "gap front left " << gap[FRONT][LEFTLANE] << std::endl;
	// std::cout << "gap back right " << gap[BACK][RIGHTLANE] << std::endl;
	// std::cout << "gap front right " << gap[FRONT][RIGHTLANE] << std::endl;
	// std::cout << "*********************" << std::endl;
	if (gap[BACK][LEFTLANE] > -SAFE_BACK_GAP || gap[FRONT][LEFTLANE] < SAFE_FRONT_GAP) {// ||
		//fabs(gap[BACK][LEFTLANE]) <= fabs(diff_vel[BACK][LEFTLANE])) {
		is_safe_left = false;
	}
	else {
		is_safe_left = true;
	}
	if (gap[BACK][RIGHTLANE] > -SAFE_BACK_GAP || gap[FRONT][RIGHTLANE] < SAFE_FRONT_GAP) {// || 
		// fabs(gap[BACK][RIGHTLANE]) <= fabs(diff_vel[BACK][RIGHTLANE])) {
		is_safe_right = false;
	}
	else {
		is_safe_right = true;
	}
	// std::cout << "*********************" << std::endl;
	// std::cout << "left safe? " << is_safe_left << std::endl;
	// std::cout << "right safe? " << is_safe_right << std::endl;
	// std::cout << "*********************" << std::endl;

	switch(state) {
		case(BehaviorType::LANECLEAR):
			//stay at mid lane:
			// std::cout << "state: LANECLEAR" << std::endl;
			reference_vel = MAX_VEL; 
			if (gap[FRONT][CURRENTLANE] < ACTION_FRONT_DISTANCE) {
				state = BehaviorType::PREP_CHANGE_LANE;
				// std::cout << " after: PREP_CHANGE_LANE" << std::endl;
			}
			// else if (myCar.current_lane != LaneType::MID) {
			// 	state = BehaviorType::PREP_CHANGE_LANE;
			// }
			next_lane = myCar.current_lane;
			break;

		case(BehaviorType::PREP_CHANGE_LANE):
			// std::cout << "state: PREP_CHANGE_LANE" << std::endl;
			if (gap[FRONT][CURRENTLANE] < FOLLOW_DISTANCE) {
				state = BehaviorType::FOLLOW;
				// std::cout << "state: FOLLOW" << std::endl;
				next_lane = myCar.current_lane;
				reference_vel = follow_speed();
			}
			else if (gap[FRONT][CURRENTLANE] < SAFE_FRONT_GAP) {
				// state = BehaviorType::LANECLEAR;
				// std::cout << "state: LANECLEAR" << std::endl;
				reference_vel = std::min<double>(MAX_VEL, MAX_VEL * gap[FRONT][CURRENTLANE] / SAFE_FRONT_GAP) ;
				next_lane = myCar.current_lane;
				// if (gap[LEFTLANE][FRONT] >= ACTION_FRONT_DISTANCE && myCar.current_lane == LaneType::RIGHT) {
				// 	next_lane = myCar.lane_at_left;
				// }
				// else if (gap[RIGHTLANE][FRONT] >= ACTION_FRONT_DISTANCE && myCar.current_lane == LaneType::LEFT) {
				// 	next_lane = myCar.lane_at_right;
				// }
			} 
			else if (gap[FRONT][CURRENTLANE] > ACTION_FRONT_DISTANCE) {
				reference_vel = MAX_VEL;
				state = BehaviorType::LANECLEAR;
				next_lane = myCar.current_lane;
			}

			if (is_safe_left == true && cost[LEFTLANE] <= cost[RIGHTLANE] && cost[LEFTLANE] < cost[CURRENTLANE]) {
				state = BehaviorType::TURNLEFT;
				// std::cout << "state: TURNLEFT" << std::endl;
				next_lane = myCar.lane_at_left;
				reference_vel = MAX_VEL;
				// LANE?
			}
			else if (is_safe_right == true && cost[RIGHTLANE] <= cost[LEFTLANE] && cost[RIGHTLANE] < cost[CURRENTLANE]) {
				state = BehaviorType::TURNRIGHT;
				// std::cout << "state: TURNRIGHT" << std::endl;
				next_lane = myCar.lane_at_right;
				reference_vel = MAX_VEL;
				// LANE?
			}
			break;

		case(BehaviorType::FOLLOW):
			// std::cout << "state: FOLLOW" << std::endl;
			if (gap[FRONT][CURRENTLANE] < FOLLOW_DISTANCE) {
				reference_vel = follow_speed();
				next_lane = myCar.current_lane;
			}
			else {
				state = BehaviorType::LANECLEAR;
				next_lane = myCar.current_lane;
				reference_vel = MAX_VEL;
			}
			break;

		case(BehaviorType::TURNLEFT):
			// std::cout << "state: TURNLEFT" << std::endl;
			std::cout << " next_d: " << d_center(next_lane) << std::endl;
			
			if (myCar.d > d_center(next_lane) - 0.5 && myCar.d < d_center(next_lane) + 0.5) {
				state = BehaviorType::LANECLEAR;
				next_lane = myCar.current_lane;
				reference_vel = follow_speed();
				std::cout << " left turn finished" << std::endl;
			}
			else if (gap[FRONT][CURRENTLANE] < FOLLOW_DISTANCE || gap[FRONT][LEFTLANE] < FOLLOW_DISTANCE) {
				next_lane = myCar.current_lane;
				reference_vel = follow_speed();
			}
			else {
				reference_vel = MAX_VEL;
				next_lane = myCar.lane_at_left;
			}
			break;

		case(BehaviorType::TURNRIGHT):
			// std::cout << "state: TURNRIGHT" << std::endl;
			std::cout << " next_d: " << d_center(next_lane) << std::endl;
			if (myCar.d > d_center(next_lane) - 0.5 && myCar.d < d_center(next_lane) + 0.5) {
				state = BehaviorType::LANECLEAR;
				std::cout << " right turn finished" << std::endl;
				next_lane = myCar.current_lane;
				reference_vel = follow_speed();
			}


			else if (gap[FRONT][CURRENTLANE] < FOLLOW_DISTANCE || gap[FRONT][RIGHTLANE] < FOLLOW_DISTANCE) {
				next_lane = myCar.current_lane;
				reference_vel = follow_speed();
			}
			
			else {
				reference_vel = MAX_VEL;
				next_lane = myCar.lane_at_right;
			}
			break;

		default:
			state = BehaviorType::LANECLEAR;
	}
	std::cout << "is left lane safe?" << is_safe_left << std::endl;
	std::cout << "is right lane safe?" << is_safe_right << std::endl;

	return next_lane;

}

// update gap and vel vector
// store the cloest gap(front and back) in each lane
void Behavior_planner::get_gap(const Vehicle &myCar, const std::vector<Vehicle>& otherCars) {
	init_gap();
	double min_front_gap_cur = MAXGAP;
	double min_back_gap_cur = MAXGAP;
	double min_front_gap_left = MAXGAP;
	double min_back_gap_left = MAXGAP;
	double min_front_gap_right = MAXGAP;
	double min_back_gap_right = MAXGAP;
	// int mid = 0;
	// int left = 0;
	// int right = 0;
	for (auto othercar : otherCars) {
		// mycar is at left lane 
		if (myCar.current_lane == LaneType::LEFT) {
			gap[FRONT][LEFTLANE] = DANGERGAP;
			gap[BACK][LEFTLANE] = -DANGERGAP;
			vel[FRONT][LEFTLANE] = DANGERVELOCITY;
			vel[BACK][LEFTLANE] = DANGERVELOCITY;
			diff_vel[FRONT][LEFTLANE] = DANGERVELOCITY - myCar.v;
			diff_vel[BACK][LEFTLANE] = DANGERVELOCITY - myCar.v;
			// the other car is at the same left lane
			if (othercar.current_lane == LaneType::LEFT) {
				double diffs = othercar.s - myCar.s;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_cur) {
						min_front_gap_cur = fabs(diffs);
						gap[FRONT][CURRENTLANE] = diffs;
						vel[FRONT][CURRENTLANE] = othercar.v;
						diff_vel[FRONT][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_cur) {
						min_back_gap_cur = fabs(diffs);
						gap[BACK][CURRENTLANE] = diffs;
						vel[BACK][CURRENTLANE] = othercar.v;
						diff_vel[BACK][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
			}
			// the other car is at the mid lane on the right of my car
			else if (othercar.current_lane == LaneType::MID) {
				double diffs = othercar.s - myCar.s;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_right) {
						min_front_gap_right = fabs(diffs);
						gap[FRONT][RIGHTLANE] = diffs;
						vel[FRONT][RIGHTLANE] = othercar.v;
						diff_vel[FRONT][RIGHTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_right) {
						min_back_gap_right = fabs(diffs);
						gap[BACK][RIGHTLANE] = diffs;
						vel[BACK][RIGHTLANE] = othercar.v;
						diff_vel[BACK][RIGHTLANE] = othercar.v - myCar.v;
					}
				}
			}
		}
		// mycar is at right lane
		else if (myCar.current_lane == LaneType::RIGHT) {
			gap[FRONT][RIGHTLANE] = DANGERGAP;
			gap[BACK][RIGHTLANE] = -DANGERGAP;
			vel[FRONT][RIGHTLANE] = DANGERVELOCITY;
			vel[BACK][RIGHTLANE] = DANGERVELOCITY;
			diff_vel[FRONT][RIGHTLANE] = DANGERVELOCITY - myCar.v;
			diff_vel[BACK][RIGHTLANE] = DANGERVELOCITY - myCar.v;
			// the other car is at the same right lane
			if (othercar.current_lane == LaneType::RIGHT) {
				double diffs = othercar.s - myCar.s;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_cur) {
						min_front_gap_cur = fabs(diffs);
						gap[FRONT][CURRENTLANE] = diffs;
						vel[FRONT][CURRENTLANE] = othercar.v;
						diff_vel[FRONT][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_cur) {
						min_back_gap_cur = fabs(diffs);
						gap[BACK][CURRENTLANE] = diffs;
						vel[BACK][CURRENTLANE] = othercar.v;
						diff_vel[BACK][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
			}
			// the other car is at the mid lane on the left of my car
			else if (othercar.current_lane == LaneType::MID) {
				double diffs = othercar.s - myCar.s;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_left) {
						min_front_gap_left = fabs(diffs);
						gap[FRONT][LEFTLANE] = diffs;
						vel[FRONT][LEFTLANE] = othercar.v;
						diff_vel[FRONT][LEFTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_left) {
						min_back_gap_left = fabs(diffs);
						gap[BACK][LEFTLANE] = diffs;
						vel[BACK][LEFTLANE] = othercar.v;
						diff_vel[BACK][LEFTLANE] = othercar.v - myCar.v;
					}
				}
			}
		}
		// mycar is at mid lane
		else {
			if (othercar.current_lane == LaneType::LEFT) {
				// left++;
				double diffs = othercar.s - myCar.s;
				// std::cout << "othercar s: " << othercar.s << std::endl;
				// std::cout << "mycar s: " << myCar.s << std::endl;
				// std::cout << "othercar v: " << othercar.v << std::endl;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_left) {
						min_front_gap_left = fabs(diffs);
						gap[FRONT][LEFTLANE] = diffs;
						vel[FRONT][LEFTLANE] = othercar.v;
						diff_vel[FRONT][LEFTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_left) {
						min_back_gap_left = fabs(diffs);
						gap[BACK][LEFTLANE] = diffs;
						vel[BACK][LEFTLANE] = othercar.v;
						diff_vel[BACK][LEFTLANE] = othercar.v - myCar.v;
					}
				}
			}
			else if (othercar.current_lane == LaneType::MID) {
				// mid++;
				double diffs = othercar.s - myCar.s;
				// std::cout << "othercar s: " << othercar.s << std::endl;
				// std::cout << "mycar s: " << myCar.s << std::endl;
				// std::cout << "othercar v: " << othercar.v << std::endl;
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_cur) {
						min_front_gap_cur = fabs(diffs);
						gap[FRONT][CURRENTLANE] = diffs;
						vel[FRONT][CURRENTLANE] = othercar.v;
						diff_vel[FRONT][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_cur) {
						min_back_gap_cur = fabs(diffs);
						gap[BACK][CURRENTLANE] = diffs;
						vel[BACK][CURRENTLANE] = othercar.v;
						diff_vel[BACK][CURRENTLANE] = othercar.v - myCar.v;
					}
				}
			}
			else if (othercar.current_lane == LaneType::RIGHT) {
				// right++;
				double diffs = othercar.s - myCar.s;
				// if (diffs == 0) {
				// 	std::cout << "othercar s: " << othercar.s << std::endl;
				// 	std::cout << "mycar s: " << myCar.s << std::endl;
				// 	std::cout << "othercar v: " << othercar.v << std::endl;
				// }
				if (diffs >= 0) {
					if (fabs(diffs) < min_front_gap_right) {
						min_front_gap_right = fabs(diffs);
						gap[FRONT][RIGHTLANE] = diffs;
						vel[FRONT][RIGHTLANE] = othercar.v;
						diff_vel[FRONT][RIGHTLANE] = othercar.v - myCar.v;
					}
				}
				else {
					if (fabs(diffs) < min_back_gap_right) {
						min_back_gap_right = fabs(diffs);
						gap[BACK][RIGHTLANE] = diffs;
						vel[BACK][RIGHTLANE] = othercar.v;
						diff_vel[BACK][RIGHTLANE] = othercar.v - myCar.v;
					}
				}
			}
		}
	}

	// std::cout << "car in left: " << left << ". " << std::endl;
	// std::cout << "car in mid: " << mid << ". " << std::endl;
	// std::cout << "car in right: " << left << ". " << std::endl;

	std::cout << "*********************" << std::endl;
	std::cout << "gap fLEFT: " << gap[FRONT][LEFTLANE] << " gap fCUR: " << gap[FRONT][CURRENTLANE] 
			<< " gap fRIGHT: " << gap[FRONT][RIGHTLANE]<< std::endl;
	std::cout << "gap bLEFT: " << gap[BACK][LEFTLANE] << " gap bCUR: " << gap[BACK][CURRENTLANE] 
			<< " gap bRIGHT: " << gap[BACK][RIGHTLANE]<< std::endl;
	std::cout << "*********************" << std::endl;
}
double Behavior_planner::d_center(const LaneType &nextlane) {
	if (nextlane == LaneType::RIGHT) {
		return 10.0;
	}
	else if (nextlane == LaneType::LEFT) {
		return 2.0;
	}
	else if (nextlane == LaneType::MID) {
		return 6.0;
	}
	else {
		return MAXGAP;
	}
}

void Behavior_planner::get_cost(const Vehicle &myCar) {
	for (int i=0; i<cost.size(); i++) {
		cost[LEFTLANE] = cal_cost(myCar, diff_vel[FRONT][LEFTLANE], vel[FRONT][LEFTLANE], gap[FRONT][LEFTLANE]) + 
			cal_cost(myCar, diff_vel[BACK][LEFTLANE], vel[BACK][LEFTLANE], gap[BACK][LEFTLANE]);
		cost[CURRENTLANE] = cal_cost(myCar, diff_vel[FRONT][CURRENTLANE], vel[FRONT][CURRENTLANE], gap[FRONT][CURRENTLANE]) + 
			cal_cost(myCar, diff_vel[BACK][CURRENTLANE], vel[BACK][CURRENTLANE], gap[BACK][CURRENTLANE]);
		cost[RIGHTLANE] = cal_cost(myCar, diff_vel[FRONT][RIGHTLANE], vel[FRONT][RIGHTLANE], gap[FRONT][RIGHTLANE]) + 
			cal_cost(myCar, diff_vel[BACK][RIGHTLANE], vel[BACK][RIGHTLANE], gap[BACK][RIGHTLANE]);
	}
	std::cout << "*********************" << std::endl;
	std::cout << "left cost: " << cost[LEFTLANE] << std::endl;
	std::cout << "current cost: " << cost[CURRENTLANE] << std::endl;
	std::cout << "right cost: " << cost[RIGHTLANE] << std::endl;
	std::cout << "*********************" << std::endl;
}

double Behavior_planner::cal_cost(const Vehicle &myCar, const double diff_velocity, const double velocity, const double gap_value) {
	double cost = 0;
	cost += fabs(myCar.v - velocity) * RELATIVE_VEL_COST;
	cost += fabs(MAX_VEL - velocity) * MAX_VEL_COST;
	if (gap_value == DANGERGAP) {
		cost += DANGERCOST;
	}
	else {
		cost += (1 - fabs(gap_value)/LOOKAHEAD_DISTANCE) * DIS_COST;
	}
	cost += fabs(((myCar.s+gap_value+velocity) - (myCar.s+myCar.v))/LOOKAHEAD_DISTANCE) * DIS_COST;
	return cost;
}

// TODO: optimize
double Behavior_planner::follow_speed() {
	double follow_velocity = 0;
	if (vel[CURRENTLANE][FRONT] <= 0.3 && gap[CURRENTLANE][FRONT] < 5) {
		follow_velocity = 0;
	}
	if (gap[CURRENTLANE][FRONT] < 15) {
		follow_velocity = (vel[CURRENTLANE][FRONT] / 2);
	}
	else {
		follow_velocity = vel[CURRENTLANE][FRONT];
	}
	return follow_velocity;
}

Behavior_planner::~Behavior_planner() {

}