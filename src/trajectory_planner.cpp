#include "../include/trajectory_planner.h"

Trajectory_planner::Trajectory_planner(const double speed, const LaneType lane_, const vector<double> &prev_path_x,
        const vector<double> &prev_path_y) {
	// mycar = mycar_;
	// lane number to travel in the future
	mycar_speed = speed;
	lane = lane_;
	set_next_lane(lane);
	// previous trajectory (x,y) path points
	pre_pts_x = prev_path_x;
	pre_pts_y = prev_path_y;
	//pre_path_size;
}

void Trajectory_planner::set_next_lane(const LaneType lane_) {
	if (lane_ == LaneType::LEFT) {
		next_lane = 0;
	}
	else if (lane_ == LaneType::MID) {
		next_lane = 1;
	}
	else if (lane_ == LaneType::RIGHT) {
		next_lane = 2;
	}
	// std::cout << "next_lane: " << next_lane << std::endl;
}

// Calculate closest waypoint to current x, y position
int Trajectory_planner::ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); ++i) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int Trajectory_planner::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = std::min(2*M_PI - angle, angle);

	if (angle > M_PI/2) {
		++closestWaypoint;
		if (closestWaypoint == maps_x.size()) {
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Trajectory_planner::getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if (next_wp == 0) {
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; ++i) {
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Trajectory_planner::getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, const vector<double> &maps_y) {
	int prev_wp = -1;

	while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
		++prev_wp;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

void Trajectory_planner::add_smooth_pts(const Vehicle& mycar_, const vector<double> &maps_s, const vector<double> &maps_x, 
		const vector<double> &maps_y) {
	double previous_x;
	double previous_y;
	// add previous points
	if (pre_pts_x.size() < 2) {
		reference_x = mycar_.x;
		reference_y = mycar_.y;
		reference_yaw = deg2rad(mycar_.yaw);

		previous_x = mycar_.x - cos(reference_yaw);
		previous_y = mycar_.y - sin(reference_yaw);
		// std::cout << "reference_yaw_emp: " << reference_yaw << std::endl;

	}
	else {
		// std::cout << "pre_pts_x: " << pre_pts_x.size() << std::endl;
		reference_x = pre_pts_x[int (pre_pts_x.size()-1)];
		reference_y = pre_pts_y[int (pre_pts_y.size()-1)];

		previous_x = pre_pts_x[int (pre_pts_x.size()-2)];
		previous_y = pre_pts_y[int (pre_pts_y.size()-2)];

		reference_yaw = atan2(reference_y - previous_y, reference_x - previous_x);
		// std::cout << "reference_yaw: " << reference_yaw << std::endl;		
	}
	// std::cout << "reference_x: " << reference_x << std::endl;
	// std::cout << "reference_y: " << reference_y << std::endl;

	path_x.push_back(previous_x);
	path_y.push_back(previous_y);
	path_x.push_back(reference_x);
	path_y.push_back(reference_y);

	// add future points	
	double next_d = 4*next_lane + 2;
	// std::cout << "next d " << next_d << std::endl;
	// std::cout << "pathx" << 0 << ", " << path_x[0] << std::endl;
	// std::cout << "pathy" << 0 << ", " << path_y[0] << std::endl;
	// std::cout << "pathx" << 1 << ", " << path_x[1] << std::endl;
	// std::cout << "pathy" << 1 << ", " << path_y[1] << std::endl;
	for (int i=0; i<3; i++) {
		vector<double> next_wp = getXY(mycar_.s + (i+1)*DELTA_S, next_d, maps_s, maps_x, maps_y);
		// std::cout << "ds: " << mycar_.s + (i+1)*DELTA_S << ", dd: " << next_d << std::endl;
		// std::cout << "dx: " << next_wp[0] << ", dy: " << next_wp[1] << std::endl;
		path_x.push_back(next_wp[0]);
		path_y.push_back(next_wp[1]);
		// std::cout << "pathx" << i+2 << ", " << path_x[i+2] << std::endl;
		// std::cout << "pathy" << i+2 << ", " << path_y[i+2] << std::endl;
	}

	for (int i=0; i<path_x.size(); i++) {
		double delta_x = path_x[i] - reference_x;
		// std::cout << "refx" << i << reference_x << std::endl;
		// std::cout << "refy" << i << reference_y << std::endl;

		double delta_y = path_y[i] - reference_y;

		// convert from global cartesian coordinates to local coordinates 
		path_x[i] = global2local(reference_yaw, delta_x, delta_y)[0];
		path_y[i] = global2local(reference_yaw, delta_x, delta_y)[1];
		// std::cout << "paths" << i << ", " << path_x[i] << std::endl;
		// std::cout << "pathd" << i << ", " << path_y[i] << std::endl;
	}

}

void Trajectory_planner::getSpline() {
	f_spline.set_points(path_x, path_y);
}

double Trajectory_planner::cal_Spline_y(double x) {
	return f_spline(x);
}

void Trajectory_planner::generate_next_Pts(vector<double> &next_x_vals, vector<double> &next_y_vals, 
				const double reference_vel) {
	for(int i=0; i<pre_pts_x.size(); i++){
        next_x_vals.push_back(pre_pts_x[i]);
        next_y_vals.push_back(pre_pts_y[i]);
    }

    if (next_x_vals.size() < 2) {
    	step = 0;
    	// std::cout << "step empty: " << step << std::endl;
    }
    else {
    	double step_x = next_x_vals[int (next_x_vals.size()-1)] - next_x_vals[int (next_x_vals.size()-2)]; 
    	double step_y = next_y_vals[int (next_y_vals.size()-1)] - next_y_vals[int (next_y_vals.size()-2)];
    	// std::cout << "step x: " << step_x << std::endl;
    	// std::cout << "step y: " << step_y << std::endl;

    	step = sqrt(step_x*step_x + step_y*step_y);
    	// std::cout << "step notempty: " << step << std::endl;
    }

    double x_add_on = 0;

    for (int i = 1; i <= NUM_PREDICT_PT - pre_pts_x.size(); i++){ // 50: defined total number of points

    	updateStep(mycar_speed, reference_vel);

        local_x = x_add_on + step;
        local_y = f_spline(local_x);

        x_add_on = local_x;

        pre_local_x = local_x;
        pre_local_y = local_y;

        // rotate back to world coordinate after rotating it earlier
        double x_point = local2global(reference_yaw, pre_local_x, pre_local_y)[0];
        double y_point = local2global(reference_yaw, pre_local_x, pre_local_y)[1];

        x_point += reference_x;
        y_point += reference_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

}

void Trajectory_planner::updateStep(const double speed, const double reference_vel) {
	double acceleration = 0.0022; // max allowed acceleration per step 6m/s^2
	const double mile_h2meter_s = 1609.344 / 3600.0; // 1Mph * 1609.344meter/h / 3600 = 0.44704 m/s
	const double max_step = MAX_VEL * mile_h2meter_s * TIME_STEP;

	double ref_step = std::min<double>(reference_vel * mile_h2meter_s * TIME_STEP, max_step);
	// check if we will potential have a collision and decelerate
	if( mycar_speed > reference_vel ) {
		step = std::max(step - acceleration, ref_step); // deceleration -6m/s^2
	} 
	else if(mycar_speed < reference_vel){
    	step = std::min(step + acceleration, ref_step);
  	}
}


vector<double> Trajectory_planner::global2local(double yaw, double x, double y) {
	double local_x = x*cos(0-yaw) - y*sin(0-yaw);
	double local_y = x*sin(0-yaw) + y*cos(0-yaw); 
	return {local_x, local_y};
}

vector<double> Trajectory_planner::local2global(double yaw, double x, double y) {
	double global_x = x*cos(yaw) - y*sin(yaw);
	double global_y = x*sin(yaw) + y*cos(yaw); 
	return {global_x, global_y};
}

double Trajectory_planner::deg2rad(double x) {
	return x * M_PI / 180;
}

double Trajectory_planner::rad2deg(double x) {
	return x * 180 / M_PI;
}

double Trajectory_planner::distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

Trajectory_planner::~Trajectory_planner() {

}