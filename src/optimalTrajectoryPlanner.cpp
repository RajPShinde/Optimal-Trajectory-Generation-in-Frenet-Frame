#include <optimalTrajectoryPlanner.hpp>

OptimalTrajectoryPlanner::OptimalTrajectoryPlanner() {
	minimumTurningRadius_ = wheelBase_/tan(maxSteeringAngle_)
	maxCurvature_ = 1/minimumTurningRadius_;
}

OptimalTrajectoryPlanner::~OptimalTrajectoryPlanner() {
}

FrenetPath OptimalTrajectoryPlanner::optimalTrajectory(double d0, double dv0, double da0,
												 double s0, double sv0, double sa0,
												 std::vector<std::vector<double>> &obstacles){
	// To store all possible tranectories in TNB/Frenet Frame
	std::vector<FrenetPath> paths;

	// Calculate all possible paths in TNB/Frenet Frame
	// Iterate over different prediction times
	for(double T = minPredictionStep_; T<maxPredictionStep_; T=T+0.1){

		// Iterate over different lanes
		for(double dT = -((noOfLanes_-1)*laneWidth_)/2; dT<= ((noOfLanes_-1)*laneWidth_)/2; dT = dT + laneWidth_){
			double dvT = 0 , daT = 0;
			std::vector<std::vector<double>> latitudionalTrajectory;
			Polynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
			double jd = 0;
			// Generate Latitudional Trajectory for a given T and lane
			for(double t=0; t<=T; t=t+0.1){
				std::vector<double> data= {t, quintic.position(t), quintic.velocity(t), quintic.acceleration(t), quintic.jerk()};
				jd += std::pow(data[4]);
				latitudionalTrajectory.push_back(data);
			}

			// Iterate over different end velocities times
			for(){
				FrenetPath path;
				path.T = T;
				path.d.push_back(latitudionalTrajectory);
				path.jd = jd;
				std::vector<std::vector<double>> longitudionalTrajectory;
				Polynomial quartic(s0, sv0, sa0, svT, 0, T);
				double js = 0;
				// Generate Longitudional Trajectory for a given v and Latitudional Trajectory
				for(double t=0; t<=T; t=t+0.1){
					std::vector<double> data= {t, quartic.position(t), quartic.velocity(t), quartic.acceleration(t), quartic.jerk()};
					js += std::pow(data[4]);
					if(data[2]>path.maxVelocity)
						path.maxVelocity = data[2];
					if(data[3]>path.maxAcceleration)
						path.maxAcceleration = data[3];
					longitudionalTrajectory.push_back(data);
				}
				path.s.push_back(longitudionalTrajectory);
				path.js = js;

				trajectoryCost(path);

				// store Frenet path
				paths.push_back(path);
			}
		}
	}

	// Convert Trajectories from Frenet Frame to Global/World Frame
	convertToWorldFrame(paths);

	// check if trajectories are valid based on kinodynamic constraints and collisions
	std::vector<FrenetPath> validPaths;
	validPaths = isValid(paths);

	// Find the optimal trajectory out of all valid paths based on cost
	FrenetPath optimalTrajectory;
	double cost = INT_MAX;
	for(FrenetPath path:paths){
		if(cost >= path.cf){
			cost = path.cf
			optimalTrajectory = path;
		}
	}

	return optimalTrajectory;
}

void OptimalTrajectoryPlanner::trajectoryCost(FrenetPath &path){
	cd = path.jd*kj_ + path.T*kt_ + std::pow(path.d.back()[0], 2)*ks_;
	cv = path.js*kj_ + path.T*kt_ + std::pow(path.s.front()[0]-path.s.back()[0],2)*ks_;
	path.cf = klat_*cd + klon_*cv;
}

bool OptimalTrajectoryPlanner::isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles){
	for(int i = 0; i<path.world.size(); i++){
		for(int j = 0; j<obstacles.size(); j++){
			if(std::sqrt(std::pow(path.world[i][0]-obstacles[j][0],2)+std::pow(path.world[i][1]-obstacles[j][1],2)) > robotFootprint_ + 1)
				return true;
		}
	}
	return false;
}

void OptimalTrajectoryPlanner::isWithinKinematicConstraints(FrenetPath &path){
	if(path.maxVelocity>maxVelocity_ || path.maxAcceleration>maxAcceleration_ || path.maxCurvature>maxCurvature_){
		return true;
	}
	return false;
}

std::vector<FrenetPath> OptimalTrajectoryPlanner::isValid(std::vector<FrenetPath> paths){
	std::vector<FrenetPath> validPaths;
	for(FrenetPath path:paths){
		if (!isColliding(path, obstacles) && !isWithinKinematicConstraints(path)){
			validPaths.push_back(path)
		}
	}
	return validPaths;
}

void OptimalTrajectoryPlanner::run(){

	// Center Lane
	std::vector<std::vector<double>> centerLane;
	for(int x{0}, x<100; x = x + 0.05){
		// Arbitary function describing a lane
		double a = 0.05;
		double b = 3.5;
		double c = 2;
		double y = 4*(sin(std::pow((x*a-b),2) + (x*a-b) + c) + 10);
		double dy = 4*cos(std::pow((x*a-b),2) + (x*a-b) + c)*(((2*a)*(x*a-b))+a);
		double ddy = 4((-1*sin(std::pow((x*a-b),2) + (x*a-b) + c)*(std::pow((((2*a)*(x*a-b))+a),2))) +
				(cos(std::pow((x*a-b),2) + (x*a-b) + c)*(2*a*a)))
		
		double curvature = std::abs(ddy)/std::sqrt(std::pow((1+(dy*dy)),3));
		double yaw = dy;
		centerLane.push_back({x, y, yaw, curvature});
	}

	// Obstacles along the lane
	std::vector<std::vector<double>> obstacles = {{10.58,43}, {10.58,45}, {20,38}, {60,43.93}, {40,42.5}};

	// Run Trajectory Planner till the end of perceived/available Lane data
	while(std::sqrt(std::pow(centerLane.back()[0]-x,2)+std::pow(centerLane.back()[1]-y,2))<1){
		
		// Get optimal Trajectory
		optimalTrajectory(d0, dv0, da0, s0, sv0)

		// visualize
		// Lane
		for(int i{0}; i<centerLane.size(); i++){
		}
		// Obstacles
		for(int i{0}; i<obstacles.size(); i++){
		}
		// Robot
	}


}

