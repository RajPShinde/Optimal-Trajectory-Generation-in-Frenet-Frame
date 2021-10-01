#include <optimalTrajectoryPlanner.hpp>

OptimalTrajectoryPlanner::OptimalTrajectoryPlanner() {
}

OptimalTrajectoryPlanner::~OptimalTrajectoryPlanner() {
}

bool OptimalTrajectoryPlanner::isColliding(){
	return false;
}



std::vector<std::vector<double>> OptimalTrajectoryPlanner::optimalTrajectory(double d0, double dv0, double da0,
												 double s0, double sv0, double sa0,
												 std::vector<std::vector<double>> &obstacles){
	// Calculate all possible paths in TNB/Frenet Frame
	for(double T = minPredictionStep; T<maxPredictionStep; T=T+0.1){
		FrenetPath path;
		path.T = T;

		// Latitudional
		for(double dT = -((noOfLanes-1)*laneWidth)/2; dT<= ((noOfLanes-1)*laneWidth)/2; dT = dT + laneWidth){
			double dvT = 0 , daT = 0;
			std::vector<std::vector<double>> trajectory;
			Polynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
			for(double t=0; t<=T; t=t+0.1){
				std::vector<double> data= {t, quintic.position(t), quintic.velocity(t), quintic.acceleration(t), quintic.jerk()};
				trajectory.push_back(data);
			}
			path.d.push_back(trajectory);
		}

		// Longitudional (Velocity Keeping)
		for(){
			std::vector<std::vector<double>> trajectory;
			Polynomial quartic(s0, sv0, sa0, svT, 0, T);
			for(double t=0; t<=T; t=t+0.1){
				std::vector<double> data= {t, quartic.position(t), quartic.velocity(t), quartic.acceleration(t), quartic.jerk()};
				trajectory.push_back(data);
			}
			path.s.push_back(trajectory);
		}
	}

	return optimalTrajectory;
}

void OptimalTrajectoryPlanner::run(){

	// Center Lane
	std::vector<std::vector<double>> centerLane;
	for(int x{0}, x<100; x = x + 0.05){
		// Arbitary function describing a lane
		y = 4*(sin(std::pow((x*0.05-3.5),2) + (x*0.05-3.5) + 2) + 10);
		centerLane.push_back({x,y});
	}

	// Obstacles along the lane
	std::std::vector<std::vector<double>> obstacles = {{10.58,43}, {10.58,45}, {20,38}, {60,43.93}, {40,42.5}};

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

