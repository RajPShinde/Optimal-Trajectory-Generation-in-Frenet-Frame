#include <optimalTrajectoryPlanner.hpp>

OptimalTrajectoryPlanner::OptimalTrajectoryPlanner() {
	run();
}

OptimalTrajectoryPlanner::~OptimalTrajectoryPlanner() {
}

bool OptimalTrajectoryPlanner::isColliding(){
	return false;
}

void OptimalTrajectoryPlanner::optimalTrajectory(double latitudionalPosition, double latitudionalVelocity, double latitudionalAcceleration,
												 double longitudionalPosition, double longitudionalVelocity, double longitudionalAcceleration,
												 std::vector<std::vector<double>> &obstacles){
}

void OptimalTrajectoryPlanner::run(){

	// Center Lane
	std::std::vector<std::vector<double>> centerLane;

	// Obstacles
	std::std::vector<std::vector<double>> obstacles;

	// Run Trajectory Planner till the end of perceived/available Lane data
	while(std::sqrt(std::pow(centerLane.back()[0]-x,2)+std::pow(centerLane.back()[1]-y,2))<1){
		
		// Get optimal Trajectory
		optimalTrajectory(d, dv, da, s, sv)

		// visualize
	}


}

