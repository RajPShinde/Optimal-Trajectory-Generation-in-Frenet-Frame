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
	// To store all possible tranectories in TNB/Frenet Frame
	std::vector<FrenetPath> paths;

	// Calculate all possible paths in TNB/Frenet Frame
	// Iterate over different prediction times
	for(double T = minPredictionStep; T<maxPredictionStep; T=T+0.1){

		// Iterate over different lanes
		for(double dT = -((noOfLanes-1)*laneWidth)/2; dT<= ((noOfLanes-1)*laneWidth)/2; dT = dT + laneWidth){
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

	return optimalTrajectory;
}

void OptimalTrajectoryPlanner::trajectoryCost(FrenetPath &path){
	cd = path.jd*kj + path.T*kt + std::pow(path.d.back()[0], 2)*ks;
	cv = path.js*kj + path.T*kt + std::pow(path.s.front()[0]-path.s.back()[0],2)*ks;
	path.cf = kLat*cd + kLon*cv;
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

