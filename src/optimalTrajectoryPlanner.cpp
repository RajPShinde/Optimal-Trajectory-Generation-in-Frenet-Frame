#include <optimalTrajectoryPlanner.hpp>

OptimalTrajectoryPlanner::OptimalTrajectoryPlanner() {
	// minimumTurningRadius_ = wheelBase_/tan(maxSteeringAngle_)
	// maxCurvature_ = 1/minimumTurningRadius_;
}

OptimalTrajectoryPlanner::~OptimalTrajectoryPlanner() {
}

FrenetPath OptimalTrajectoryPlanner::optimalTrajectory(double d0, double dv0, double da0,
												 	   double s0, double sv0,
												 	   std::vector<std::vector<double>> &centerLane,
												 	   std::vector<std::vector<double>> &obstacles,
												 	   std::vector<FrenetPath> &allPaths){
	// To store all possible tranectories in TNB/Frenet Frame
	std::vector<FrenetPath> paths;

	// Calculate all possible paths in TNB/Frenet Frame
	// Iterate over different prediction times
	for(double T = minPredictionStep_; T<maxPredictionStep_; T=T+0.2){

		// Iterate over different lanes
		for(double dT = -((noOfLanes_-1)*laneWidth_)/2; dT<= ((noOfLanes_-1)*laneWidth_)/2; dT = dT + laneWidth_){
			double dvT = 0 , daT = 0;
			std::vector<std::vector<double>> latitudionalTrajectory;
			Polynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
			double jd = 0;
			// Generate Latitudional Trajectory for a given T and lane
			for(double t=0; t<=T; t=t+0.1){
				std::vector<double> data= {quintic.position(t), quintic.velocity(t), quintic.acceleration(t), quintic.jerk(t), t};
				jd += std::pow(data[3],2);
				latitudionalTrajectory.push_back(data);
			}

			// Iterate over different end velocities times
			for(double svT = targetVelocity_ - velocityStep_ ; svT<=targetVelocity_ + velocityStep_; svT = svT+velocityStep_){
				FrenetPath path;
				path.T = T;
				path.d = latitudionalTrajectory;
				path.jd = jd;
				std::vector<std::vector<double>> longitudionalTrajectory;
				Polynomial quartic(s0, sv0, 0, svT, 0, T);
				double js = 0;
				// Generate Longitudional Trajectory for a given v and Latitudional Trajectory
				for(double t=0; t<=T; t=t+0.1){
					std::vector<double> data= {quartic.position(t), quartic.velocity(t), quartic.acceleration(t), quartic.jerk(t), t};
					js += std::pow(data[3],2);
					if(data[1]>path.maxVelocity)
						path.maxVelocity = data[1];
					if(data[2]>path.maxAcceleration)
						path.maxAcceleration = data[2];
					longitudionalTrajectory.push_back(data);
				}
				path.s = longitudionalTrajectory;
				path.js = js;

				trajectoryCost(path);

				// store Frenet path
				paths.push_back(path);
			}
		}
	}

	// Convert Trajectories from Frenet Frame to Global/World Frame
	convertToWorldFrame(paths, centerLane);

	// check if trajectories are valid based on kinodynamic constraints and collisions
	std::vector<FrenetPath> validPaths;
	validPaths = isValid(paths, obstacles);
	allPaths = paths;

	// Find the optimal trajectory out of all valid paths based on cost
	FrenetPath optimalTrajectory;
	double cost = INT_MAX;
	for(FrenetPath &path:validPaths){
		if(cost >= path.cf){
			cost = path.cf;
			optimalTrajectory = path;
		}
	}

	return optimalTrajectory;
}

void OptimalTrajectoryPlanner::trajectoryCost(FrenetPath &path){
	double cd = path.jd*kjd_ + path.T*ktd_ + std::pow(path.d.back()[0], 2)*ksd_;
	double cv = path.js*kjs_ + path.T*kts_ + std::pow(path.s.front()[0]-path.s.back()[0],2)*kss_;
	path.cf = klat_*cd + klon_*cv;
}

bool OptimalTrajectoryPlanner::isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles){
	for(int i = 0; i<path.world.size(); i++){
		for(int j = 0; j<obstacles.size(); j++){
			if(std::sqrt(std::pow(path.world[i][0]-obstacles[j][0],2)+std::pow(path.world[i][1]-obstacles[j][1],2)) <= 3)
				// std::cout<<"Is Colliding\n";
				return true;
		}
	}
	return false;
}

bool OptimalTrajectoryPlanner::isWithinKinematicConstraints(FrenetPath &path){
	if(path.maxVelocity>maxVelocity_ || path.maxAcceleration>maxAcceleration_ || path.maxCurvature>maxCurvature_){
		return false;
	}
	return true;
}

std::vector<FrenetPath> OptimalTrajectoryPlanner::isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles){
	std::vector<FrenetPath> validPaths;
	for(FrenetPath &path:paths){
	if (!isColliding(path, obstacles) && isWithinKinematicConstraints(path)){
			validPaths.push_back(path);
		}
	}
	return validPaths;
}

void OptimalTrajectoryPlanner::convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane){
	for(FrenetPath &path:paths){
		
		// Calculate x, y in world frame
		int j = 0;
		for(int i =0; i<path.s.size(); i++){
			double x, y, yaw;
			for(; j<centerLane.size(); j++){
				if(std::abs(path.s[i][0]-centerLane[j][4])<=0.1){
					x = centerLane[j][0];
					y = centerLane[j][1];
					yaw = centerLane[j][2];
					break;
				}
			}
			double d = path.d[i][0];
			path.world.push_back({x + d * std::cos(yaw + 1.57), y + d * std::sin(yaw + 1.57), 0, 0});
		}


		// Calculate Yaw in world Frame
		for(int i = 0; i<path.world.size()-1; i++){
			path.world[i][2] = std::atan2((path.world[i+1][1]-path.world[i][1]), (path.world[i+1][0]-path.world[i][0])); 
			path.world[i][3] = std::sqrt(std::pow(path.world[i+1][0]-path.world[i][0],2)+std::pow(path.world[i+1][1]-path.world[i][1],2)); 
		}
		path.world[path.world.size()-1][2]= path.world[path.world.size()-2][2];
		path.world[path.world.size()-1][3]= path.world[path.world.size()-2][3];

		// Calculate maximum curvature for the trajectory
		double curvature = INT_MIN;
		for(int i = 0; i<path.world.size()-1; i++){
			double tempCurvature = abs((path.world[i+1][2]-path.world[i][2])/(path.world[i][3]));
			if(curvature<tempCurvature)
				curvature=tempCurvature;
		}
		path.maxCurvature=curvature;
	}
}

cv::Point2i OptimalTrajectoryPlanner::windowOffset(float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
}

Eigen::Vector2d OptimalTrajectoryPlanner::rotation(double theta, double x, double y){
	Eigen::Matrix2d rotationZ;
	rotationZ << std::cos(theta), -1*std::sin(theta),
							 std::sin(theta), std::cos(theta);
	Eigen::Vector2d point;
	point << x, y;
	return rotationZ*point;
}

void OptimalTrajectoryPlanner::run(){

	// Center Lane
	std::vector<std::vector<double>> centerLane;
	double a = 0.05;
	double b = 3.5;
	double c = 2;
	double distanceTraced =0;
	double prevX=0;
	double prevY=4*(sin(std::pow((prevX*a-b),2) + (prevX*a-b) + c) + 1);
	for(double x{0}; x<140; x = x + 0.01){
		// Arbitary function describing a lane
		double y = 4*(sin(std::pow((x*a-b),2) + (x*a-b) + c) + 1);
		double dy = 4*cos(std::pow((x*a-b),2) + (x*a-b) + c)*(((2*a)*(x*a-b))+a);
		double ddy = 4*((-1*sin(std::pow((x*a-b),2) + (x*a-b) + c)*(std::pow((((2*a)*(x*a-b))+a),2))) + (cos(std::pow((x*a-b),2) + (x*a-b) + c)*(2*a*a)));
		
		double curvature = std::abs(ddy)/std::sqrt(std::pow((1+(dy*dy)),3));
		double yaw = dy;
		distanceTraced += std::sqrt(std::pow(x-prevX,2)+std::pow(y-prevY,2));
		centerLane.push_back({x, y, yaw, curvature, distanceTraced});
		prevX = x;
		prevY = y;
	}

	cv::namedWindow("Optimal Trajectory Planner", cv::WINDOW_NORMAL);

	// Obstacles along the lane
	std::vector<std::vector<double>> obstacles = {{10.58, 12,}, {60, 7.93}, {25.57, 0}, {94.42,-4}};
	double d0 = -laneWidth_; 
	double dv0 = 0;
	double da0 = 0;
	double s0 = 0;
	double sv0 = 10/3.6;
	double x = 0;
	double y = 0;
	// Run Trajectory Planner till the end of perceived/available Lane data
	while(true){
		// Get optimal Trajectory
		std::vector<FrenetPath> allPaths;
		FrenetPath p = optimalTrajectory(d0, dv0, da0, s0, sv0, centerLane, obstacles, allPaths);
		d0 = p.d[1][0];
		dv0 = p.d[1][1];
		da0 = p.d[1][2];
		s0 = p.s[1][0];
		sv0 = p.s[1][1];
		x = p.world[1][0];
		y = p.world[1][1];

		// Stop planner when within goal threshold
		if(std::sqrt(std::pow(centerLane[10000][0]-x,2)+std::pow(centerLane[10000][1]-y,2))<=1)
			break;	

		// visualize
		cv::Mat lane(4000, 10500, CV_8UC3, cv::Scalar(255, 255, 255));
		
		// Lane
		for(int i{1}; i<centerLane.size(); i++){
			cv::line(lane, windowOffset(centerLane[i-1][0], centerLane[i-1][1], lane.cols, lane.rows), windowOffset(centerLane[i][0], centerLane[i][1], lane.cols, lane.rows), cv::Scalar(0, 0, 0), 10);
		}

		// Obstacles
		for(int i{0}; i<obstacles.size(); i++){
			cv::circle(lane, windowOffset(obstacles[i][0], obstacles[i][1], lane.cols, lane.rows), 40, cv::Scalar(255, 0, 0), 60);
		}

		// Robot
		cv::line(lane, windowOffset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), lane.cols, lane.rows),
									 windowOffset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), lane.cols, lane.rows), cv::Scalar(0, 0, 0), 30);
		cv::line(lane, windowOffset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), lane.cols, lane.rows),
									 windowOffset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), lane.cols, lane.rows), cv::Scalar(0, 0, 0), 30);
		cv::line(lane, windowOffset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), lane.cols, lane.rows),
									 windowOffset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), lane.cols, lane.rows), cv::Scalar(0, 0, 0), 30);
		cv::line(lane, windowOffset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), lane.cols, lane.rows),
									 windowOffset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), lane.cols, lane.rows), cv::Scalar(0, 0, 0), 30);

		// All Trajectories
		for(FrenetPath &t:allPaths){
			for(int i{1}; i<t.world.size(); i++){
				cv::line(lane, windowOffset(t.world[i-1][0], t.world[i-1][1], lane.cols, lane.rows), windowOffset(t.world[i][0], t.world[i][1], lane.cols, lane.rows), cv::Scalar(0, 0, 255), 5);
			}
		}

		// Trajectory
		for(int i{1}; i<p.world.size(); i++){
			cv::line(lane, windowOffset(p.world[i-1][0], p.world[i-1][1], lane.cols, lane.rows), windowOffset(p.world[i][0], p.world[i][1], lane.cols, lane.rows), cv::Scalar(0, 255, 0), 30);
		}

		cv::resizeWindow("Optimal Trajectory Planner", 1050, 400);
		cv::imshow("Optimal Trajectory Planner", lane);
		cv::waitKey(1);
	}
}

