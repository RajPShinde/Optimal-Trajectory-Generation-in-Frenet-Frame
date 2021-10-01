#ifndef INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_
#define INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
#include <polynomial.hpp>
#include <frenetPath.hpp>

class OptimalTrajectoryPlanner
{
    public:

        OptimalTrajectoryPlanner();

        ~OptimalTrajectoryPlanner();

        bool isColliding();

        std::vector<std::vector<double>> optimalPath();

        void run();

    private:
        double pi_ = 3.14159;
        double maxSpeed;
        double maxAcceleration;
        double robotFootprint = 2;
        double maxSteeringAngle;
        double maxPredictionStep;
        double minPredictionStep;
        double noOfLanes;
        double laneWidth;
        double timeStep = 0.1;
        double roadWidth = 10;
        int KLat, KLon, kj, kt, ks;
};

#endif  //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_