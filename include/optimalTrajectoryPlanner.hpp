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

        FrenetPath optimalPath();

        void trajectoryCost(FrenetPath &path);

        bool isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles);

        void isWithinKinematicConstraints(FrenetPath &path);

        std::vector<FrenetPath> isValid(std::vector<FrenetPath> paths);

        void run();

    private:
        double pi_ = 3.14159;
        double maxVelocity_;
        double maxAcceleration_;
        double maxSteeringAngle_;
        double maxCurvature_;
        double robotFootprint_ = 2;
        double maxSteeringAngle_;
        double maxPredictionStep_;
        double minPredictionStep_;
        double noOfLanes_;
        double laneWidth_;
        double timeStep_ = 0.1;
        double roadWidth_ = 10;
        int klat_, klon_, kj_, kt_, ks_;
};

#endif  //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_