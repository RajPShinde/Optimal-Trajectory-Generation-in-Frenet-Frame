#ifndef INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_
#define INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
#include <polynomial.hpp>
#include <frenetPath.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

        std::vector<FrenetPath> isValid(std::vector<FrenetPath> &paths);

        void convertToWorldFrame(std::vector<FrenetPath> &paths);

        cv::Point2i windowOffset(float x, float y, int image_width=2000, int image_height=2000)

        void run();

    private:
        double pi_ = 3.14159;
        double maxVelocity_ = 15;
        double maxAcceleration_ = 2;
        double maxSteeringAngle_ = 0.7;
        double maxCurvature_ = 1;
        double robotFootprint_ = 2;
        double maxPredictionStep_ = 5;
        double minPredictionStep_ = 4;
        double noOfLanes_ = 3;
        double laneWidth_ = 2;
        double targetVelocity = 3;
        double timeStep_ = 0.1;
        double klat_ = 1;
        double klon_ = 1;
        double kj_ = 1;
        double kt_ = 1;
        double ks_ = 1;
        int distanceTracedIndex_ = 0;
};

#endif  //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_