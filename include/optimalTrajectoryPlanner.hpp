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
#include <Eigen/Dense>

class OptimalTrajectoryPlanner
{
    public:

        OptimalTrajectoryPlanner();

        ~OptimalTrajectoryPlanner();

        bool isColliding();

        FrenetPath optimalTrajectory(double d0, double dv0, double da0,
                                     double s0, double sv0,
                                     std::vector<std::vector<double>> &centerLane,
                                     std::vector<std::vector<double>> &obstacles,
                                     std::vector<FrenetPath> &allPaths);

        void trajectoryCost(FrenetPath &path);

        bool isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles);

        bool isWithinKinematicConstraints(FrenetPath &path);

        std::vector<FrenetPath> isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles);

        std::vector<FrenetPath> isValid(std::vector<FrenetPath> &paths);

        void convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane);

        cv::Point2i windowOffset(float x, float y, int image_width, int image_height);

        Eigen::Vector2d rotation(double theta, double x, double y);

        void run();

    private:
        double pi_ = 3.14159;
        double maxVelocity_ = 50/3.6;
        double maxAcceleration_ = 2;
        double maxSteeringAngle_ = 0.7;
        double maxCurvature_ = 1;
        double robotFootprint_ = 1.5;
        double maxPredictionStep_ = 5;
        double minPredictionStep_ = 4;
        double noOfLanes_ = 5;
        double laneWidth_ = 4;
        double targetVelocity_ = 30/3.6;
        double velocityStep_ = 5/3.6;
        double timeStep_ = 0.1;
        double klat_ = 1;
        double klon_ = 1;
        double kjd_ = 0.1;
        double ktd_ = 0.1;
        double ksd_ = 0.7;
        double kjs_ = 0.1;
        double kts_ = 0.1;
        double kss_ = 1;
};

#endif  //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_