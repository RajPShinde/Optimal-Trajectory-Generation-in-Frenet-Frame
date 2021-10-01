#ifndef INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_
#define INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>

class OptimalTrajectoryPlanner
{
    public:

        OptimalTrajectoryPlanner();

        ~OptimalTrajectoryPlanner();

        bool isColliding();

        void optimalPath();

        void run();

    private:
        double pi_ = 3.14159;
        double maxSpeed;
        double maxAcceleration;
        double robotFootprint = 2;
        double predictionStep;
        double timeStep = 0.1;
        double roadWidth = 10;
};

#endif  //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_