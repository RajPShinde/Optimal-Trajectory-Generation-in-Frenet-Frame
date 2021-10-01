#ifndef INCLUDE_FRENETPATH_HPP_
#define INCLUDE_FRENETPATH_HPP_

#include <vector>
#include <cmath>
#include <iostream>

class FrenetPath
{
    public:
        std::vector<std::vector<double>> s;       // Longitudional Data
        std::vector<std::vector<double>> d;       // Latitudional Data
        std::vector<std::vector<double>> world;
        double T;                                // Prediction Time
        double cf;                        // Cost
        double jd, js;
        double maxVelocity = INT_MIN;
        double maxAcceleration = INT_MIN; 
        double maxCurvature = INT_MIN; 
};

#endif  //  INCLUDE_FRENETPATH_HPP_