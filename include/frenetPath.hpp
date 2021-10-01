#ifndef INCLUDE_FRENETPATH_HPP_
#define INCLUDE_FRENETPATH_HPP_

#include <vector>
#include <cmath>
#include <iostream>

class FrenetPath
{
    public:
        std::vector<std::vector<std::vector<double>>> s;       // Longitudional Data
        std::vector<std::vector<std::vector<double>>> d;       // Latitudional Data
        double T;                                 // Prediction Time
        double cd, ct, cv;                        // Cost's
        int KLat, KLon, kj, kt, ks;               // Gains
};

#endif  //  INCLUDE_FRENETPATH_HPP_