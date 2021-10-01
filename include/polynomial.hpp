#ifndef INCLUDE_POLYNOMIAL_HPP_
#define INCLUDE_POLYNOMIAL_HPP_

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <Eigen/Dense>

class Polynomial
{
    public:

        Polynomial();

        Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T);

        Polynomial(double x0, double v0, double a0, double vT, double aT, double T);

        ~Polynomial();

        double position(double t);

        double velocity(double t);

        double acceleration(double t);

        double jerk(double t);

    private:
        double a1_, a2_, a3_, a4_, a5_, a6_;
        double x0_, v0_, a0_, xT_, vT_, aT_, T_;
        std::string type_;

};

#endif  //  INCLUDE_POLYNOMIAL_HPP_