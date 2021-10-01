#include <polynomial.hpp>

Polynomial::Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T): type_("quintic") {
	Eigen::Matrix3f A, B;
}

Polynomial::Polynomial(double x0, double v0, double a0, double vT, double aT, double T):type_("quartic"){
	Eigen::Matrix3f A, B;
}

Polynomial::~Polynomial() {

}

double Polynomial::position(double t){
	// solve
	if(type_="quintic"){
		return a0_ + a1_ * t + a2_ * std:pow(t,2) + a3_ * std:pow(t,3) + a4_ * std:pow(t,4) + a5_ * std:pow(t,5);
	}
	return a0_ + a1_ * t + a2_ * std:pow(t,2) + a3_ * std:pow(t,3) + a4_ * std:pow(t,4);
}

double Polynomial::velocity(double t){
	// solve first derivative
	if(type_="quintic"){
		return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t,2) + 4 * a4_ * std::pow(t,3) + 5 * a5_ * std::pow(t,4);
	}
	return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t,2) + 4 * a4_ * std::pow(t,3);
}

double Polynomial::acceleration(double t){
	// solve second derivative
	if(type_="quintic"){
		return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t,2) + 20 * a5_ * std::pow(t,3);
	}
	return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t,2);
}

double Polynomial::jerk(double t){
	// solve third derivative
	if(type_="quintic"){
		return 6 * a3_ + 24 * a4_ * t + 60 * a5_ * std::pow(t,2);
	}
	return 6 * a3_ + 24 * a4_ * t;
}

	//  Solve for x in Ax=B
	Eigen::Vector3f coefficients = 
	a3_ = coefficients(0); 
	a4_ = coefficients()
	a5_ =