#include <polynomial.hpp>

Polynomial::Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T):
					   type_("quintic"), x0_(x0), v0_(v0), a0_(a0), xT_(xT), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0/2) {
	Eigen::Matrix3d A;
	Eigen::Vector3d B;
	A << std::pow(T_,3), std::pow(T_,4), std::pow(T_,5),
		 3 * std::pow(T_,2), 4 * std::pow(T_,3), 5 * std::pow(T_,4),
		 6 * T_, 12 * std::pow(T_,2), 20 * std::pow(T_,3);

	B << xT - a1_ - a2_ * T_ - a3_ * std::pow(T_,2),
		 vT - a2_ - 2 * a3_ * T_,
		 aT - 2 * a3_;

	//  Solve for x in Ax=B
	Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0); 
	a5_ = coefficients(1);
	a6_ = coefficients(2);
}

Polynomial::Polynomial(double x0, double v0, double a0, double vT, double aT, double T):
				       type_("quartic"), x0_(x0), v0_(v0), a0_(a0), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0/2) {
	Eigen::Matrix2d A;
	Eigen::Vector2d B; 
	A << 3 * std::pow(T_,2), 4 * std::pow(T_,3),
	     6 * T_, 12 * std::pow(T_,2);

	B << vT - a2_ - 2 * a3_ * T_,
	     aT - 2 * a3_;

	//  Solve for x in Ax=B
	Eigen::Vector2d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0); 
	a5_ = coefficients(1);
}

Polynomial::~Polynomial() {

}

double Polynomial::position(double t){
	// solve
	if(type_=="quintic"){
		return a1_ + a2_ * t + a3_ * std::pow(t,2) + a4_ * std::pow(t,3) + a5_ * std::pow(t,4) + a6_ * std::pow(t,5);
	}
	return a1_ + a2_ * t + a3_ * std::pow(t,2) + a4_ * std::pow(t,3) + a5_ * std::pow(t,4);
}

double Polynomial::velocity(double t){
	// solve first derivative
	if(type_=="quintic"){
		return a2_ + 2 * a3_ * t + 3 * a4_ * std::pow(t,2) + 4 * a5_ * std::pow(t,3) + 5 * a6_ * std::pow(t,4);
	}
	return a2_ + 2 * a3_ * t + 3 * a4_ * std::pow(t,2) + 4 * a5_ * std::pow(t,3);
}

double Polynomial::acceleration(double t){
	// solve second derivative
	if(type_=="quintic"){
		return 2 * a3_ + 6 * a4_ * t + 12 * a5_ * std::pow(t,2) + 20 * a6_ * std::pow(t,3);
	}
	return 2 * a3_ + 6 * a4_ * t + 12 * a5_ * std::pow(t,2);
}

double Polynomial::jerk(double t){
	// solve third derivative
	if(type_=="quintic"){
		return 6 * a4_ + 24 * a5_ * t + 60 * a6_ * std::pow(t,2);
	}
	return 6 * a4_ + 24 * a5_ * t;
}