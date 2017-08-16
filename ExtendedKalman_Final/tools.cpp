#include <iostream>
#include "Eigen/Dense"
#include "tools.h"


/*
 * Constructor.
 */
Tools::Tools() {}

/*
 * Destructor.
 */
Tools::~Tools() {}


Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  Eigen::VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
		|| estimations.size() == 0){
	std::cout << "Invalid estimation or ground_truth data" << std::endl;
	return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

	Eigen::VectorXd residual = estimations[i] - ground_truth[i];

	//coefficient-wise multiplication
	residual = residual.array()*residual.array();
	rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;



}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  Eigen::MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(std::fabs(c1) < 0.0001){
	std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
	return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
	  -(py/c1), (px/c1), 0, 0,
	  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}


const Eigen::VectorXd Tools::PolarToCartesian(const Eigen::VectorXd& polar_vector) {
  Eigen::VectorXd cart_vector(4);

  float rho = polar_vector(0);
  float phi = polar_vector(1);
  float rho_dot = polar_vector(2);

  float x  = rho * cos(phi);
  float y  = rho * sin(phi);
  float vx = rho_dot * cos(phi);
  float vy = rho_dot * sin(phi);

  cart_vector << x, y, vx, vy;
  return cart_vector;
}


const Eigen::VectorXd Tools::CartesianToPolar(const Eigen::VectorXd& x_state) {
  Eigen::VectorXd polar_vector(3);

  float x = x_state(0);
  float y = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if (std::fabs(x+y) < 1e-4) {
    x = 1e-4;
    y = 1e-4;
  }

  float rho = sqrt(x*x + y*y);
  float phi = atan2(y,x);
  float rho_dot = (x*vx + y*vy)/rho;

  polar_vector << rho ,phi, rho_dot;
  return polar_vector;
}
