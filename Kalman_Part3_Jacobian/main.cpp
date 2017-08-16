#include <iostream>
#include <vector>
#include "Eigen/Dense"


Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	Eigen::VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	Eigen::MatrixXd Hj = CalculateJacobian(x_predicted);

	std::cout << "Hj:" << std::endl << Hj << std::endl;

	return 0;
}

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) {

	Eigen::MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px==0 && py==0)
	{
	    std::cout<<"CalculateJacobian () - Error - Division by Zero"<<"\n";
	}
	else
	{
	    //compute the Jacobian matrix
	    Hj<< px/std::sqrt(std::pow(px,2)+std::pow(py,2)), py/std::sqrt(std::pow(px,2)+std::pow(py,2)), 0, 0,
	        -py/(std::pow(px,2)+std::pow(py,2)), px/(std::pow(px,2)+std::pow(py,2)), 0, 0,
	        (py*(vx*py-vy*px))/std::pow(std::pow(px,2)+std::pow(py,2),1.5),
	        (px*(vy*px-vx*py))/std::pow(std::pow(px,2)+std::pow(py,2),1.5),
	        px/std::sqrt(std::pow(px,2)+std::pow(py,2)), py/std::sqrt(std::pow(px,2)+std::pow(py,2));

	}


	return Hj;
}
