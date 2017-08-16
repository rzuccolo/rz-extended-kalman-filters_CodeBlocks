// Writing a function 'filter()' that implements a 1D Kalman Filter,
// but using multi-dimensional equations
//============================================================================
#include <iostream>
#include "Eigen/Dense"
#include <vector>


//Kalman Filter variables
Eigen::VectorXd x;	// Object state
Eigen::MatrixXd P;	// Object covariance matrix
Eigen::VectorXd u;	// External motion
Eigen::MatrixXd F; // State transition matrix
Eigen::MatrixXd H;	// Measurement matrix
Eigen::MatrixXd R;	// Measurement covariance matrix
Eigen::MatrixXd I; // Identity matrix
Eigen::MatrixXd Q;	// Process covariance matrix

std::vector<Eigen::VectorXd> measurements;
void filter(Eigen::VectorXd &x, Eigen::MatrixXd &P);


int main() {
    //design the KF with 1D motion
	x = Eigen::VectorXd(2); // Object state here is defined with position and velocity
	x << 0, 0; // initial null values for both position and velocity, initially we know nothing

	P = Eigen::MatrixXd(2, 2); // Object covariance/uncertainty matrix
	P << 1000, 0, 0, 1000; // high initial uncertanty, only diagonal terms which means initiallly there is not correlation
                           // between position and velocity

	u = Eigen::VectorXd(2); // External motion is unknow, that would be for known acceleraion/deacceleration of object
	u << 0, 0;

	F = Eigen::MatrixXd(2, 2); // State transition matrix, designed for simple liner motion equations; x'=x*1+v*dt with dt=1; v'=x*0+v*1
	F << 1, 1, 0, 1;

	H = Eigen::MatrixXd(1, 2); // Measurement matrix, designed to measurement update z=x'*1+v'*0
	H << 1, 0;

	R = Eigen::MatrixXd(1, 1); // Measurement covariance/uncertainty, that must come based on sensor manfactured and calibration
	R << 1;

	I = Eigen::MatrixXd::Identity(2, 2); // Indentity matrix

	Q = Eigen::MatrixXd(2, 2); // Process covariance/uncertainty matrix, here is null, but would need to be calibrated
	Q << 0, 0, 0, 0;

	//create a list of measurements
	Eigen::VectorXd single_meas(1);
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
	measurements.push_back(single_meas);

	//call Kalman filter algorithm
	filter(x, P);

	return 0;

}


void filter(Eigen::VectorXd &x, Eigen::MatrixXd &P) {

	for (unsigned int n = 0; n < measurements.size(); ++n) {

		Eigen::VectorXd z = measurements[n]; // measurement for update
		Eigen::VectorXd Y; // Measurement error
		Eigen::MatrixXd S, K; // Weighting uncertanty and Kalman gain

		// KF Measurement update step
		/*
        Y = Z - H * x
        S = H * P * Ht + R
        K = P * Ht * Sinv
        x' = x + K*Y
        P' = (I - K * H ) * P
        */
        Y = z - H * x;  // Measurement error calculation
        S = H * P * H.transpose() + R;  // weighting the uncertanty, measurement (R, noise) or prediction (P)
        K = P * H.transpose() * S.inverse(); // Kalman gain

		// new state
		x = x + K * Y;  //new state position for the moment being, right now
        P = (I - K * H) * P; //new state uncertanty for the moment being, right now

		// KF Prediction step
        // x' = F * x + u
        // P' = F * P * Ft + Q
        x = F * x + u; //state position prediction for the next step
        P = F * P * F.transpose() + Q; //state uncertanty prediction for the next step

		std::cout << "x=" << std::endl <<  x << std::endl;
		std::cout << "P=" << std::endl <<  P << std::endl;


	}
}
