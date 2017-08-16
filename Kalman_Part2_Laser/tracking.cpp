#include <iostream>
#include "Eigen/Dense"
#include "tracking.h"


Tracking::Tracking() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	//create a 4D state vector, we don't know yet the values of the x state
	kf_.x_ = Eigen::VectorXd(4); // px, py, vx, vy

	//state covariance matrix P
	kf_.P_ = Eigen::MatrixXd(4, 4); // we are certain about initial position coming from Lidar, so terms = 1, but not certain about velocity, so terms = 1000
	kf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//measurement covariance
	kf_.R_ = Eigen::MatrixXd(2, 2);  // given by sensor manufacturer sigmapx^2, sigmapy^2
	kf_.R_ << 0.0225, 0,
			  0, 0.0225;

	//measurement matrix
	kf_.H_ = Eigen::MatrixXd(2, 4); // this is for LIDAR, brings 4D process state vector to 2D measurement vector, we get rid of velocity inf
	kf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//the initial transition matrix F_
	kf_.F_ = Eigen::MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	//set the acceleration noise components
	noise_ax = 5;  // sigmaax^2
	noise_ay = 5;  // sigmaay^2


	//process covariance matrix Q
	kf_.Q_ = Eigen::MatrixXd(4, 4);

}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	if (!is_initialized_) {
		//cout << "Kalman Filter Initialization " << endl;

		//set the state with the initial location and zero velocity
		kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; // px, py, vx, vy

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds,  dividing by 10^6 to convert microseconds to seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	//1. Modify the F matrix so that the time is integrated
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;
	//2. Set the process covariance matrix Q
	kf_.Q_ << std::pow(dt,4)*noise_ax/4, 0, std::pow(dt,3)*noise_ax/2, 0,
			  0, std::pow(dt,4)*noise_ay/4, 0, std::pow(dt,3)*noise_ay/2,
			  std::pow(dt,3)*noise_ax/2, 0, std::pow(dt,2)*noise_ax, 0,
			  0, std::pow(dt,3)*noise_ay/2, 0, std::pow(dt,2)*noise_ay;
	//3. Call the Kalman Filter predict() function
	kf_.Predict();
	//4. Call the Kalman Filter update() function
	// with the most recent raw measurements_
	kf_.Update(measurement_pack.raw_measurements_);

	std::cout << "x_= " << kf_.x_ << std::endl;
	std::cout << "P_= " << kf_.P_ << std::endl;

}

