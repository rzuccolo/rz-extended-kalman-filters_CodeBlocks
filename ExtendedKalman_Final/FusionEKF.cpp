#include <iostream>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "measurement_package.h"

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  //measurement covariance matrix - laser
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //set the process noise covariance matrix Q
  ekf_.Q_ = Eigen::MatrixXd(4, 4);


  //the initial transition matrix F_
  ekf_.F_ = Eigen::MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	         0, 1, 0, 1,
	         0, 0, 1, 0,
	         0, 0, 0, 1;

  //measurement matrix
  H_laser_ = Eigen::MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement matrix for radar
  Hj_ = Eigen::MatrixXd(3, 4);

  //state covariance matrix P
  ekf_.P_ = Eigen::MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    std::cout << "Initializing EKF: " << std::endl;
    ekf_.x_ = Eigen::VectorXd(4);
    ekf_.x_ << 0, 0, 5.5, 0.5;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ << tools.PolarToCartesian(measurement_pack.raw_measurements_);


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

    }

    //we can't let x and y be zero.
    if ( std::fabs(ekf_.x_(0)+ekf_.x_(1)) < 1e-4){
  		ekf_.x_(0) = 1e-4;
  		ekf_.x_(1) = 1e-4;
  	}

    //get initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds, 10^6 converts microseconds to seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Update the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  //Update the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
	      0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
	      dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
	      0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //Make Prediction only if we the dt is big enough.
  if ( dt > 1e-3 )
  {
     ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //measurement covariance matrix
    ekf_.R_ = R_radar_;

    //measurement matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    //Update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    //measurement covariance matrix
    ekf_.R_ = R_laser_;

    //measurement matrix
    ekf_.H_ = H_laser_;

    //Update
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //std::cout << "x_ = " << ekf_.x_ << std::endl;
  //std::cout << "P_ = " << ekf_.P_ << std::endl;
}
