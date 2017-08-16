#include "kalman_filter.h"
#include "Eigen/Dense"

/*
 * Constructor.
 */
KalmanFilter::KalmanFilter() {}


/*
 * Destructor.
 */
KalmanFilter::~KalmanFilter() {}


/*
 * Initialization.
 */
void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


void KalmanFilter::Predict() {
  /**
    * predict the state
  */

  x_ = F_ * x_;
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */

  Eigen::VectorXd z_pred = H_ * x_;
  Eigen::VectorXd y = z - z_pred;
  CallRestOfUpdate(y);

}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  Eigen::VectorXd hx = tools.CartesianToPolar(x_);
  Eigen::VectorXd y = z - hx;

  const long double PI = 3.141592653589793238L;

  //Normalization of angles
  while(y(1) > PI || y(1) < -PI)
    {
      if(y(1) > PI)
        y(1) -= 2*PI;
      else y(1) += 2*PI;
    }

  CallRestOfUpdate(y);

}

void KalmanFilter::CallRestOfUpdate(const Eigen::VectorXd &y) {

  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
