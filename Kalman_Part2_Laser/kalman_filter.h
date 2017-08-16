#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"


class KalmanFilter {
public:

	///* state vector
	Eigen::VectorXd x_;

	///* state covariance matrix
	Eigen::MatrixXd P_;

	///* state transistion matrix
	Eigen::MatrixXd F_;

	///* process covariance matrix
	Eigen::MatrixXd Q_;

	///* measurement matrix
	Eigen::MatrixXd H_;

	///* measurement covariance matrix
	Eigen::MatrixXd R_;

	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Predict Predicts the state and the state covariance
	 * using the process model
	 */
	void Predict();

	/**
	 * Updates the state and
	 * @param z The measurement at k+1
	 */
	void Update(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */


