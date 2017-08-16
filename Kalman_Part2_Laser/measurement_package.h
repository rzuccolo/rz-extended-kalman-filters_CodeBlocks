#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
	long timestamp_; // Time member of the class

	enum SensorType {LASER, RADAR}; // enumerator, associates 0 to LASER and 1 RADAR, but facilitates since one can works with names instead of numbers

	SensorType sensor_type_; // Sensor type member of the class

	Eigen::VectorXd raw_measurements_; //  Vector of raw measurements

};

#endif /* MEASUREMENT_PACKAGE_H_ */
