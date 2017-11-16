#ifndef __MEASUREMENT_PACKAGE_H__
#define __MEASUREMENT_PACKAGE_H__

#include "Dense"

class MeasurementPackage{
public:
	int64_t timestamp_;
	enum SensorType{
		LASER, RADAR
	} sensor_type_;

	Eigen::VectorXd raw_measurements_;
}

#endif