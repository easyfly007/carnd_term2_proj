#ifndef __TRACKING_H__
#define __TRACKING_H__

#include <vector>
#include <string>
#include <fstream>
#include "measurement_package.h"
#include "kalman_filter.h"

class Tracking{
public:
	Tracking();
	virtual ~Tracking();
	void processMeasurement(const MeasurementPackage &measurement_pack);
	KalmanFilter kf_;

private:
	bool is_initialized_;
	int64_t previous_timestamp_;

	// acceleration noise component
	float noise_ax;
	float noise_ay;
};
#endif