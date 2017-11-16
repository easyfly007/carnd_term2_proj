#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__
#include "Dense"

using Eigen::MatrixXd;
using eigen::VectorXd;

class KalmanFilter {
public:
	VectorXd x_;
	// state vector

	MatrixXd P_;
	// state covariance matrix

	MatrixXd F_;
	// state transition matrix

	MatrixXd Q_;
	// process covariance matrix

	MatrixXd H_;
	// measurement matrix

	MatrixXd R_;
	// measurement convariance matrix

	KalmanFilter();
	// constructor function

	virtual ~KalmanFIlter();
	// destructure function

	void predict();
	// predict the state and the state covariance
	// using the process model

	void update(const VectorXd &z);
	// update the state and z, which is the measurement at k+1
}

#endif
