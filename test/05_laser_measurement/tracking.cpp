#include "Dense"
#include <iostream>
#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tracking::Tracking()
{
	is_initialized_ = false;
	kf_.x_ = VectorXd(4);
	// create a 4D state vector, we don't know yet the value of the x state

	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ = << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;
	// state covariance matrix P

	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ <<  0.0255, 0,
				0, 0.0255;
	// measurement covarience, which should be provided by sensor vendor

	kf_.H_ = MatrixXd(2, 4);
	kf_.H_ << 1, 0, 0, 0,
				0, 1, 0, 0;
	// measurement matrix

	kf_.F_ = MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
				0, 1, 0, 1,
				0, 0, 1, 0,
				0, 0, 0, 1;
	// the initial transition matrix F_

	noise_ax = 5;
	noide_ay = 5;
	// the acceleration noise component
}

Tracking::~Tracking()
{}

void Tracking::processMeasurement(const MeasurementPackage &measurement_pack)
{
	if (is_initialized_)
	{
		// cout << "Kalman Filter Initializing" << endl;
		kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// modify the F matrix to consider time step
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;

	// set process covariance matrix Q
	kf_.Q_ = MatrixXd(4, 4);
	dt2 = dt * dt;
	dt3 = dt2 * dt;
	dt4 = dt3 * dt;
	kf_.Q_ << dt4 /4.0 * noise_ax, dt3 * noise_ax / 2, 0, 0,
		0, dt4 / 4.0 * noise_ay, 0, dt3 * noise_ay / 2.0,
		dt3 * noise_ax / 2, 0, dt2 * noise_ax, 0,
		0, dt3 * noise_ay/ 2, 0, dt2 * noise_ay;

	// predict
	kf_.predict();

	// update with the measurement result
	kf_.update(measurement_pack.raw_measurements_);

	std::cout << " x_ = " << kf_.x_ << std::endl;
	std::cout << " P_ = " << kf_.P_ << std::endl;

}