#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{}

KalmanFilter::~KalmanFilter()
{}

void KalmanFilter::predict()
{
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::update(const VectorXd &z)
{
	VectorXd z_pred = H_ *x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(z_size, x_size);
	P_ = (I - K * H_) * P_;
}