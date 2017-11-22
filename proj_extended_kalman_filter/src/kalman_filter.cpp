#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

extern bool debug;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  x_ = Eigen::VectorXd(4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  if (debug)
    cout << "KalmanFilter::Predict() begin" << endl;
  x_ = F_ * x_;
  if (debug)
    cout << " step 1" << endl;
  MatrixXd Ft = F_.transpose();
  if (debug)
    cout << " step 2" << endl;
  P_ = F_ * P_ * Ft + Q_;
  if (debug)
    cout << "KalmanFilter::Predict() end" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  // z is polar coordinate, with size (3)

  // please note for y, we will still use h(x_), rather than H_ * x_
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  VectorXd z_pred = VectorXd(3);
  z_pred(0) = sqrt(px*px + py*py);
  z_pred(1) = atan(py/px);
  z_pred(2) = (px * vx + py * vy) / sqrt( px*px + py*py);
  
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
