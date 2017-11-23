#include "kalman_filter.h"
#include <iostream>
#include <math.h>

// for pi value visit, M_PI
#define _USE_MATH_DEFINES

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
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
  // normalize the theta, which is y(1)
  double d_theta = y(2);
  while (d_theta > M_PI)
    d_theta -= 2*M_PI;
  while (d_theta < -M_PI)
    d_theta += 2*M_PI;
  y(2) = d_theta;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  while (x_(1) > M_PI)
    x_(1) -= 2*M_PI;
  while (x_(1) < M_PI)
    x_(1) += 2*M_PI;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
