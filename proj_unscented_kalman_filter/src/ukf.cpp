#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// for pi value visit, M_PI
#define _USE_MATH_DEFINES

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Parameters above this line are scaffolding, do not modify
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

   // initialization
  if (!is_initialized_) {
    is_initialized_ = true;
    time_us_ = measurement_pack.timestamp_;
  
    double px, py, v, yaw, yawd;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      double ro = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      double ro_dot = measurement_pack.raw_measurements_[2];
      while (theta > M_PI)
        theta -= 2 * M_PI;
      while (theta < - M_PI)
        theta += 2 * M_PI;

      // change from radar space to CTRV space
      px = ro * cos(theta);
      py = ro * sin(theta);
      v  = ro_dot;
      yaw = theta;
      yawd = 0.0;
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1000;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      v  = 0.0;
      yaw = atan2(py, px);
      yawd = 0.0;
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1000, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1000;
    }
    x_(0) = px;
    x_(1) = py;
    x_(2) = v;
    x_(3) = yaw;
    x_(4) = yawd;
  }

  float dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
  time_us_ = measurement_pack.timestamp_;
  
  // prediction
  Predict(dt);

  // update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(measurement_pack);
  }
  else
  {
    UpdateLidar(measurement_pack);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  // 1. build augmented sigma popints
  VectorXd x_aug = VectorXd(n_aug_); // the mean for the augmented value
  x_aug.fill(0.0);
  x_aug.head(n_x) = x_;

  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug.fill(0.0);
  
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P_;
  P_aug(n_x, n_x) = std_a_ * std_a_;
  P_aug(n_x + 1, n_x + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd offset = sqrt(lambda_ + n_aug_) * (P_aug.llt.matrixL());
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i ++)
  {
    Xsig_aug.col(i + 1) = x_aug + offset.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - offset.col(i);
  }

  // 2. predict the new state by the augmented sigma time points, through delta_t  
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    vectorXd onePoint = VectorXd(n_x_);
    double px    = Xsig_aug(0, i);
    double py    = Xsig_aug(1, i);
    double v     = Xsig_aug(2, i);
    double yaw   = Xsig_aug(3, i);
    double yawd  = Xsig_aug(4, i);
    double va    = Xsig_aug(5, i);
    double yawdd = Xsig_aug(6, i);
    if (fabs(yawd) < 1.0e-5)
    {
      Xsig_pred(0, i) = Xsig_aug(0, i) + v * cos(yaw) * delta_t + 0.5 * cos(yaw) * va * delta_t * delta_t;
      Xsig_pred(1, i) = Xsig_aug(1, i) + v * sin(yaw) * delta_t + 0.5 * sin(yaw) * va * delta_t * delta_t;
    }
    else
    {
      Xsig_pred(0, i) = Xsig_aug(0, i) + v/yawd *(sin(yaw + delta_t * yawd) - sin(yaw))
        + 0.5 * cos(yaw) * va * delta_t * delta_t;
      Xsig_pred(1, i) = Xsig_aug(1, i) + v / yawd * (-cos(yaw+ delta_t * yawd) + cos(yaw))
        + 0.5 * sin(yaw) * va * delta_t * delta_t;
    }

    Xsig_pred(2, i) = Xsig_aug(2, i) + va * delta_t;
    Xsig_pred(3, i) = Xsig_aug(3, i) + yawd * delta_t + 0.5 * yawdd * delta_t * delta_t;
    Xsig_pred(4, i) = Xsig_aug(4, i) + yawdd * delta_t;
  }

  // 3. calculate the predicted sigma points mean and covariance
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i ++)
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));

  x_ = VectorXd(n_x_);
  P_ = MatrixXd(n_x_, n_x_);
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    x_ += weights_(i)* Xsig_pred.col(i);
  }
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    P_ += weights(i) * (Xsig_pred.col(i) - x_) * (Xsig_pred.col(i) - x_).transpose(); 
  }

  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
