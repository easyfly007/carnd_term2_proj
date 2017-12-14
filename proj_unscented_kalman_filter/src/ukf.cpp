#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool debug = true;

// for pi value visit, M_PI
#define _USE_MATH_DEFINES

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  if (debug)
    cout << "begin UKF::UKF()" << endl;
  
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;
  std_a_ = 0.2; // suppose the a_max is 2 m /s^2, then use std_a = 0.5 * a_max

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  std_yawdd_ = 0.2; // shall we change it to smaller?

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
  n_aug_ = 7;
  
  n_x_ = 5;

  is_initialized_ = false;
  
  // Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i ++)
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  

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

  use_radar_ = false;
  use_laser_ = false;

  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    use_laser_ = true;
  else
    use_radar_ = true;
  
  if (debug)
  {
    cout << "\n\nbegin UKF::ProcessMeasurement()";
    if (use_radar_)
    {
      cout << "  meas = radar" << endl;
      cout << "    row   = " << meas_package.raw_measurements_[0] << endl;
      cout << "    theta = " << meas_package.raw_measurements_[1] << endl;
      cout << "    ro_dot= " << meas_package.raw_measurements_[2] << endl;
    }
    else
    {
      cout << "  meas = lidar" << endl;
      cout << "    px = " << meas_package.raw_measurements_[0] << endl;
      cout << "    py = " << meas_package.raw_measurements_[1] << endl;
    }

  }
  
   // initialization
  if (!is_initialized_) 
  {
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  
    double px, py, v, yaw, yawd;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      double ro = meas_package.raw_measurements_[0];
      double theta = meas_package.raw_measurements_[1];
      double ro_dot = meas_package.raw_measurements_[2];
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
            0, 0, 100, 0, 0,
            0, 0, 0, 100, 0,
            0, 0, 0, 0, 1000;
      // px and py are high accurate, 
      // v is less accurate, yaw and yawd is actually not known
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
    {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      v  = 0.0;
      yaw = 0.0;
      yawd = 0.0;
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1000, 0, 0,
            0, 0, 0, 1000, 0,
            0, 0, 0, 0, 1000;
    }
    x_(0) = px;
    x_(1) = py;
    x_(2) = v;
    x_(3) = yaw;
    x_(4) = yawd;
    
    if (debug)
      cout << "the initial value for x_ = " << endl << x_ << endl;
    if (debug)
      cout << "the initial value for P_ = " << endl << P_ << endl;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  // prediction
  Prediction(dt);

  // update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    UpdateLidar(meas_package);
  }
  if (debug)
    cout << "end UKF::ProcessMeasurement()" << endl;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  if (debug)
    cout << "\nbegin UKF::Prediction(), delta_t = " << delta_t << endl;
  if (debug)
    cout << "the old x_ = " << endl << x_ << endl;
  if (debug)
    cout << "the old P_ = " << endl << P_ << endl;

  // 1. build augmented sigma popints
  VectorXd x_aug = VectorXd(n_aug_); // the mean for the augmented value
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);

  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i ++)
  {
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  if (debug)
    cout << "build Xsig_aug = " << endl << Xsig_aug << endl;
  if (debug)
    cout << "build x_aug = " << endl << x_aug << endl;

  // 2. predict the new state by the augmented sigma time points, through delta_t  
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    double px    = Xsig_aug(0, i);
    double py    = Xsig_aug(1, i);
    double v     = Xsig_aug(2, i);
    double yaw   = Xsig_aug(3, i);
    double yawd  = Xsig_aug(4, i);
    double nu_a    = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double px_p, py_p;
    if (fabs(yawd) < 0.001)
    {
      px_p = px + v * delta_t * cos(yaw);
      py_p = py + v * delta_t * sin(yaw);
    }
    else
    {
      px_p = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = py + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd + yawd;

    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  if (debug)
    cout << "build Xsig_pred_ = " << endl << Xsig_pred_ << endl;

  // 3. calculate the predicted sigma points mean and covariance

  x_ = VectorXd(n_x_);
  P_ = MatrixXd(n_x_, n_x_);
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    x_ += weights_(i)* Xsig_pred_.col(i);
  }
  P_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    VectorXd diff = Xsig_pred_.col(i) - x_;
    if (diff(3) > M_PI)
      diff(3) -= 2 * M_PI;
    if (diff(3) < - M_PI)
      diff(3) += 2 * M_PI;
    P_ += weights_(i) * (Xsig_pred_.col(i) - x_) * (Xsig_pred_.col(i) - x_).transpose(); 
  }

  if (debug)
    cout << "for the predicted, x_ = " << endl << x_ << endl;
  if (debug)
    cout << "for the predicted, P_ = " << endl << P_ << endl;

  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  if (debug)
    cout << "end UKF::Prediction() " << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  if (debug)
    cout << "\nbegin UKF::UpdateLidar()" << endl;
  
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  // z is the measurement result
  // z_pred is the predicted result based on x_
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];

  if (debug)
    cout << "meas result for lidar, z = " << endl << z << endl;

  Zsig.fill(0.0);
  for (int i = 0; i <2 * n_aug_ + 1; i ++)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }

  if (debug)
    cout << "Xsig_pred_ transfered to Zsig = " << endl << Zsig << endl;

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }
  if (debug)
    cout << "z_pred = " << endl << z_pred << endl;

  // 2. calculate the covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff* z_diff.transpose();
  }
  if (debug)
    cout << "S = " << endl << S << endl;

  MatrixXd R = MatrixXd(2, 2);
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  if (debug)
    cout << "R = " << endl << R << endl;

  S += R;
  if (debug)
    cout << "S = " << endl << S << endl;

  // 3. calc corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    VectorXd xdiff = Xsig_pred_.col(i) - x_;

    while (xdiff(3) > M_PI) 
      xdiff(3) -= 2*M_PI;
    while (xdiff(3) < - M_PI) 
      xdiff(3) += 2*M_PI;
    
    VectorXd zdiff = z - Zsig.col(i);
    Tc += weights_(i) * xdiff *zdiff.transpose();
  }

  if (debug)
    cout << "TC =" << endl << Tc << endl;

  // 4. calculate kalman gain
  MatrixXd K = Tc * S.inverse();
  VectorXd diff = z - z_pred;
  x_ = x_ + K * diff;

  P_ = P_ - K * S * K.transpose();

  if (debug)
    cout << "the final updated x_ = " << endl << x_ << endl;
  if (debug)
    cout << "the final updated P_ = " << endl << P_ << endl;
  // calc NIS
  double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);

  if (debug)
    cout << "time = " << meas_package.timestamp_ << ", meas = Lidar, NIS = " << nis << endl;

  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  if (debug)
    cout << "end UKF::UpdateLidar()" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  if (debug)
    cout << "\nbegin UKF::UpdateRadar()" << endl;

  if (debug)
    cout << "old x_ = " << endl << x_ << endl;
  if (debug)
    cout << "old P_ = " << endl << P_ << endl;

  // reuse the Xsig_pred_ from the predict sigma points
  // 1. switch from CVRT to radar z space
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);

  // measurement results
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_[0]; // row
  z(1) = meas_package.raw_measurements_[1]; // theta
  z(2) = meas_package.raw_measurements_[2]; // row_dot

  if (debug)
    cout << "the radar measurement z = " << endl << z << endl;

  Zsig.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    double px   = Xsig_pred_(0, i);
    double py   = Xsig_pred_(1, i);
    double v    = Xsig_pred_(2, i);
    double yaw  = Xsig_pred_(3, i);
    double yawd = Xsig_pred_(4, i);
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    if (px*px + py*py < 1.0e-5)
      Zsig(2, i) = 0.0;
    else
      Zsig(2, i) = (px * cos(yaw)* v + py * sin(yaw) * v )/ sqrt(px * px + py * py);
  }

  if (debug)
    cout <<"Xsig_pred_ switched to z space, Zsig = " << endl << Zsig << endl;

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ +1; i ++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  if (debug)
    cout << "z_pred = " << endl << z_pred << endl;

  // 2. calculate the covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2 * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2 * M_PI;
    S += weights_(i) * z_diff* z_diff.transpose();
  }

  MatrixXd R = MatrixXd(3, 3);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  S += R;

  // 3. calc corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i ++)
  {
    VectorXd xdiff = Xsig_pred_.col(i) - x_;
    while (xdiff(3) > M_PI)
      xdiff(3) -= 2*M_PI;
    while (xdiff(3) < - M_PI)
      xdiff(3) += 2*M_PI;
    
    VectorXd zdiff = z - Zsig.col(i);
    while (zdiff(1) > M_PI)
      zdiff(1) -= 2 *M_PI;
    while (zdiff(1) < M_PI)
      zdiff(1) += 2 *M_PI;
    Tc += weights_(i) * xdiff *zdiff.transpose();
  }

  // 4. calculate kalman gain
  MatrixXd K = Tc * S.inverse();
  VectorXd diff = z - z_pred;
  while (diff(1) > M_PI) 
    diff(1) -= 2 *M_PI;
  while (diff(1) < - M_PI)
    diff(1) += 2 * M_PI;
  
  x_ = x_ + K * diff;

  P_ = P_ - K * S * K.transpose();
  if (debug)
    cout << "updated x_ = " << endl << x_ << endl;
  if (debug)
    cout << "udpated P_ = " << endl << P_ << endl;

  double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  if (debug)
    cout << "time = " << meas_package.timestamp_ << ", meas = RADAR, NIS = " << nis << endl;

  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  if (debug)
    cout << "end UKF::UpdateRadar()" << endl;
}
