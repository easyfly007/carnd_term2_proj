#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool debug = false;
Tools tool;

// for pi value visit M_PI
#define _USE_MATH_DEFINES

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  if (debug)
    cout << "begin UKF::UKF()" << endl;

  // if this is false, laser measurements will be ignored (except during init)  
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;
  n_aug_ = 7;
  n_aug_sigcnt_ = 2 * n_aug_ + 1;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;
  std_a_ = 0.2; // suppose the a_max is 2 m /s^2, then use std_a = 0.5 * a_max
  std_a_ = 1.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  std_yawdd_ = 0.2; // shall we change it to smaller?
  std_yawdd_ = 0.1;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  //std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  R_laser_ = MatrixXd(2, 2);
  R_laser_.fill(0.0);
  R_laser_(0, 0) = std_laspx_ * std_laspx_;
  R_laser_(1, 1) = std_laspy_ * std_laspy_;
  
  R_radar_ = MatrixXd(3, 3);
  R_radar_.fill(0.0);
  R_radar_(0, 0) = std_radr_ * std_radr_;
  R_radar_(1, 1) = std_radphi_ * std_radphi_;
  R_radar_(2, 2) = std_radrd_ * std_radrd_;

  is_initialized_ = false;
  
  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigcnt_);

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(n_aug_sigcnt_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < n_aug_sigcnt_; i ++)
    weights_(i) = 0.5 / (lambda_ + n_aug_);

  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigcnt_);

  x_aug_ = VectorXd(n_aug_);

  time_us_ = 0.0;
  
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (debug)
  {
    cout << "\n\nbegin UKF::ProcessMeasurement()";
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
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
    if (debug)
      cout << "begin initialization" << endl;

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  
    double px, py, v, yaw, yawd;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      double rho = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);
      tool.normalAngle(&theta);

      // change from radar space to CTRV space
      px = rho * cos(theta);
      py = rho * sin(theta);
      v  = rho_dot;
      yaw = theta;
      yawd = 0.0;
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 10, 0,
            0, 0, 0, 0, 10;
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
            0, 0, 10, 0, 0,
            0, 0, 0, 10, 0,
            0, 0, 0, 0, 10;
    }
    x_(0) = px;
    x_(1) = py;
    x_(2) = v;
    x_(3) = yaw;
    x_(4) = yawd;
    
    if (debug)
    {
      cout << "the initial value for x_ = " << endl << x_ << endl;
      cout << "the initial value for P_ = " << endl << P_ << endl;
    }

    time_us_ = meas_package.timestamp_;
    return;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
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
  {
    cout << "\nbegin UKF::Prediction(), delta_t = " << delta_t << endl;
   if (use_laser_ )
      cout << "laser ..." << endl;
    else
      cout << "radar ..." << endl;
    cout << "the old x_ = " << endl << x_ << endl;
    cout << "the old P_ = " << endl << P_ << endl;
  }
    
  // 1. build augmented sigma popints
  x_aug_.fill(0.0);
  x_aug_.head(n_x_) = x_;
  
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_, n_x_) = std_a_ * std_a_;
  P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug_.llt().matrixL();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_aug_sigcnt_);
  Xsig_aug.fill(0.0);

  Xsig_aug.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i ++)
  {
    Xsig_aug.col(i + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  if (debug)
  {
    cout << "build Xsig_aug = " << endl << Xsig_aug << endl;
    cout << "build x_aug_ = " << endl << x_aug_ << endl;
  }

  // 2. predict the new state by the augmented sigma time points, through delta_t  
  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigcnt_);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
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
    double yawd_p = yawd;

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
  
  x_ = Xsig_pred_ * weights_;

  P_.fill(0.0);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    tool.normalAngle(&x_diff(3));
    P_ += weights_(i) * x_diff * x_diff.transpose();
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
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigcnt_);
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  // z is the measurement result
  // z_pred is the predicted result based on x_
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];

  if (debug)
    cout << "meas result for lidar, z = " << endl << z << endl;

  Zsig = Xsig_pred_.block(0, 0, n_z, n_aug_sigcnt_);

  if (debug)
    cout << "Xsig_pred_ transfered to Zsig = " << endl << Zsig << endl;

  z_pred.fill(0.0);
  z_pred = Zsig * weights_;

  if (debug)
    cout << "z_pred = " << endl << z_pred << endl;

  // 2. calculate the covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff* z_diff.transpose();
  }
  if (debug)
    cout << "S = " << endl << S << endl;

  S += R_laser_;
  if (debug)
    cout << "S = " << endl << S << endl;

  // 3. calc corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    tool.normalAngle(&x_diff(3));

    VectorXd zdiff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * x_diff *zdiff.transpose();
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

  // if (debug)
    cout << "time = " << meas_package.timestamp_ << ", meas = Lidar, NIS = " << nis << endl;

  if (debug)
    cout << "end UKF::UpdateLidar()" << endl;
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  if (debug)
  {
    cout << "\nbegin UKF::UpdateRadar()" << endl;
    cout << "old x_ = " << endl << x_ << endl;
    cout << "old P_ = " << endl << P_ << endl;
  }

  // reuse the Xsig_pred_ from the predict sigma points
  // 1. switch from CVRT to radar z space
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigcnt_);
  VectorXd z_pred = VectorXd(n_z);

  // measurement results
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_[0]; // row
  z(1) = meas_package.raw_measurements_[1]; // theta
  z(2) = meas_package.raw_measurements_[2]; // row_dot

  if (debug)
    cout << "the radar measurement z = " << endl << z << endl;

  Zsig.fill(0.0);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {
    double px   = Xsig_pred_(0, i);
    double py   = Xsig_pred_(1, i);
    double v    = Xsig_pred_(2, i);
    double yaw  = Xsig_pred_(3, i);

    double v1 = v * cos(yaw);
    double v2 = v * sin(yaw);

    Zsig(0, i) = sqrt(px * px + py * py);
    if (sqrt(px * px + py * py) < 0.001)
      Zsig(1, i) = 0.0;
    else
      Zsig(1, i) = atan2(py, px);
  
    if (sqrt(px * px + py * py) < 0.001)
      Zsig(2, i) = 0.0;
    else
      Zsig(2, i) = (px * v1 + py * v2) / sqrt(px * px + py * py);
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
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {

    VectorXd z_diff = Zsig.col(i) - z_pred;
    tool.normalAngle(&z_diff(1));
    S += weights_(i) * z_diff* z_diff.transpose();
  }

  S += R_radar_;

  // 3. calc corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_aug_sigcnt_; i ++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    tool.normalAngle(&x_diff(3));
    
    VectorXd z_diff = Zsig.col(i) - z_pred;
    tool.normalAngle(&z_diff(1));
    Tc += weights_(i) * x_diff *z_diff.transpose();
  }

  // 4. calculate kalman gain
  MatrixXd K = Tc * S.inverse();
  VectorXd diff = z - z_pred;
  tool.normalAngle(&diff(1));
  
  x_ = x_ + K * diff;

  P_ = P_ - K * S * K.transpose();
  if (debug)
  {
    cout << "updated x_ = " << endl << x_ << endl;
    cout << "udpated P_ = " << endl << P_ << endl;
  }

  double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  // if (debug)
    cout << "time = " << meas_package.timestamp_ << ", meas = RADAR, NIS = " << nis << endl;

  if (debug)
    cout << "end UKF::UpdateRadar()" << endl;
}
