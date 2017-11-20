#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4); // measurement matrix
  Hj_ = MatrixXd(3, 4);  // measurement matrix, jacobian

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    // ekf_.x_ = VectorXd(4);
    // ekf_.x_ << 1, 1, 1, 1;
    
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      Eigen::VectorXd meas_polar = measurement_pack.raw_measurements_;
      double rho = meas_polar(0);
      double theta = meas_polar(1);
      double rho_dot = meas_polar(2);
      double px = rho * cos(theta);
      double py = rho * sin(theta);
      double vx = rho_dot * cos(theta);
      double vy = rho_dot * sin(theta);
      
      // state
      Eigen::VectorXd x_in = VectorXd(4);
      x_in << px, py, vx, vy;

      Eigen::MatrixXd P_in = MatrixXd(4, 4);
      P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

      Eigen::MatrixXd F_in = MatrixXd(4, 4);
      F_in <<  0, 0, 1, 0,
      			0, 0, 0, 1,
      			0, 0, 0, 0,
      			0, 0, 0, 0;
      Eigen::MatrixXd H_in = MatrixXd(2, 4);
      H_in << 1, 0, 0, 0,
      			0, 1, 0, 0;
      
      Eigen::MatrixXd R_in = R_radar_;

      // ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      Eigen::VectorXd x_in = measurement_pack.raw_measurements_;
      
      //state covariance matrix P
      Eigen::MatrixXd P_in = MatrixXd(4, 4);
      P_in << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

      // prediction matrix F
      Eigen::MatrixXd F_in = MatrixXd(4, 4);
      F_in << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

      // measurement transforming matrix H
      Eigen::MatrixXd H_in = MatrixXd(2, 4);
      H_in << 1, 0, 0, 0,
        0, 1, 0, 0;

      // measurement noise covarience
      Eigen::MatrixXd R_in = R_laser_;

      // predict covariance
      // note that predict covariance depends on the dt, in the initial step
      // we juest set all to 0
      Eigen::MatrixXd Q_in = MatrixXd(4, 4);
      Q_in << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

      ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // modify the F matrix to consider time step
  ekf_.F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
  // ekf_.F_(0, 2) = dt;
  // ekf_.F_(1, 3) = dt;

  // set process covariance matrix Q, based on the noise_ax, noise_ay and dt
  ekf_.Q_ = MatrixXd(4, 4);
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  ekf_.Q_ << dt4 /4.0 * noise_ax, dt3 * noise_ax / 2, 0, 0,
    0, dt4 / 4.0 * noise_ay, 0, dt3 * noise_ay / 2.0,
    dt3 * noise_ax / 2, 0, dt2 * noise_ax, 0,
    0, dt3 * noise_ay/ 2, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // rebuild Hj_, based on the current state
    Tools tool;
    Hj_ = tool.CalculateJacobian(ekf_.x_);
    /*
    double px = ekf_.x_(0);
    double py = ekf_.x_(1);
    double vx = ekf_.x_(2);
    double vy = ekf_.x_(3);
    double r1 = sqrt(px*px + py*py);
    double r2 = r1 * r1;
    double r3 = r1 * r2;
    Hj_(0, 0) = px / r1;
    Hj_(0, 1) = py / r1;
    Hj_(0, 2) = 0.0;
    Hj_(0, 3) = 0.0;
    Hj_(1, 0) = - py / r2;
    Hj_(1, 1) = - px / r2;
    Hj_(1, 2) = - 0.0;
    Hj_(1, 3) = - 0.0;
    Hj_(2, 0) = py*(vx*py - vy*px)/ r3;
    Hj_(2, 1) = px*(vy*px - vx*py)/ r3;
    Hj_(2, 2) = px / r1;
    Hj_(2, 3) = py / r1;
    */
  
    ekf_.R_ = R_radar_; // size (3, 3)
    ekf_.H_ = Hj_; // size (3, 4)

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // need to update H, the meas matix, and R, the measurement covariance 
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
