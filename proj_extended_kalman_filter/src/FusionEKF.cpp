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

  // Hj_ cannot be set an given value, as the x_ is not given yet
  // Hj_ << ;
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
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      Eigen::VectorXd meas_polar = meas_package.raw_measurements_;
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

      ekf_.x_ = x_in;

      // Eigen::MatrixXd P_in = MatrixXd(4, 4);
      // P_in << 1,0, 0, 0, 0,
      //       0, 1, 0, 0,
      //       0, 0, 1000, 0,
      //       0, 0, 0, 1000;
      // Eigen::MatrixXd F_in = MatrixXd() 
      // R_in = R_radar_;

      // ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      Eigen::VectorXd x_in = meas_package.raw_measurements_;

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
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
