#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

void UKF::Init() {

}


/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
     Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; //time diff in sec
  double dt2 = delta_t * delta_t;
/*******************************************************************************
 * Student part begin
 ******************************************************************************/
    for (int i = 0; i < 15; i ++)
  {
      VectorXd onePoint = VectorXd(5);
      double px   = Xsig_aug(0, i);
      double py   = Xsig_aug(1, i);
      double v    = Xsig_aug(2, i);
      double yaw  = Xsig_aug(3, i);
      double yawd = Xsig_aug(4, i);
      double va   = Xsig_aug(5, i);
      double yawdd= Xsig_aug(6, i);
      if (abs(yawd) > 1.0e-10)
      {
          Xsig_pred(0, i) = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw)) + 1/2 * cos(yaw) * va * delta_t * delta_t;
          Xsig_pred(1, i) = py + v / yawd * (- cos(yaw + yawd*delta_t) + cos(yaw)) + 1/2 * sin(yaw) * va * delta_t * delta_t;
          Xsig_pred(2, i) = v + 0 + va * delta_t;
          Xsig_pred(3, i) = yaw + yawd * delta_t + 1/2 * yawdd * delta_t * delta_t;
          Xsig_pred(4, i) = yawd + 0 + yawdd * delta_t;
      }
      else
      {
          Xsig_pred(0, i) = px + v * cos(yaw) * delta_t + 1 /2 * delta_t * delta_t * cos(yaw) * va;
          Xsig_pred(1, i) = py + v * sin(yaw) * delta_t + 1 /2 * delta_t * delta_t * sin(yaw) * va;
          Xsig_pred(2, i) = v + 0 + delta_t * va;
          Xsig_pred(3, i) = yaw + yawd * delta_t + 1/2 * delta_t * delta_t * yawdd;
          Xsig_pred(4, i) = yawd + 0 + delta_t * yawdd;
      }
      

   
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}
