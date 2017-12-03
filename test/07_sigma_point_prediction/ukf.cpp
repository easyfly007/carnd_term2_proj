#include <iostream>
#include "ukf.h"

UKF::UKF()
{
  init();
}

UKF::~UKF()
{}

void UKF::init()
{}

void UKF::sigmaPointPrediction(MatrixXd *Xsig_out)
{
  int n_x = 5;
  int n_aug = 7;

  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug << 
  5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;
  
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  double delta_t = 0.1;

  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    vectorXd onePoint = VectorXd(n_x);
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
  *Xsig_out = Xsig_pred;
}