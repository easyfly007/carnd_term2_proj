#include <iostream>
#include "ukf.h"

UKF::UKF()
{
  init();
}

UKF::~UKF()
{}

UKF::init()
{}

void UKF::updateState(VectorXd *x_out, MatrixXd *P_out)
{
  int n_x = 5;
  int n_aug = 7;
  int n_z = 3;
  double lambda = 3 - n_aug;
  VectorXd weights = VectorXd(2 * n_aug + 1);
  double weight_0 = lambda / ( lambda + n_aug);
  weights(0) = weight_0;
  for int(i = 1; i < 2 * n_aug + 1; i ++)
  {
    weights(i) = 0.5 / (n_aug + lambda);
  }

  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred << 
           5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  
  // predicted state mean
  VectorXd x = VectorXd(n_x);
  x <<      5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  // predicted state covariance
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<   0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

 // predicted state transformed to z space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig <<
      6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
     0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
      2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
  VectorXd z_pred = VectorXd(n_z);
  z_pred << 
          6.12155,
     0.245993,
      2.10313;

  // matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z, n_z);
  S <<       0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  VectorXd z = VectorXd(n_z);
  z <<       
      5.9214,   //rho in m
      0.2187,   //phi in rad
      2.0062;   //rho_dot in m/s

  // corss correlation
  MatrixXd Tc = MatrixXd(n_x, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    VectorXd xdiff = Xsig_pred.col(i) - x;
    if (xdiff(1) > M_PI) xdiff(1) -= 2*M_PI;
    if (xdiff(1) < - M_PI) xdiff(1) += 2*M_PI;
    
    VectorXd zdiff = Zsig.col(i) - z;
    if (zdiff(1) > M_PI) zdiff(1) -= 2 *M_PI;
    if (zdiff(1) < M_PI) zdiff(1) += 2 *M_PI;
    Tc += weights(i) * zdiff *zdiff.transpose();
  }

  // kalman gain
  MatrixXd K = Tc * S.inverse();
  VectorXd diff = z- z_pred;
  if (diff(1) > M_PI) diff(1) -= 2 *M_PI;
  if (diff(1) < - M_PI) diff(1) += 2 * M_PI;
  x = x + K * diff;

  P = P - K * S * K.transpose();

  *x_out = x;
  *P_out = P;
}