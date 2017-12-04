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

void UKF::predictMeanAndCovariance(VectorXd *x_out, MatrixXd *P_out)
{
  int n_x = 5;
  int n_aug = 7;
  double lambda = 3 - n_aug;
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
            5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  
  VectorXd weights = VectorXd(2 * n_aug + 1);
  VectorXd x = VectorXd(n_x);
  MatrixXd P = MatrixXd(n_x, n_x);

  // set up weights
  weights(0) = lambda / (lambda + n_aug);
  for (int i = 1; i < 2 *n_aug + 1; i ++)
    weights(i) = 1 / (2 * (lambda + n_aug));

  x.fill(0.0);
  for (int i = 0; i < 2 *n_aug + 1; i ++)
  {
    x += weights(i)* Xsig_pred.col(i);
  }

  // predict covariance
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    P += weights(i) * (Xsig_pred.col(i) - x) * (Xsig_pred.col(i) - x).transpose(); 
  }

  *x_out = x;
  *P_out = P;


}