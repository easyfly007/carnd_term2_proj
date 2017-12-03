#include "ukf.h"
#include <iostream>
using namespace std;

UKF::UKF()
{
  init();
}

UKF::~UKF()
{}

void UKF::init()
{}

void UKF::augmentedSigmaPoints(MatrixXd *Xsig_out)
{
  int n_x = 5;
  int n_aut = 7;
  double std_a = 0.2;
  double std_yawdd = 0.2;
  double lambda = 3 - n_aug;

  VectorXd x = VectorXd(n_x);
  x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

  MatrixXd P = MatrixXd(n_x, n_x);
  P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0022,
    -0.0013,  0.0077,  0.0011,  0.0071,  0.0060,
     0.0030,  0.0011,  0.0054,  0.0007,  0.0008,
    -0.0022,  0.0071,  0.0007,  0.0098,  0.0100,
    -0.0020,  0.0060,  0.0008,  0.0100,  0.0123;

  // the augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);
  x_aug.fill(0.0);
  x_aug.head(n_x) = x;

  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug.fill(0.0);

  MatrixXd P_aug = MatrixXd(n_aug, n_aug);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug(n_x, n_x) = std_a * std_a;
  P_aug(n_x + 1, n_x + 1) = std_yawdd * std_yawdd;

  MatrixXd offset = sqrt(lambda + n_aug) * (P_aug.llt.matrixL());

  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug; i ++)
  {
    Xsig_aug.col(i + 1) = x_aug + offset.col(i);
    Xsig_aug.col(i + 1 + n_aug) = x_aug - offset.col(i);
  }

  *Xsig_out = Xsig_aug;
}