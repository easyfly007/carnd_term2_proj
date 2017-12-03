#include <iostream>
#include "ukf.h"

UKF::UKF()
{
  // todo auto-generate constructor stub
  init();
}

UKF::~UKF()
{
  return;
}

void UKF::init()
{
  return;
}

void UKF::generateSigmaPoints(MatrixXd *Xsig_out)
{
  int n_x = 5;
  double lambda = 3 - n_x;
  VectorXd x = VectorXd(n_x);
  x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

  MatrixXd P = MatrixXd(n_x, n_x);
  P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0022,
      -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
      0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
      -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
      -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);
  Xsig.fill(0.0);

  MatrixXd A = P.llt().matrixL();

  // assign the mean value
  Xsig.col(0) = x;
  MatrixXd offset = sqrt(lambda + n_x) * A;

  for (int i = 0; i < n_x; i ++)
  {
    Xsig.col(i + 1) = x + offset.col(i);
    Xsig.col(i + 1 + n_x) = x - offset.col(i);
  }

  *Xsig_out = Xsig;

}