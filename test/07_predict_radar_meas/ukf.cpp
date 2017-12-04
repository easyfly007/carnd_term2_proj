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

void UKF::predictRadarMeasurement(VectorXd *z_out, MatrixXd *S_out)
{
  int n_x = 5;
  int n_aug = 7;
  int n_z = 3;
  double lambda = 3 - n_aug;
  VectorXd weights = VectorXd(2 * n_aug + 1);
  double weight_0 = lambda / (lambda + n_aug);
  weights(0) = weight_0;
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    double weight = 0.5 / (n_aug + lambda);
    weights(0) = weight;
  }

  double std_radar = 0.3;
  double std_radphi = 0.0175;
  double std_radrd = 0.1;

  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
           5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);

  Zsig.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    double px   = Xsig_pred(0, i);
    double py   = Xsig_pred(1, i);
    double v    = Xsig_pred(2, i);
    double yaw  = Xsig_pred(3, i);
    double yawd = Xsig_pred(4, i);
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    if (px*px + py*py < 1.0e-5)
      Zsig(2, i) = 0.0;
    else
      Zsig(2, i) = (px * cos(yaw)* v + py * sin(yaw) * v )/ sqrt(px * px + py * py);
  }

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug +1; i ++)
  {
    z_pred += weights(i) * Zsig.col(i);
  }

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i ++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (z_diff(1) > M_PI)
      z_diff(1) -= 2 * M_PI;
    if (z_diff(1) < -M_PI)
      z_diff(1) += 2 * M_PI;
    S += weights(i) * z_diff* z_diff.transpose();
  }
  MatrixXd R = MatrixXd(3, 3);
  R.fill(0.0);
  R(0, 0) = std_radr * std_radr;
  R(1, 1) = std_radphi * std_radphi;
  R(2 ,2) = std_radrd * std_radrd;
  S += R;
  
  *S_out = S;
  *z_out = z_pred;
}