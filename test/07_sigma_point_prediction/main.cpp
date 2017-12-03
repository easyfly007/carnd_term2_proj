#include <iostream>
#include <vector>
#include "Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main()
{
	UKF ukf;
	MatrixXd Xsig_pred = MatrixXd(5, 15);
	ukf.sigmaPointPrediction(&Xsig_pred);
	return 0;
}