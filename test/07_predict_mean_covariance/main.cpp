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
	VectorXd x_pred = VectorXd(5);
	MatrixXd P_pred = MatrixXd(5, 5);
	ukf.predictMeanAndCovariance(&x_pred, &P_pred);
	return 0;
}