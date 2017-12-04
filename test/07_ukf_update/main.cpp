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
	VectorXd x_out = VectorXd(5);
	MatrixXd P_out = MatrixXd(5, 5);
	ukf.updateState(&x_out, &P_out);
	return 0;
}