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
	VectorXd z_out = VectorXd(3);
	MatrixXd S_out = MatrixXd(3,3);
	ukf.predictRadarMeasurement(&z_out, &S_out);
	return 0;
}