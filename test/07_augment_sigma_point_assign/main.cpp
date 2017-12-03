#include <iostream>
#include <vector>
#include "Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main()
{
	UKF ukf;
	MatrixXd Xsig_aug = MatrixXd(7, 15);
	ukf.augmentedSigmaPoints(&Xsig_aug);
	return 0;
}