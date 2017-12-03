#include <iostream>
#include "Dense"
#include <vector>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main()
{
	UKF ukf;

	MatrixXd Xsig = MatrixXd(5, 11);
	ukf.generateSigmaPoints(&Xsig);
	std::cout << "Xsig = " << std::endl << std::endl;
	return 0;
}