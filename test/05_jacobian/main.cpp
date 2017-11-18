#include <iostream>
#include <vecotr>
#include "Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd calculateJacobian(const VectorXd &x_state);

int main()
{
	/*
	compute the jacobian matrix 
	*/

	// predicted state example
	// px = 1, py = 2, vx = 0.1, vy = 0.4
	VectorXd x_predict(4);
	x_predict << 1, 2, 0.2, 0.4;

	MatrixXd Hj = calculateJacobian(x_predict);
	cout << "Hj: " << endl << Hj << endl;
	return 0;
}

MatrixXd calculateJacobian(const VectorXd &x_state)
{
	MatrixXd Hj(3, 4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	double r2 = px*px + py * py;
	if (r2 < 1.0e-15)
		return Hj;

	double r1 = sqrt(r2);
	double r3 = r1 * r2;
	Hj(0, 0) = px/r1;
	Hj(0, 1) = py/r1;
	Hj(0, 2) = 0.0;
	Hj(0, 3) = 0.0;
	Hj(1, 0) = - py / r2;
	Hj(1, 1) = px/r2;
	Hj(1, 2) = 0.0;
	Hj(1, 3) = 0.0;
	Hj(2, 0) = Py *(vx*py - vy * px) / r3;
	Hj(2, 1) = Px *(vy*px - vx * py) / r3;
	Hj(2, 2) = px/r1;
	Hj(2, 3) = py/r1;

	return Hj;
}