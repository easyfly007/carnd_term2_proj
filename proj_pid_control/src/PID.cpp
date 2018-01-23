#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp2, double Ki2, double Kd2) {
	Kp = Kp2;
	Ki = Ki2;
	Kd = Kd2;
}

void PID::UpdateError(double cte)
{
	double last_cte = p_error;
	p_error = cte;
	i_error = i_error + cte;
	d_error = (cte - last_cte);
}

double PID::TotalError() {
	return total_error;
}

double PID::GetSteerValue()
{
	double p_steer = - p_error * Kp;
	double i_steer = - i_error * Ki;
	double d_steer = - d_error * Kd;

	cout << "p_error=" << p_error << ", i_error=" << i_error << ", d_error=" << d_error << endl;
	cout << "p_steer=" << p_steer << ", i_steer=" << i_steer << ", d_steer=" << d_steer << endl;
	return p_steer + i_steer + d_steer;
}