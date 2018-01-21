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

void PID::UpdateError(double cte, double speed) 
{
	double last_cte = p_error;
	p_error = cte;
	i_error = i_error * 0.9 + (cte + last_cte) * speed / 2.0;
	d_error = (cte - last_cte) * speed;
}

double PID::TotalError() {
	return total_error;
}

double PID::GetSteerValue(double speed)
{
	double p_steer = - p_error * Kp / speed;
	double i_steer = - i_error * Ki / speed;
	double d_steer = - d_error * Kd;
	cout << "p_error=" << p_error << ", p_steer =" << p_steer << endl;
	cout << "i_error=" << i_error << ", i_steer =" << i_steer << endl;
	cout << "d_error=" << d_error << ", d_steer =" << d_steer << endl;
	return p_steer + i_steer + d_steer;
}

double PID::GetThrottleValue(double speed)
{
	double throttle = 0.0;
	if (p_error < 2.0 && (d_error > -1.0 && d_error < 1.0))
		throttle = 0.3;
	else
		throttle = -0.1;

	if (speed < 15.0 && throttle < 0.0)
		throttle = 0.1;

	return throttle;
}

