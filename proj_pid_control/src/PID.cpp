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
	i_error = i_error + (cte + last_cte) / 2.0;
	d_error = (cte - last_cte);
}

double PID::TotalError() {
	return total_error;
}

double PID::GetSteerValue(double speed)
{
	static double cte = 0.0;

	double p_steer = - p_error * Kp ;
	double i_steer = - i_error * Ki ;// / speed;
	if (p_error > 1.0 && p_error > cte + 0.1)
		p_steer = p_steer + p_steer * (1 + (p_error - cte) * 10);
	if (p_error < -1.0 && p_error < cte - 0.1)
		p_steer = p_steer + p_steer * (1 + (cte - p_error) * 10);


	double d_steer = - d_error * Kd;
	cout << "p_error=" << p_error << ", i_error=" << i_error << ", d_error=" << d_error << endl;
	cout << "p_steer=" << p_steer << ", i_steer=" << i_steer << ", d_steer=" << d_steer << endl;
	

	cte = p_error;
	return p_steer + i_steer + d_steer;
}

double PID::GetThrottleValue(double speed)
{
	// the last cte
	static double cte = 0.0;
	double throttle = 0.1;

	if (p_error > 1.0 && p_error > cte + 0.05)
		throttle = -0.1;
	else if (p_error > 1.2 && p_error > cte + 0.002)
		throttle = -0.1;
	else if (p_error < -1.0 && p_error < cte - 0.05)
		throttle = -0.1;
	else if (p_error < -1.2 && p_error < cte - 0.002)
		throttle = -0.1;

	if (speed < 10)
		throttle = 0.3;
	cte = p_error;

	return throttle;
}

