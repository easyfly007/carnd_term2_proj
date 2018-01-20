#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	p_error = 1.0;
	i_error = 1.0;
	d_error = 1.0;
}

PID::~PID() {}

void PID::Init(double Kp2, double Ki2, double Kd2) {
	Kp = Kp2;
	Ki = Ki2;
	Kd = Kd2;
	total_error = 0.0;
}

void PID::UpdateError(double cte, double speed) 
{
	total_error = total_error * 0.9 +  cte *speed;
	double last_cte = p_error;
	p_error = cte;
	i_error = total_error;
	d_error = (cte - last_cte) * speed;
}

double PID::TotalError() {
	return total_error;
}

double PID::GetSteerValue(double speed)
{
	std::cout << "get steering value...total_error = " << total_error << endl;
	std::cout << "   steering from P = =" << - p_error *Kp / speed;
	std::cout << ", from I = " << - i_error * Ki / speed;
	std::cout << ", from D = " << -d_error* Kd << endl;
	return - p_error * Kp / speed - i_error * Ki / speed - d_error * Kd;
}

