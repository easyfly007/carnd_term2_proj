#include "PID.h"

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

void PID::UpdateError(double cte) {
	total_error += cte *cte;
	double last_cte = p_error;
	p_error = cte;
	i_error += cte;
	d_error = cte - last_cte;
}

double PID::TotalError() {
	return total_error;
}

double PID::GetSteerValue()
{
	return - p_error * Kp - i_error * Ki - d_error * Kd;
}

