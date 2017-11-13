#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
    assert(estimations.size() >0);
    assert(estimations.size() == ground_truth.size())l

    int size = estimations.size()
	VectorXd rmse(size);
	for (int i = 0; i < size; i ++)
		rmse(i) = 0.0;

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        for (int j = 0; j < 4; j++)
    	    rmse(i) += estimations[i](j) - ground_truth[i](j)	
	}
	
	for (int j = 0; j < 4; j ++)
	    rmse(j) = rmse(j) / size;

	//calculate the mean
	for (int j = 0; j < 4; j ++)
	    rmse(j) = sqrt(rmse(j));

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px == 0.0 && py == 0.0)
	    return Hj;
	    
	//compute the Jacobian matrix
	double r2 = px*px + py*py;
	double r1 = sqrt(r2);
	Hj(0, 0) = px / r1;
	Hj(0, 1) = py / r1;
	Hj(0, 2) = 0.0;
	Hj(0, 3) = 0.0;
	
	Hj(1, 0) = - py /r2;
	Hj(1, 1) = px / r2;
	Hj(1, 2) = 0.0;
	Hj(1, 3) = 0.0;
	
	Hj(2, 0) = py*(vx*py - vy*px) / (r1*r1*r1);
	Hj(2, 1) = px*(vy*px - vx*py) / (r1*r1*r1);
	Hj(2, 2) = px/r1;
	Hj(2, 3) = py/r1;

	return Hj;
}
