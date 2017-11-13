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
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
