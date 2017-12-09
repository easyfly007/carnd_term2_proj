#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	cout << "begin Tools::calculateRMSE" << endl;
	size_t dataCnt = estimations.size();
	assert(estimations.size() == ground_truth.size());
	assert(estimations.size() > 0);
	size_t dataDim = estimations[0].size();
	assert(dataCnt > 0);
	assert(estimations[0].size() == ground_truth[0].size());
	cout <<"in calculateRMSE:: dataCnt = " << dataCnt << ", dataDim = " << dataDim << endl;
	VectorXd rmse(dataDim);
	rmse.fill(0.0);
	
	for (int i = 0; i < dataCnt; i ++)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	rmse = rmse/dataCnt;

	//calculate the squared root
	rmse = rmse.array().sqrt();
	cout << "rmse = " << rmse << endl;
	cout << "end tools::calculateRMSE()" << endl;

	return rmse;
}