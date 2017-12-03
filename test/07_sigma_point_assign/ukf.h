#ifdef __UKF_H__
#define __UKF_H__

#include <vector>
#include "Ddense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
public:
	UKF();
	virtual ~UKF();

	void init();

	void generateSigmaPoints(MatrixXd *Xsig_out);
	void augmentedSigmaPoints(MatrixXd *Xsig_out);
	void predictSigmaPoints(VectorXd *z_pred, MatrixXd *p_pred);
	void predictMeanAndCovariance(VectorXd *z_out, MatrixXd *s_out);
	void updateState(VectorXd *x_out, MatrixXd *P_out);
};
#endif
