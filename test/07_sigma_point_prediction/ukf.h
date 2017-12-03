#ifdef __UKF_H__
#define __UKF_H__

#include <vector>
#include "Dense"

class UKF
{
public:
	UKF();
	virtual ~UKF();
	void init();
	void generateSigmaPoints(MatrixXd *Xsig_out);
	void augmentedSigmaPoints(MatrixXd *Xsig_out);
	void sigmaPointPrediction(MatrixXd *Xsig_out);
	void predictMeanAndCovariance(VectorXd *x_pred, MatrixXd *P_pred);
	void predictRadarMeasurement(VectorXd *z_out, MatrixXd* S_out);
	void updateState(VectorXd *x_out, MatrixXd *P_out);
};

#endif
