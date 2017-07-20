#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	cout << "tools: CalculateRMSE: estimations size =" << estimations.size() << endl;
	cout << "tools: CalculateRMSE: ground truth size =" << ground_truth.size() << endl;
	
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	int est_size = estimations.size();

	if (est_size == 0 || est_size != ground_truth.size()) {
	  cout << "tools: CalculateRMSE: either empty estimate or not equal, retun rmse =" << rmse << endl;
	  return rmse;
	}
	    

	//accumulate squared residuals
	for(int i=0; i < est_size; ++i){
	  VectorXd residual = estimations[i] - ground_truth[i];
	  residual = residual.array() * residual.array();
	  rmse += residual;
	}

	rmse = rmse/est_size;
	rmse = rmse.array().sqrt();

	cout << "tools: CalculateRMSE: after scan, retun rmse =" << rmse << endl;
	return rmse;
		
}