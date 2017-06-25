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

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  int est_size = estimations.size();

  if (est_size == 0 || est_size != ground_truth.size())
      return rmse;

  //accumulate squared residuals
  for(int i=0; i < est_size; ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse/est_size;
  rmse = rmse.array().sqrt();
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //if divide by 0, print erro message
    float combo = px*px + py*py;
    if (fabs(combo) < .0001) {
      std::cout << "tried to divide by 0" << std::endl;
      Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
    } else {
      float root_combo = std::sqrt(combo);
      float combo_15 = std::pow(combo, 1.5);

      Hj << px/root_combo, py/root_combo, 0, 0,
            -py/combo, px/combo, 0, 0,
            py*(vx*py - vy*px) / combo_15, px*(vy*px - vx*py) / combo_15, px/root_combo, py/root_combo;
    }
    return Hj;
}
