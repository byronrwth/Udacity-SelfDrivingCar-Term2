#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations for lidar,
    * use linear, basic Kalman filter equations:
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;




  VectorXd y = VectorXd(2);
  y = z - H_ * x_;

  MatrixXd PHt = P_ * H_.transpose();

  MatrixXd S = MatrixXd(2, 2);
  S = H_ * PHt + R_;

  MatrixXd K = MatrixXd(2, 2);
  K = PHt * S.inverse();

  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations for radar,
    * use non-linear equations, involves linearizing the equations with the Jacobian matrix
  */

  float sqrt_sq = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  // prevent 0 vals as atan2(0,0) undefined
  if (fabs(x_(0)) < 1e-6)
    x_(0) = 1e-6;
  if (fabs(x_(1)) < 1e-6)
    x_(1) = 1e-6;

  VectorXd h(3);
  // use atan2 to keep between -pi and pi
  h << sqrt_sq, atan2(x_(1), x_(0)), (x_(0)*x_(2) + x_(1)*x_(3))/sqrt_sq;

  VectorXd y = z - h;

  // normalize to keep between -pi and pi
  while (y(1) > 3.1415) {
    y(1) -= 6.2830;
  }
  while (y(1) < -3.1415) {
    y(1) += 6.2830;
  }

  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}
