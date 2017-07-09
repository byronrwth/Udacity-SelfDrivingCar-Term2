#include "kalman_filter.h"
#include <iostream>

using namespace std;
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

  // for both Lidar and Radar, use same prediction()

  //debug
  cout << "KalmanFilter, Predict(): old x_" << x_ << endl;

  // x:  [px, py, vx, vy]
  x_ = F_ * x_;

  cout << "KalmanFilter, Predict(): new x_" << x_ << endl;

  cout << "KalmanFilter, Predict(): old P_" << P_ << endl;
  P_ = F_ * P_ * F_.transpose() + Q_;

  cout << "KalmanFilter, Predict(): new P_" << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // for Lidar, y=[ d(px), d(py)]
  VectorXd y = VectorXd(2);

  cout << "KalmanFilter, Update( Laser): z" << z << endl;
  cout << "KalmanFilter, Update( Laser): x_" << x_ << endl;
  cout << "KalmanFilter, Update( Laser): H_" << H_ << endl;
  y = z - H_ * x_;

  MatrixXd PHt = P_ * H_.transpose();

  MatrixXd S = MatrixXd(2, 2);
  S = H_ * PHt + R_;

  MatrixXd K = MatrixXd(2, 2);
  K = PHt * S.inverse();

  cout << "KalmanFilter, update(Laser): old x_" << x_ << endl;
  x_ = x_ + K * y;

  cout << "KalmanFilter, update(Laser): new x_" << x_ << endl;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  cout << "KalmanFilter, update(Laser): old P_" << P_ << endl;
  P_ = (I - K*H_) * P_;
  cout << "KalmanFilter, update(Laser): new P_" << P_ << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // x: [px, py, vx, vy]
  float sqrt_sq = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  // prevent 0 vals as atan2(0,0) undefined
  if (fabs(x_(0)) < 1e-6)
    x_(0) = 1e-6;
  if (fabs(x_(1)) < 1e-6)
    x_(1) = 1e-6;

  VectorXd h(3);
  // use atan2 to keep between -pi and pi
  h << sqrt_sq, atan2(x_(1), x_(0)), (x_(0)*x_(2) + x_(1)*x_(3))/sqrt_sq;

  /*
  h= [  sqrt( px * px, py * py),
        arctan ( py / px),
        (px*vx + py*vy) /  sqrt( px * px, py * py)
  ]

  h = to_polar(x)
  */


  // notice ! not y = z - h * x_;
  VectorXd y = z - h;

  // normalize to keep between -pi and pi
  while (y(1) > 3.1415) {
    y(1) -= 6.2830;  // move [ pi, 2pi] into [-pi, pi]
  }
  while (y(1) < -3.1415) {
    y(1) += 6.2830; //move [ -pi, -2pi] into [-pi, pi]
  }

  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}
