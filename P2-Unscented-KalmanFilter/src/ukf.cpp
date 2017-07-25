#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  /*
  [ pos1 
    pos2 
    vel_abs 
    yaw_angle 
    yaw_rate
    ]
  */

  // initial covariance matrix
  P_ = MatrixXd(5, 5); // (x'-x)(x'-x).transpose()
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  /* These will need to be adjusted in order to get your Kalman filter working */
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  /* The measurement noise values should not be changed; these are provided by the sensor manufacturer. */
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // R matrices for measurement noise !!
  R_radar_ = MatrixXd(3,3);
  R_radar_ <<
    std_radr_*std_radr_,                       0,                     0,
                      0, std_radphi_*std_radphi_,                     0,
                      0,                       0, std_radrd_*std_radrd_;
  
  // laser measurement has only position info !!
  R_laser_ = MatrixXd(2,2);
  R_laser_ <<
  std_laspx_*std_laspx_,                     0,
                      0, std_laspy_*std_laspy_;
  


  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // initialize variables defined in ukf.h
  is_initialized_ = false ;

  n_x_ = 5;

  n_aug_ = 7;  // added process noise / motion acceleration noise

  n_sig_ = 2 * n_aug_ + 1 ; 



  //create sigma point matrix, when t = k
  MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);  // 5 * 11


  //create augmented sigma point matrix, when t = k, adding noise into consideration
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1); // 7 * 15


  // 7.20 Predicted sigma points as columns, when t = k+1
  Xsig_pred_ = MatrixXd(n_x_,  2 * n_aug_ + 1); // notice! 5 * 15


  time_us_ = 0.0;



  lambda_ = 3 - n_aug_;




  R_laser_ <<
  std_laspx_ * std_laspx_ ,                         0,
                  0, std_laspy_ * std_laspy_;


  R_radar_ <<    
           std_radr_ * std_radr_,                           0,                        0,
                                0,  std_radphi_ * std_radphi_,                        0,
                                0,                          0,  std_radrd_ * std_radrd_;



  weights_ = VectorXd(n_sig_);

  double weight_0 = lambda_ /(lambda_ + n_aug_ );
  weights_(0) = weight_0;
  for (int i=1; i < n_sig_ ; i++) {  
    double weight = 0.5 /(n_aug_ + lambda_ );
    weights_(i) = weight;
  }




  tools = Tools();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  double Px, Py, Vx, Vy;

  if (!is_initialized_) { // init

    // use first measurement either from Lidar or Radar to initialize
    cout << "UKF: init: " << endl;
    //state vector  x_ = VectorXd(5);
    /*
    [ px,
      py,
      v,
      yaw,
      yawrate
      ]
    */

     double Px;
     double Py; 
     double Vx; 
     double Vy;

    // state covariance matrix P_ = MatrixXd(5, 5);


    //debug
    cout << "UKF init: timestamp=" << measurement_pack.timestamp_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      
      //Convert radar from polar to cartesian coordinates and initialize state.
      

      //meas_package.raw_measurements_ << ro,theta, ro_dot;
      double ro = measurement_pack.raw_measurements_[0];  // distance to pedestrian
      double phi = measurement_pack.raw_measurements_[1];  // bearing angle

      double rho_dot = measurement_pack.raw_measurements_[2]; // range rate



      // init at: ekf_.x_ = [ x, y, vx != 0 , vy != 0] 4 *1
      Px = ro*std::cos(phi);
      Py = ro*std::sin(phi);
      Vx = rho_dot * cos(phi);
      Vy = rho_dot * sin(phi);




    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      
      //Initialize state.

      // init at:  x, y, vx =0, vy = 0


      Px = measurement_pack.raw_measurements_[0]; 
      Py = measurement_pack.raw_measurements_[1];
      Vx = 0;
      Vy = 0;


    }

    // Handle small px, py  ??



    // transfer above linear EKF state vector to nonlinear UKF vector:
    x_ << Px, Py, sqrt(pow(Vx, 2) + pow(Vy, 2)), 0, 0;
    // set init yaw = 0, yawrate = 0 ?

    cout << "init x_: " << x_ << endl;



    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;

  }

  long long incoming = measurement_pack.timestamp_;
  float dt = (incoming - previous_timestamp_) / 1000000.0;



  // SigmaPointPrediction
  ukf.Prediction(dt);
  // generate X(k+1) of 7 * 15 sigma points

  // update
  if (  ) { //lidar

    //ekf_.H_ = H_laser_;
    //ekf_.R_ = R_laser_;

    ukf.UpdateLidar(meas_package);
  }

  if (  ) { // radar

    //ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    //ekf_.R_ = R_radar_;

    ukf.UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  if () {  // lidar

    //predict sigma points
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
      //extract values for better readability
      double p_x      = Xsig_aug(0, i);
      double p_y      = Xsig_aug(1, i);
      double v        = Xsig_aug(2, i);
      double yaw      = Xsig_aug(3, i); // polar angle
      double yawd     = Xsig_aug(4, i); // polar angle rate
      double nu_a     = Xsig_aug(5, i); // v acceleration noise
      double nu_yawdd = Xsig_aug(6, i); // angle rate acceleration noise

      //predicted state values
      double px_p, py_p;

      //avoid division by zero
      if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
      }
      else {
        px_p = p_x + v * delta_t*cos(yaw);
        py_p = p_y + v * delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd * delta_t;
      double yawd_p = yawd;

      //add noise
      px_p = px_p + 0.5 * nu_a * delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5 * nu_a * delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a * delta_t;

      yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd * delta_t;

      //write predicted sigma point into right column
      Xsig_pred(0, i) = px_p;
      Xsig_pred(1, i) = py_p;
      Xsig_pred(2, i) = v_p;
      Xsig_pred(3, i) = yaw_p;
      Xsig_pred(4, i) = yawd_p;
    }

    //create vector for weights
    VectorXd weights = VectorXd(2 * n_aug + 1);

    //create vector for predicted state
    VectorXd x = VectorXd(n_x);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x, n_x);

    // set weights
    double weight_0 = lambda / (lambda + n_aug);

    weights(0) = weight_0;
    for (int i = 1; i < 2 * n_aug + 1; i++) { //2n+1 weights
      double weight = 0.5 / (n_aug + lambda);
      weights(i) = weight;
    }

    //predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
      x = x + weights(i) * Xsig_pred.col(i);
    }

    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x;  // 5 * 1


      //angle normalization
      while (x_diff(3) > M_PI) {
        x_diff(3) -= 2.*M_PI;
      }
      while (x_diff(3) < -M_PI) {
        x_diff(3) += 2.*M_PI;
      }

      MatrixXd tmp = weights(i) * x_diff * x_diff.transpose() ;  // 5 * 5

      std::cout << "tmp" << tmp << std::endl;

      P = P + tmp ; // 5 * 5
    }


  }

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {  // radar
    /*
     Prediction:  7.14 sigma generation at t=k 
                  7.16 Augmentation for t=k
                  7.20 sigma prediction for t=k+1
                  7.23 calc mean of signma prediction and covariance P  at t=k+1 

     */
    /*
      UpdateRadar: 7.26 predict measurement sigma points at t=k+1
                   7.29 with real measure report at t=k+1, update state vector x(k+1) and P(k+1)
    */
    double p_x ;
    double p_y ;
    double v ;
    double yaw ; //polar angle
    double v1 ; //cos(yaw) * v;
    double v2 ; //sin(yaw) * v;


    double yawd ; // polar angle rate
    double nu_a ;  // v acceleration noise
    double nu_yawdd ; // angle rate acceleration noise

    /********************************
     7.14 sigma generation Xsig_ at t=k:
    ********************************/
    MatrixXd A = P_.llt().matrixL();
    //set first column of sigma point matrix
    Xsig_.col(0)  = x_; //x_ << Px, Py, sqrt(pow(Vx, 2) + pow(Vy, 2)), yaw, yawrate;


    //set remaining sigma points
    for (int i = 0; i < n_x_; i++)
    {
      Xsig_.col(i+1)     = x_ + sqrt(lambda + n_x_) * A.col(i);
      Xsig_.col(i+1+n_x_) = x_ - sqrt(lambda + n_x_) * A.col(i);
    }

    /********************************
    7.16 Augmentation for t=k
    ********************************/
    //create augmented mean vector
    VectorXd x_aug_ = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug_ = MatrixXd(7, 7);

    //create augmented mean state
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;  // [x; 0 ; 0]

    //create augmented covariance matrix
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5,5) = P_;
    P_aug_(5,5) = std_a_ * std_a_;
    P_aug_(6,6) = std_yawdd_ * std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug_.llt().matrixL();  // 5 * 5

    //create augmented sigma points
    Xsig_aug_.col(0)  = x_aug_;
    for (int i = 0; i< n_aug_; i++)
    {
      Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda + n_aug_) * L.col(i);
      Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda + n_aug_) * L.col(i);
    }


    /********************************
    7.20 sigma prediction for t=k+1
    ********************************/

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

      //extract values for better readability
      p_x = Xsig_aug_(0,i);
      p_y = Xsig_aug_(1,i);
      v = Xsig_aug_(2,i);
      yaw = Xsig_aug_(3,i);// polar angle
      yawd = Xsig_aug_(4,i); // polar angle rate
      nu_a = Xsig_aug_(5,i);  // v acceleration noise
      nu_yawdd = Xsig_aug_(6,i); // angle rate acceleration noise

      //predicted state values
      double px_p, py_p;

      //avoid division by zero
      if (fabs(yawd) > 0.001) {
          px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
          py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
      }
      else {
          px_p = p_x + v*delta_t*cos(yaw);
          py_p = p_y + v*delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;

      //add noise
      px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a*delta_t;

      yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd * delta_t;

      //write predicted sigma point into right column
      Xsig_pred_(0,i) = px_p;
      Xsig_pred_(1,i) = py_p;
      Xsig_pred_(2,i) = v_p;
      Xsig_pred_(3,i) = yaw_p;
      Xsig_pred_(4,i) = yawd_p;
    }

    /********************************
     7.23 calc mean of signma prediction and covariance P  at t=k+1
    ********************************/

  //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
      x_ = x_ + weights(i) * Xsig_pred_.col(i);
    }

  //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;  // 5 * 1

    //angle normalization
    while (x_diff(3)> M_PI) {
      x_diff(3)-=2.*M_PI;
    }
    while (x_diff(3)<-M_PI) {
      x_diff(3)+=2.*M_PI;
    }

    MatrixXd tmp = weights(i) * x_diff * x_diff.transpose() ;  // 5 * 5

    std::cout << "tmp" << tmp << std::endl;

    P_ = P_ + tmp ; // 5 * 5
    }


  }// radar
}//prediction

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // get z_measurement
  z = meas_package.raw_measurements_  ;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);


  /********************************
   7.26 predict measurement sigma points at t=k+1
  ********************************/

  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

     // extract values for better readibility
     double p_x = Xsig_pred(0,i);
     double p_y = Xsig_pred(1,i);
     double v  = Xsig_pred(2,i);
     double yaw = Xsig_pred(3,i);

     double v1 = cos(yaw)*v;
     double v2 = sin(yaw)*v;

     // measurement model
     Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
     Zsig(1,i) = atan2(p_y,p_x);                                 //phi
     Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
   }

   //mean predicted measurement
   VectorXd z_pred = VectorXd(n_z);

   z_pred.fill(0.0);
   for (int i=0; i < 2*n_aug+1; i++) {
       z_pred = z_pred + weights(i) * Zsig.col(i);
   }

   //measurement covariance matrix S
   MatrixXd S = MatrixXd(n_z,n_z);

   S.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
     //residual
     VectorXd z_diff = Zsig.col(i) - z_pred;

     //angle normalization
     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

     S = S + weights(i) * z_diff * z_diff.transpose(); // 3 * 3
   }

   //add measurement noise covariance matrix
   MatrixXd R = MatrixXd(n_z,n_z);
   R <<    std_radr*std_radr, 0, 0,
           0, std_radphi*std_radphi, 0,
           0, 0,std_radrd*std_radrd;
           
   S = S + R;



  /********************************
   7.29 with real measure report at t=k+1, update state vector x(k+1) and P(k+1)
  ********************************/


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_measdiff = z_meas - z_predmean;

  //angle normalization
  while (z_measdiff(1)> M_PI) z_measdiff(1)-=2.*M_PI;
  while (z_measdiff(1)<-M_PI) z_measdiff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_predmean = x_predmean + K * z_measdiff;
  P = P - K * S * K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // get z_measurement
  z = meas_package.raw_measurements_  ;

  /*
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
    5.9214,   //rho in m
    0.2187,   //phi in rad
    2.0062;   //rho_dot in m/s

  */

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K * S * K.transpose();



}




