#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // 10 times
  //R_laser_ << 0.225, 0,
  //      0, 0.225;

  // test with 100 times error
  
  /*
  R_laser_ << 2.25, 0,
        0, 2.25; */


  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  
  // 10 times
  /*
  R_radar_ << 0.9, 0, 0,
        0, 0.009, 0,
        0, 0, 0.9; */

  // test with 100 times error
  /*
  R_radar_ << 9, 0, 0,
      0, 0.09, 0,
        0, 0, 9; */

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //keep px and py only for liday
   H_laser_ << 1, 0, 0, 0,
               0, 1, 0, 0;

   MatrixXd F_ = MatrixXd(4, 4);
   F_ << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

   MatrixXd Q_ = MatrixXd(4, 4);

   MatrixXd P_ = MatrixXd(4, 4);
   P_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

   VectorXd x_ = VectorXd(4);
   x_ << 1, 1, 1, 1;

   // initialize variables in Kalman Filter
   ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

   tools = Tools();

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //debug
    cout << "FusionEKF: timestamp=" << measurement_pack.timestamp_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      //Convert radar from polar to cartesian coordinates and initialize state.
      

      //meas_package.raw_measurements_ << ro,theta, ro_dot;
      double ro = measurement_pack.raw_measurements_[0];  // distance to pedestrian
      double phi = measurement_pack.raw_measurements_[1];  // bearing angle

      double rho_dot = measurement_pack.raw_measurements_[2]; // range rate

      // init at: ekf_.x_ = [ x, y, vx =0, vy = 0] 4 *1
      ekf_.x_ << ro*std::cos(phi), ro*std::sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      //Initialize state.
      


      // init at:  x, y, vx =0, vy = 0
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }


    //debug
    cout << "FusionEKF: init with ekf_.x_=" << ekf_.x_ << endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //use time difference to calculate F state transtion and Q error from acceleration
  
  // only calc dt for lidar
  //if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
  //if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  long long incoming = measurement_pack.timestamp_;
  float dt = (incoming - previous_timestamp_) / 1000000.0;

  //debug
  cout << "FusionEKF: dt=" << dt << endl;

  previous_timestamp_ = incoming;

  ekf_.F_(0, 2) = dt;

  ekf_.F_(1, 3) = dt;
  /*
     F_ <<  1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  */
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  ekf_.Q_ <<  dt_4 / 4 * noise_ax_,                       0,  dt_3 / 2 * noise_ax_,                     0,
                                  0,   dt_4 / 4 * noise_ay_,                      0, dt_3 / 2 * noise_ay_,
              dt_3 / 2 * noise_ax_,                       0,      dt_2 * noise_ax_,                     0,
                                  0,    dt_3 /2 * noise_ay_,                      0,      dt_2 * noise_ay_;
  

    //cout << "FusionEKF: now only predict for radar " << endl;
    //cout << "FusionEKF: now only predict for laser " << endl;
    ekf_.Predict();
  //}
  

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // calc jacobian to map to linear
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    cout << "FusionEKF: UpdateEKF( Radar ) with raw_measurements_: " << measurement_pack.raw_measurements_ << endl;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_); //ro, theta, ro_dot;

  }else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    cout << "FusionEKF: Update( Laser ) with raw_measurements_: " << measurement_pack.raw_measurements_ << endl;

    ekf_.Update(measurement_pack.raw_measurements_);  // px, py
  } 

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
