#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

void UKF::Init() {

}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);

  double weight_0 = lambda/(lambda+n_aug);

  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

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
   for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
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

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;
/*
z_pred: 
 6.12155
0.245993
 2.10313

S: 
   0.0946171 -0.000139448   0.00407016
-0.000139448  0.000617548 -0.000770652
  0.00407016 -0.000770652    0.0180917

Zsig: 
 6.11908  6.23346  6.15315  6.12835  6.11436  6.11908  6.12218  6.11908  6.00792  6.08839  6.11255  6.12488  6.11908  6.11886  6.12057
0.244289  0.23371 0.273165 0.246166 0.248461 0.244289 0.245307 0.244289 0.257001 0.216927 0.244336 0.241934 0.244289 0.245157 0.245239
 2.11044  2.21881  2.06391   2.1875  2.03413  2.10616  2.14509  2.10929  2.00166   2.1298  2.03466  2.16518  2.11454  2.07862  2.11295

*/
  //write result
  *z_out = z_pred;
  *S_out = S;
}