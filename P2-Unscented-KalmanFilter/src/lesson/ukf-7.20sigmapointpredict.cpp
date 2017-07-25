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

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;


  //create example sigma point matrix, t = k:  
  // Xsig_aug comes from 7.17 output
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
     Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,

         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);   // careful !! 5 * 15

  double delta_t = 0.1; //time diff in sec
/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //predict sigma points
   for (int i = 0; i< 2*n_aug+1; i++)
   {
     //extract values for better readability
     double p_x = Xsig_aug(0,i);
     double p_y = Xsig_aug(1,i);
     double v = Xsig_aug(2,i);
     double yaw = Xsig_aug(3,i);// polar angle
     double yawd = Xsig_aug(4,i); // polar angle rate
     double nu_a = Xsig_aug(5,i);  // v acceleration noise
     double nu_yawdd = Xsig_aug(6,i); // angle rate acceleration noise

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
     yawd_p = yawd_p + nu_yawdd*delta_t;

     //write predicted sigma point into right column
     Xsig_pred(0,i) = px_p;
     Xsig_pred(1,i) = py_p;
     Xsig_pred(2,i) = v_p;
     Xsig_pred(3,i) = yaw_p;
     Xsig_pred(4,i) = yawd_p;
   }

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;



  /*
5.93553  6.06251  5.92217   5.9415  5.92361  5.93516  5.93705  5.93553  5.80832  5.94481  5.92935  5.94553  5.93589  5.93401  5.93553
1.48939  1.44673  1.66484  1.49719    1.508  1.49001  1.49022  1.48939   1.5308  1.31287  1.48182  1.46967  1.48876  1.48855  1.48939
 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.23954   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049  2.17026   2.2049
0.53678 0.473387 0.678098 0.554557 0.643644 0.543372  0.53678 0.538512 0.600173 0.395462 0.519003 0.429916 0.530188  0.53678 0.535048
 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528 0.387441 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528 0.318159
  */

  //write result
  *Xsig_out = Xsig_pred;

}
