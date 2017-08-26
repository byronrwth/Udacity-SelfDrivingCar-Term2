#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    this->Kp = Kp; // param[0]
    this->Kd = Kd; // param[1]
    this->Ki = Ki; // param[2]

    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;

    this->total_square_error = 0.0;
    this->average_error = 0.0;
    this->n = 1;
}

void PID::UpdateError(double cte) {
    this->total_square_error += cte * cte;
    
    this->average_error = total_square_error / n;
    this->n++;

    // cte = robot.y ; diff_cte = cte - prev_cte
    this->d_error = cte - this->p_error;

    //  save for next time as prev_cte
    this->p_error = cte;

    
    //  int_cte
    this->i_error += cte;

}

double PID::TotalError() {
    //steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte ;
    //steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte ;
    double steer = 1. * (this->Kp * this->p_error + this->Kd * this->d_error + this->Ki * this->i_error) ;
    return steer;

}

