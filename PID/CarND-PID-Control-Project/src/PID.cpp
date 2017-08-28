#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  // Constructor
}

PID::~PID() {
  // Destructor
  if (PLOT) {
    outfile.close();
  }
}

void PID::Init(double Kp, double Ki, double Kd, bool dbg, string plotfile) {
  // Initialize with PID gains
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  // initialize errors to zero
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  max_error = 0.0;
  speed = 0.0;

  // Output file
  if (plotfile != "") {
    PLOT = true;
    outfile.open(plotfile);
    outfile << "#";
    outfile << std::setw(11) << "p_error";
    outfile << std::setw(12) << "i_error";
    outfile << std::setw(12) << "d_error";
    outfile << std::setw(12) << "max_error";
    outfile << std::setw(12) << "speed";
    outfile << std::setw(12) << "control" << std::endl;

    // Require two empty datapoints to start off gnuplot w/o warnings
    outfile << std::setw(12) << p_error;
    outfile << std::setw(12) << i_error;
    outfile << std::setw(12) << d_error;
    outfile << std::setw(12) << max_error;
    outfile << std::setw(12) << speed;
    outfile << std::setw(12) << 0.0 << std::endl;

    outfile << std::setw(12) << p_error;
    outfile << std::setw(12) << i_error;
    outfile << std::setw(12) << d_error;
    outfile << std::setw(12) << max_error;
    outfile << std::setw(12) << speed;
    outfile << std::setw(12) << 0.0 << std::endl;
    
    outfile.flush();
  }
  
  else {   
    PID::PLOT = false;
  }

  PID::DEBUG = dbg;

  if (PID::DEBUG) {
    std::cout << "Initializing gains/errors " << std::endl;
    std::cout << "Kp = " << Kp << " p_error = " << p_error << std::endl;
    std::cout << "Ki = " << Ki << " i_error = " << i_error << std::endl;
    std::cout << "Kd = " << Kd << " d_error = " << d_error << std::endl;

    if (PLOT) {
      std::cout << "Writing to file: " << plotfile << std::endl;
    }
  }
}

void PID::UpdateError(double cte) {
  // Integral error
  i_error += cte;  

  // Differential error  
  d_error = cte - p_error;

  // Proportional error
  p_error = cte;

  // Maximum error
  if (std::fabs(cte) > max_error) max_error = std::fabs(cte);

  if (DEBUG) {
    std::cout << " ** Updating errors ** " << std::endl;
    std::cout << " Kp = " << Kp << " p_error = " << p_error;
    std::cout << " Ki = " << Ki << " i_error = " << i_error;
    std::cout << " Kd = " << Kd << " d_error = " << d_error;
    std::cout << " | Max_error = " << max_error << std::endl;
  }
}

double PID::CalculateControl(double offset) {
  // Return control value based on cte
  double output = -Kp*p_error - Ki*i_error - Kd*d_error + offset;
  if (output > 1.0) {
    output = 1.0;
  } else if (output < -1.0) {
    output = -1.0;
  }

  if (DEBUG) {
    std::cout << "Updating control value: " << output << std::endl;
  }

  if (PLOT) {
    outfile << std::setw(12) << p_error;
    outfile << std::setw(12) << i_error;
    outfile << std::setw(12) << d_error;
    outfile << std::setw(12) << max_error;
    outfile << std::setw(12) << speed;
    outfile << std::setw(12) << output << std::endl;
    outfile.flush();
  }

  return output;
}

