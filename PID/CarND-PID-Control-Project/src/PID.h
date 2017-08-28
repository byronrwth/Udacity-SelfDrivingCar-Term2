#ifndef PID_H
#define PID_H

#include <string>
#include <fstream>
#include <iomanip>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double max_error;
  double speed;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool DEBUG;
  bool PLOT;
  std::ofstream outfile;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool dbg = false,
	    std::string plotfile = "");

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the control value.
  */
  double CalculateControl(double offset = 0.0);
};

#endif /* PID_H */
