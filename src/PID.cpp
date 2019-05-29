#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  //Change these values later
  Kp=1;
  Ki=1;
  Kd=1;
  
  //start with zero error
  i_error = 0;
  d_error = 0;

}

void PID::UpdateError(double cte) {

  //UPDATE PID values
  Kp = cte*Kp; //proportional constant... what should the constant be?
  
  d_error = cte - d_error; //constant equal to the change in error
  Kd = Kd*d_error;
  
  i_error += cte; //integral error keeps track of total error
  Ki = i_error * Ki;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}