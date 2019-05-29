#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp=Kp_;
  Ki=Ki_;
  Kd=Kd_;
  
  //start with zero error
  p_error = 0;
  i_error = 0;
  d_error = 0;

}

void PID::UpdateError(double cte) {

  //UPDATE PID values
  p_error = cte;
  Kp = p_error*Kp; //proportional constant... what should the constant be?
  
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