#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   /**
    * PID Errors
    */
   p_error = 0.0;
   i_error = 0.0;
   d_error = 0.0;
   cte_old = 0.0;
   /**
    * PID Coefficients
    */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

   p_error = cte;
   i_error += cte;
   d_error = (cte-cte_old);
   cte_old = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   double total_error;
   total_error = (-1.0)*Kp*p_error - Kd*d_error - Ki*i_error;
   std::cout << "total_error = " << total_error << ", Kp = " << Kp << ", Kd = " << Kd << ", Ki = " << Ki << std::endl;
  return total_error;  // TODO: Add your total error calc here!
}
