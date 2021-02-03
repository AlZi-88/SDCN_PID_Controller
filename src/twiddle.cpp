#include "twiddle.h"

Twiddle::Twiddle(bool activate) {
  distance =0;
  best_distance =0;
  is_used = activate;
  error_sum = 0;
  best_error = 9999999;
  error_av = 0;
  dp = {.01, .0001, .01};
  sum_dp = sum_dp = dp[0]+dp[1]+dp[2];
  parameter_index = 0;
  p = {0.01, 0.0001, 0.01};
  init_done = false;
  update_factor = 1.0;

}
Twiddle::~Twiddle() {}


void Twiddle::new_best_err(){
  best_error = error_av;
  dp[parameter_index] *= 1.01;
  best_distance = distance;

  }

void Twiddle::no_new_best_err(){
  update_p(1.0);
  dp[parameter_index] *= 0.09;
}

void Twiddle::update_sum_dp(void){
  sum_dp = dp[0]+dp[1]+dp[2];
}

void Twiddle::update_p(double factor){
  update_factor = factor;
  p[parameter_index] += factor*dp[parameter_index];
}

void Twiddle::update_pid(PID &pid){
  pid.Kp = p[0];
  pid.Ki = p[1];
  pid.Kd = p[2];
}
void Twiddle::Init(PID &pid){
  p[0] = pid.Kp;
  p[1] = pid.Ki;
  p[2] = pid.Kd;
  init_done = true;
}
