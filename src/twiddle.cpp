#include "twiddle.h"
#include <iostream>


Twiddle::Twiddle(bool activate, PID &pid) {
  distance =0;
  best_distance =0;
  is_used = activate;
  error_sum = 0;
  best_error = 9999999;
  error_av = 0;
  dp = {.1, .0001, .1};
  sum_dp = sum_dp = dp[0]+dp[1]+dp[2];
  parameter_index = 0;
  p = {pid.Kp, pid.Ki, pid.Kd};
  init_done = false;
  increase_p = {true, true, true};

}
Twiddle::~Twiddle() {}


void Twiddle::new_best_err(){
  //update the performance values, increase dp values
  //and then go the next parameter
  best_error = error_av;
  dp[parameter_index] *= 1.01;
  best_distance = distance;
  update_sum_dp();
  increase_p[parameter_index] = true;
  //next parameter
  parameter_index += 1;
  parameter_index = parameter_index%3;
  }

void Twiddle::no_new_best_err(){
  //set back p values and decrease dp
  update_p();
  dp[parameter_index] *= 0.09;
  update_sum_dp();
}

void Twiddle::update_sum_dp(void){
  sum_dp = dp[0]+dp[1]+dp[2];
}

void Twiddle::update_p(){
  //increases or decreses p values
  if (increase_p[parameter_index]){
  p[parameter_index] += dp[parameter_index];
} else {
  p[parameter_index] -= 2*dp[parameter_index];
}
}

void Twiddle::update_pid(PID &pid){
  //write back internal twiggle values to PID controller
  pid.Kp = p[0];
  pid.Ki = p[1];
  pid.Kd = p[2];
}
void Twiddle::Init_Reference(){
  //initialize performance value after first lap
  //without having modified any parameter
  best_distance = distance;
  best_error = error_av;
  init_done = true;
}
void Twiddle::print_status(void){
  //for debugging
  std::cout << "dp values = " << dp[0] << " " << dp[1] << " " << dp[2]  << std::endl;
  std::cout << "p values = " << p[0] << " " << p[1] << " " << p[2]  << std::endl;
  std::cout << "Increasing p = " << increase_p[0] << " " << increase_p[1] << " " << increase_p[2] << std::endl;
  std::cout << "average error = " << error_av << std::endl;
  std::cout << "best distance = " << best_distance << std::endl;

}
