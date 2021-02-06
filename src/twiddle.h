
#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <vector>

class Twiddle{
public:
  Twiddle(bool activate, PID &pid);

  virtual ~Twiddle();

  void new_best_err();

  void no_new_best_err();

  void update_sum_dp(void);

  void update_p();

  void update_pid(PID &pid);
  void Init_Reference();
  void print_status(void);
  //number of iterations
  int distance;
  //highest number of iterations
  int best_distance;
  //activate or deactivates the twiggle algorithm
  bool is_used;
  //indication whether the car is having it's
  //first lap or if it is already improving the parameters
  bool init_done;
  //integrated absolute error
  double error_sum;
  //best error of all trials
  double best_error;
  //average error of current lap
  double error_av;
  //sum of all delta updates for the parameters
  double sum_dp;
  //index of the currently modified parameter
  int parameter_index;
  //internal PID values
  std::vector<double> p;
  //changes which are applied to p values
  std::vector<double> dp;
  //indicates if corresponding parameter value is increased or decreased
  std::vector < bool > increase_p;


private:



};


#endif //TWIDDLE_H
