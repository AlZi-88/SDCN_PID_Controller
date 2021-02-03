
#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <vector>

class Twiddle{
public:
  Twiddle(bool activate);

  virtual ~Twiddle();

  void new_best_err();

  void no_new_best_err();

  void update_sum_dp(void);

  void update_p(double factor);

  void update_pid(PID &pid);
  void Init(PID &pid);

  int distance;
  int best_distance;
  bool is_used;
  bool init_done;
  double error_sum;
  double best_error;
  double error_av;

  double sum_dp;
  double update_factor;
  int parameter_index;



private:

  std::vector<double> p;
  std::vector<double> dp;

};


#endif //TWIDDLE_H
