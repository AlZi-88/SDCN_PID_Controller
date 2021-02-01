
#ifndef TWIDDLE_H
#define TWIDDLE_H

class Twiddle{
public:
  Twiddle();

  virtual ~Twiddle();

  void new_best_err();

  void no_new_best_err();

  void update_sum_dp(void);

  void update_p(double factor);

  void update_pid(PID &pid);
  void Init(PID &pid);

private:

  int distance;
  bool is_used;
  double error;
  double best_error;
  double error_av;
  std::vector<double> dp;
  double sum_dp;
  int parameter_index;
  std::vector<double> p;


};


#endif //TWIDDLE_H
