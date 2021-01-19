#ifndef PID_H
#define PID_H

#include <cfloat>

class PID {
 public:
  PID(double k_p, double k_i, double k_d, 
      double min_out = DBL_MIN, double max_out = DBL_MAX);
  virtual ~PID() = default;

  void Reset();
  double Update(double error, double time = 1.0);
  double Get() const;

 private:
  double p_error_ = 0.0;
  double i_error_ = 0.0;
  double d_error_ = 0.0;
  double min_out_;
  double max_out_;

  double coeff_p_;
  double coeff_i_;
  double coeff_d_;
};

#endif  // PID_H
