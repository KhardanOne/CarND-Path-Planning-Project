#include "pid.h"

#include <algorithm>

using std::min;
using std::max;

PD::PD(double k_p, double k_d, double min_out, double max_out)
  : coeff_p_(k_p), coeff_d_(k_d), min_out_(min_out),
    max_out_(max_out), p_error_(0.0), d_error_(0.0) {}

double PD::Update(double error) {
  d_error_ = error - p_error_;
  p_error_ = error;
  return Get();
}

void PD::Reset() {
  p_error_ = 0.0;
  d_error_ = 0.0;
}

double PD::Get() const {
  double result = max(min_out_, min(max_out_,
    -coeff_p_ * p_error_ - coeff_d_ * d_error_));
  return result;
}


PDPow::PDPow(double k_p, double k_d, double k_e, double min_out, double max_out)
  : PD(k_p, k_d, min_out, max_out), exp_(k_e) {}

double PDPow::Get() const {
  const double sign = double(p_error_ >= 0) - double(p_error_ < 0);
  double result = max(min_out_, min(max_out_,
    -coeff_p_ * sign * pow(abs(p_error_), exp_) - coeff_d_ * d_error_));
  return result;
}


PID::PID(double k_p, double k_i, double k_d, double min_out, double max_out)
  : coeff_p_(k_p), coeff_i_(k_i), coeff_d_(k_d), min_out_(min_out),
    max_out_(max_out), p_error_(0.0), i_error_(0.0), d_error_(0.0) {}

double PID::Update(double error, double time) {
  i_error_ += error * time;
  d_error_ = error - p_error_;
  p_error_ = error;
  return Get();
}

void PID::Reset() {
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

double PID::Get() const {
  double result = max(min_out_, min(max_out_,
    -coeff_p_ * p_error_ - coeff_i_ * i_error_ - coeff_d_ * d_error_));
  return result;
}
