#include "pid.h"
#include <iostream>
#include <algorithm>

using std::min;
using std::max;
using std::cout;
using std::endl;

PID::PID(double k_p, double k_i, double k_d, double min_out, double max_out)
  : coeff_p_(k_p), coeff_i_(k_i), coeff_d_(k_d), min_out_(min_out),
    max_out_(max_out), p_error_(0.0), i_error_(0.0), d_error_(0.0) {}

void PID::Reset() {
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

double PID::Update(double error, double time) {
  i_error_ += error * time;
  d_error_ = error - p_error_;
  p_error_ = error;
  return Get();
}

double PID::Get() const {
  //if (i_error_ > 0.0)
  //  cout << "i_error: " << i_error_ << endl;
  return max(min_out_, min(max_out_, -coeff_p_ * p_error_ - coeff_i_ * i_error_ 
                           - coeff_d_ * d_error_));
}
