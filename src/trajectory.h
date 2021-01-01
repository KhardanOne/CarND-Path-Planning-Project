#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
using std::vector;

class Trajectory {
 
 public:
  Trajectory() = default;
  virtual ~Trajectory() = default;
};

struct PreviousPath {
  vector<double> x_vals;
  vector<double> y_vals;
  double end_s;
  double end_d;
};

#endif //  TRAJECTORY_H
