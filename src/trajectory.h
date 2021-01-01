#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "vehicle.h"
#include "map.h"
#include <vector>
using std::vector;

struct PreviousPath {
  vector<double> x_vals;
  vector<double> y_vals;
  double end_s;
  double end_d;
};

void CreateTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double>& /* out */ out_y_vals,
  int target_lane,
  Map const& map,
  LocalizationData const& ego,
  PreviousPath const* prev_path);

#endif //  TRAJECTORY_H
