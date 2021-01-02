#include "behavior_planner.h"
#include "helpers.h"
#include "config.h"
#include "trajectory.h"
#include <cmath>
#include <iostream>

using std::min;
using std::max;

void BehaviorPlanner::GetTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double> & /* out */ out_y_vals,
  Map const & map,
  LocalizationData const & ego_loc,
  vector <vector<double>> const & sensor_fusion,
  PreviousPath const & prev_path
) const {

  CreateTrajectory(out_x_vals, out_y_vals, ego_loc.GetLane(), 30.0, map, ego_loc, prev_path);

}
