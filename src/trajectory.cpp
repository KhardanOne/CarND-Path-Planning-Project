#include "trajectory.h"
#include "helpers.h"
#include "config.h"
#include <cmath>
#include <iostream>
using std::min;
using std::max;

void CreateTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double>& /* out */ out_y_vals,
  int target_lane,
  Map const& map,
  LocalizationData const& ego,
  PreviousPath const* prev_path) {
  
  out_x_vals.clear();
  out_y_vals.clear();
  out_x_vals.reserve(CFG::trajectory_node_count);
  out_y_vals.reserve(CFG::trajectory_node_count);
  int prev_node_count = (int)prev_path->x_vals.size();
  int nodes_to_keep = min(prev_node_count, CFG::trajectory_min_node_count);
  int i = 0;
  if (prev_path) {
    while (i < nodes_to_keep) {
      out_x_vals.push_back(prev_path->x_vals[i]);
      out_y_vals.push_back(prev_path->y_vals[i]);
      ++i;
    }
  }

  // TODO: this is temporary! Causes jump!
  double target_d = double(target_lane) * CFG::lane_width + CFG::half_lane_width;

  if (CFG::verbose >= CFG::All)
    std::cout << "kept nodes: " << i << std::endl;

  while (i < CFG::trajectory_node_count - 1) {
    double next_s = ego.s + ((long long)i + 1) * CFG::preferred_dist_per_frame;
    double next_d = target_d;
    vector<double> xy = getXY(next_s, next_d, map.waypoints_s, map.waypoints_x, map.waypoints_y);

    out_x_vals.push_back(xy[0]);
    out_y_vals.push_back(xy[1]);
    ++i;
  }
}
