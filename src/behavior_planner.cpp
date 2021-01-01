#include "behavior_planner.h"
#include "helpers.h"
#include "config.h"

void BehaviorPlanner::GetTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double> & /* out */ out_y_vals,
  Map const & map,
  LocalizationData const & ego_loc,
  vector <vector<double>> const & sensor_fusion,
  PreviousPath const & prev_path
) const {

  for (int i = 0; i < CFG::trajectory_node_count; ++i) {
    out_x_vals.push_back(ego_loc.x + (CFG::preferred_dist_per_frame * i) * cos(deg2rad(ego_loc.yaw)));
    out_y_vals.push_back(ego_loc.y + (CFG::preferred_dist_per_frame * i) * sin(deg2rad(ego_loc.yaw)));
  }
}
