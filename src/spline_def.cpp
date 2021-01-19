#include "config.h"
#include "spline_def.h"
#include "helpers.h"
#include "localization.h"
#include "trajectory.h"
#include <iostream>

using std::cout;
using std::endl;

SplineDef::SplineDef(PrevPathFromSim const& sim_prev) {
  size_t nodes_to_keep = TrajectoryBuilder::NumKeptNodes(sim_prev);
  x.push_back(sim_prev.x_vals[nodes_to_keep - 3]);
  y.push_back(sim_prev.y_vals[nodes_to_keep - 3]);
  x.push_back(sim_prev.x_vals[nodes_to_keep - 2]);
  y.push_back(sim_prev.y_vals[nodes_to_keep - 2]);
  x.push_back(sim_prev.x_vals[nodes_to_keep - 1]);
  y.push_back(sim_prev.y_vals[nodes_to_keep - 1]);
}

SplineDef::SplineDef(LocalizationData const& ego) {
  double ref_yaw = DegToRad(ego.yaw);
  constexpr double small_dist = 10.0;  // to define another point right in front of us
  double delta_x = small_dist * cos(ref_yaw);
  double delta_y = small_dist * sin(ref_yaw);
  x.push_back(ego.x - delta_x);
  x.push_back(ego.x);
  x.push_back(ego.x + delta_x);
  y.push_back(ego.y - delta_y);
  y.push_back(ego.y);
  y.push_back(ego.y + delta_y);
}

void SplineDef::Extend(int target_lane, const Map& map,
                       double ref_x, double ref_y, double ref_yaw) {
  vector<double> sd = GetFrenet(ref_x, ref_y, ref_yaw, map);  // TODO: this function was never tested. Test it.
  vector<double> far_wp0 = GetXY(sd[0] + 30.0, LaneToD(target_lane), map);
  vector<double> far_wp1 = GetXY(sd[0] + 60.0, LaneToD(target_lane), map);
  vector<double> far_wp2 = GetXY(sd[0] + 90.0, LaneToD(target_lane), map);
  x.push_back(far_wp0[0]);
  x.push_back(far_wp1[0]);
  x.push_back(far_wp2[0]);
  y.push_back(far_wp0[1]);
  y.push_back(far_wp1[1]);
  y.push_back(far_wp2[1]);

  if (CFG::kDebug) {
    int x_greater = 0;
    int x_smaller = 0;
    int y_greater = 0;
    int y_smaller = 0;
    for (size_t i = 0; i < x.size() - 1; ++i) {
      if (x[i] < x[i+1])
        ++x_smaller;
      if (x[i] > x[i+1])
        ++x_greater;
      if (y[i] < y[i+1])
        ++y_smaller;
      if (y[i] > y[i+1])
        ++y_greater;
    }
    if (x_smaller < 5 && x_greater < 5 && y_smaller < 5 && y_greater < 5)
      cout << "ERROR: SplineDef(): x or y should be monotonously increasing or decreasing!" << endl;
  }
}
