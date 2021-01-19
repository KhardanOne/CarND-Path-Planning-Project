#include "config.h"
#include "spline_def.h"
#include "helpers.h"
#include "localization.h"
#include "trajectory.h"
#include <iostream>

constexpr double kSmallDist = 5.0;       // define a point in front of the car this far away
constexpr double kFarPointDist1 = 35.0;  // how far away do we want our spline to end
constexpr double kFarPointDist2 = 60.0;  // TODO: make it dependent on the target car
constexpr double kFarPointDist3 = 85.0;

using std::cout;
using std::endl;

SplineDef::SplineDef(PrevPathFromSim const& sim_prev) {
  size_t nodes_to_keep = TrajectoryBuilder::NumKeptNodes(sim_prev);
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 3]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 3]);
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 2]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 2]);
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 1]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 1]);
}

SplineDef::SplineDef(LocalizationData const& ego) {
  double ref_yaw = DegToRad(ego.yaw);
  constexpr double small_dist = kSmallDist;
  double delta_x = small_dist * cos(ref_yaw);
  double delta_y = small_dist * sin(ref_yaw);
  xs.push_back(ego.x - delta_x);
  xs.push_back(ego.x);
  xs.push_back(ego.x + delta_x);
  ys.push_back(ego.y - delta_y);
  ys.push_back(ego.y);
  ys.push_back(ego.y + delta_y);
}

void SplineDef::Extend(int target_lane, const Map& map,
                       double ref_x, double ref_y, double ref_yaw) {
  vector<double> frenet = GetFrenet(ref_x, ref_y, ref_yaw, map);  // TODO: this function was never tested. Test it.
  vector<double> far_wp0 = GetXY(frenet[0] + kFarPointDist1, LaneToD(target_lane), map);
  vector<double> far_wp1 = GetXY(frenet[0] + kFarPointDist2, LaneToD(target_lane), map);
  vector<double> far_wp2 = GetXY(frenet[0] + kFarPointDist3, LaneToD(target_lane), map);
  xs.push_back(far_wp0[0]);
  xs.push_back(far_wp1[0]);
  xs.push_back(far_wp2[0]);
  ys.push_back(far_wp0[1]);
  ys.push_back(far_wp1[1]);
  ys.push_back(far_wp2[1]);
  if (CFG::kDebug)
    TrajectoryBuilder::VerifyIsMonotonic(xs, ys);
}
