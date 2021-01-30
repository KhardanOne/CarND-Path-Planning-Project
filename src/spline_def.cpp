#include "spline_def.h"

#include <iostream>

#include "config.h"
#include "ego_car.h"
#include "helpers.h"
#include "trajectory.h"

constexpr double kSmallDist = 2.0;       // define a point in front of the car this far away
constexpr double kFarPointDist1 = 40.0;  // how far away do we want our spline to end
constexpr double kFarPointDist2 = 70.0;  // TODO: make it dependent on the target car
constexpr double kFarPointDist3 = 100.0;

using std::cout;
using std::endl;
using std::vector;


SplineDef::SplineDef(PrevPathFromSim const& sim_prev, size_t nodes_to_keep) {
  if (cfg::kVerbose >= cfg::kAll)
    cout << "nodes_to_keep:" << nodes_to_keep;
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 3]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 3]);
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 2]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 2]);
  xs.push_back(sim_prev.x_vals[nodes_to_keep - 1]);
  ys.push_back(sim_prev.y_vals[nodes_to_keep - 1]);
}

SplineDef::SplineDef(double x, double y, double yaw) {
  if (cfg::kVerbose >= cfg::kImportant)
    cout << "start with 0 nodes";
  double delta_x = kSmallDist * cos(yaw);
  double delta_y = kSmallDist * sin(yaw);
  xs.push_back(x - delta_x);
  xs.push_back(x);
  xs.push_back(x + delta_x);
  ys.push_back(y - delta_y);
  ys.push_back(y);
  ys.push_back(y + delta_y);
}

// TODO: use more points if available
// TODO: check for sudden changes
void SplineDef::Extend(int target_lane, Map const& map, EgoCar const& ego) {
  // TODO: if GetFrenet() is just approximate (which probably is), then it causes jumps !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  vector<double> far_wp0 = GetXY(ego.s + kFarPointDist1, LaneToD(target_lane), map);
  vector<double> far_wp1 = GetXY(ego.s + kFarPointDist2, LaneToD(target_lane), map);
  vector<double> far_wp2 = GetXY(ego.s + kFarPointDist3, LaneToD(target_lane), map);
  xs.push_back(far_wp0[0]);
  xs.push_back(far_wp1[0]);
  xs.push_back(far_wp2[0]);
  ys.push_back(far_wp0[1]);
  ys.push_back(far_wp1[1]);
  ys.push_back(far_wp2[1]);
  if (cfg::kDebug) {
    //TrajectoryBuilder::IsMonotonic(xs, ys, ref_x, ref_y);
  } 
}
