#include "trajectory.h"

#include <iomanip>
#include <iostream>

#include "tk_spline.h"
#include "config.h"
#include "helpers.h"
#include "pid.h"
#include "spline_def.h"

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::vector;

TrajectoryBuilder::TrajectoryBuilder(Map const& map,
                                     EgoCar const& ego,
                                     PrevPathFromSim const& sim_prev,
                                     bool force_restart)
  : map_(map),
    ego_(ego),
    sim_prev_(sim_prev),
    kept_prev_nodes_count_(NumNodesToKeep(force_restart)) {
  SetReferencePose();
}

Spline TrajectoryBuilder::DefineSpline(int target_lane) const {
  SplineDef spline_def;
  if (kept_prev_nodes_count_ >= 3) {
    spline_def = SplineDef(sim_prev_, kept_prev_nodes_count_);
  } else {
    spline_def = SplineDef(ref_x_, ref_y_, ref_yaw_);
  }
  spline_def.Extend(target_lane, map_, ego_);
  TransformCoordsIntoRefSys(spline_def.xs, spline_def.ys);
  Spline spline;
  spline.set_points(spline_def.xs, spline_def.ys);
  return spline;
}

size_t TrajectoryBuilder::NumNodesToKeep(bool force_restart) const {
  if (!force_restart && CanContinuePrevPath()) {
    const size_t prev_node_count = sim_prev_.x_vals.size();
    const size_t nodes_to_keep = min(prev_node_count, (size_t)cfg::kTrajectoryMinNodeCount);
    return nodes_to_keep;
  }
  return 0;
}

size_t TrajectoryBuilder::Create(vector<double>& out_x_vals,
                                 vector<double>& out_y_vals,
                                 int target_lane,
                                 double front_car_dist,
                                 double front_car_speed) {
  InitOutput(out_x_vals, out_y_vals);
  size_t nodes_added = CopyPrevious(out_x_vals, out_y_vals);
  const Spline spline = DefineSpline(target_lane);

  const double target_dist = max(front_car_dist, 0.0) - cfg::kCarLength - cfg::kBufferDist;

  // in-place stateless PD-Controller
  const double pd_error_d   = (ego_.speed - front_car_speed) * cfg::kSimTimeStepS;
  const double pd_out       = 0.1 * target_dist - 3.0 * pd_error_d;
  const double throttle     = Crop(0.0,  pd_out, 1.0);
  const double brake        = Crop(0.0, -pd_out, 1.0);
  const double x_disp_accel = cfg::kPreferredDistPerFrameIncrement * throttle;  // triangle ratio omitted because close to 1.0
  const double x_disp_decel = -cfg::kMaxDistPerFrameDecrement * brake;          // triangle ratio omitted because close to 1.0

  static double prev_x_displacement = 0.0;
  const double x_displacement = max(0.0, min(cfg::kPreferredDistPerFrame,
    prev_x_displacement + ((throttle > 0.0) ? x_disp_accel : x_disp_decel)));
  prev_x_displacement = x_displacement;

  if (x_displacement > 0.00000001) {  // avoid having two points whith the same coords
    double x = 0.0;
    for (size_t i = kept_prev_nodes_count_; i < cfg::kTrajectoryNodeCount; ++i) {
      x += x_displacement;
      const double y = spline(x);
      const vector<double> pos = TransformCoordFromRef(x, y);
      out_x_vals.push_back(pos[0]);
      out_y_vals.push_back(pos[1]);
      ++nodes_added;
    }
  }
  return nodes_added;
}

bool TrajectoryBuilder::CanContinuePrevPath() const {
  if (sim_prev_.x_vals.size() < 3) {
    cout << "PrevPath too short (" << sim_prev_.x_vals.size() << ") - START NEW PATH" << endl;
    return false;
  }
  return true;
}

void TrajectoryBuilder::InitOutput(vector<double>& out_x_vals,
                                   vector<double>& out_y_vals) const {
  out_x_vals.clear();
  out_y_vals.clear();
  out_x_vals.reserve(cfg::kTrajectoryNodeCount);
  out_y_vals.reserve(cfg::kTrajectoryNodeCount);
}

size_t TrajectoryBuilder::CopyPrevious(vector<double>& out_x_vals,
                                       vector<double>& out_y_vals) const {
  size_t nodes_added = 0;
  if (kept_prev_nodes_count_ >= 3 && sim_prev_.x_vals.size() >= 3) {
    for (size_t i = 0; i < kept_prev_nodes_count_; ++i) {
      out_x_vals.push_back(sim_prev_.x_vals[i]);
      out_y_vals.push_back(sim_prev_.y_vals[i]);
      ++nodes_added;
    }
  } else if (kept_prev_nodes_count_ >= 3 && sim_prev_.x_vals.size() < 3) {
    cout << "ERROR: TrajectoryBuilder::CopyPrevious(): less than 3 nodes to keep!" << endl;
  }
  return nodes_added;
}

vector<double> TrajectoryBuilder::TransformCoordFromRef(double x, double y) const {
  const double delta_x = x;
  const double delta_y = y;
  x = delta_x * cos(ref_yaw_) - delta_y * sin(ref_yaw_);
  y = delta_x * sin(ref_yaw_) + delta_y * cos(ref_yaw_);
  x += ref_x_;
  y += ref_y_;
  return { x, y };
}

void TrajectoryBuilder::TransformCoordsIntoRefSys(vector<double>& x_in_out_vals,
                                                  vector<double>& y_in_out_vals) const {
  for (size_t i = 0; i < x_in_out_vals.size(); ++i) {
    double delta_x = x_in_out_vals[i] - ref_x_;
    double delta_y = y_in_out_vals[i] - ref_y_;
    x_in_out_vals[i] = delta_x * cos(-ref_yaw_) - delta_y * sin(-ref_yaw_);
    y_in_out_vals[i] = delta_x * sin(-ref_yaw_) + delta_y * cos(-ref_yaw_);
  }
}

void TrajectoryBuilder::SetReferencePose() {
  double ref_displacement = -1.0;
  if (kept_prev_nodes_count_ >= 3) {
    const size_t last = kept_prev_nodes_count_ - 1;
    ref_yaw_ = atan2(sim_prev_.y_vals[last] - sim_prev_.y_vals[last-1],
                     sim_prev_.x_vals[last] - sim_prev_.x_vals[last-1]);
    ref_x_ = sim_prev_.x_vals[last];
    ref_y_ = sim_prev_.y_vals[last];
    ref_displacement = Distance(sim_prev_.x_vals[last-1], sim_prev_.y_vals[last-1],
                                sim_prev_.x_vals[last], sim_prev_.y_vals[last]);
  } else {
    ref_yaw_ = ego_.yaw;
    ref_x_ = ego_.x;
    ref_y_ = ego_.y;
    ref_displacement = ego_.speed * cfg::kSimTimeStepS;
  }
  ref_speed_ = ref_displacement / cfg::kSimTimeStepS;
}
