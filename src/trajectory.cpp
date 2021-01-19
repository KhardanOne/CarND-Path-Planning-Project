#include "trajectory.h"
#include "helpers.h"
#include "config.h"
#include "tk_spline.h"
#include "pid.h"
#include "spline_def.h"
#include <cmath>
#include <iostream>

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::vector;

TrajectoryBuilder::TrajectoryBuilder(Map const& map,
                                     LocalizationData const& ego,
                                     PrevPathFromSim const& sim_prev)
  : map_(map), ego_(ego), sim_prev_(sim_prev) {
  SetRef();
  if (CFG::kDebug)
    VerifyIsMonotonic(sim_prev.x_vals, sim_prev.y_vals);
}

bool TrajectoryBuilder::VerifyIsMonotonic(vector<double> const& xs, vector<double> const& ys) {
  size_t count = xs.size();
  int x_greater = 0;
  int x_smaller = 0;
  int y_greater = 0;
  int y_smaller = 0;
  if (count > 0) {
    for (size_t i = 0; i < count-1; ++i) {
      if (xs[i] < xs[i+1])
        ++x_smaller;
      if (xs[i] > xs[i+1])
        ++x_greater;
      if (ys[i] < ys[i+1])
        ++y_smaller;
      if (ys[i] > ys[i+1])
        ++y_greater;
    }
  }
  if (count > 0 && x_smaller < count-1 && x_greater < count-1 && y_smaller < count-1 && y_greater < count-1) {
    cout << "ERROR: TrajectoryBuilder::VerifyIsMonotonic(): x or y should be monotonously increasing or decreasing!" << endl;
    cout << "    Count: " << count << " x_smaller:" << x_smaller << " x_greater:" << x_greater;
    cout << " y_smaller:" << y_smaller << " y_greater:" << y_greater << endl;
    return false;
  } else {
    return true;
  }
}

tk::spline TrajectoryBuilder::DefineSpline(int target_lane) const {
  SplineDef spline_def;
  enum class DebugType {CONTINUE, START_NEW} dbg;
  if (CanContinuePrevPath()) {
    spline_def = SplineDef(sim_prev_);
    dbg = DebugType::CONTINUE;
  } else {
    spline_def = SplineDef(ego_);
    dbg = DebugType::START_NEW;
  }
  spline_def.Extend(target_lane, map_, ref_x_, ref_y_, ref_yaw_);
  if (spline_def.xs[5] < 0.0)
    cout << "ERROR: Extend returned a negative value" << endl;
  vector<double> xValsBeforeTransform(spline_def.xs.begin(), spline_def.xs.end());
  vector<double> yValsBeforeTransform(spline_def.ys.begin(), spline_def.ys.end());
  TransformCoordsIntoRefSys(spline_def.xs, spline_def.ys);
  if (CFG::kDebug && spline_def.xs[5] < 0.0)
    cout << "ERROR: Extend or TransferCoordsIntoRefSys returned a negative value" << endl;
  tk::spline spl;
  spl.set_points(spline_def.xs, spline_def.ys);
  return spl;
}

void TrajectoryBuilder::Create(vector<double>& out_x_vals,
                               vector<double>& out_y_vals,
                               int target_lane,
                               double front_car_dist,
                               double front_car_speed_mps) {
  size_t kept_nodes_count = NumKeptNodes(sim_prev_);
  InitOutAndCopy(kept_nodes_count, out_x_vals, out_y_vals);
  tk::spline spl = DefineSpline(target_lane);

  // calculate how to break up the spline points
  constexpr double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double x_progress = 0.0;

  double split_num = target_dist / CFG::kPreferredDistPerFrame;
  double preferred_delta_x = target_x / split_num;
  double x_ratio = target_x / target_dist;
  double last_x_displacement = ref_displacement_ * x_ratio;

  // calculate target pose
  double target_car_dist, target_car_speed_mps;
  if (front_car_dist >= CFG::kInfinite) {
    target_car_dist = CFG::kInfinite;
    target_car_speed_mps = CFG::kPreferredSpeedMps;
  } else {
    target_car_dist = max(front_car_dist, CFG::kCarLength);
    target_car_speed_mps = front_car_speed_mps;
  }
  target_car_dist -= (CFG::kCarLength + CFG::kBufferDist + kept_nodes_count * CFG::kPreferredDistPerFrame);
  
  double delta_speed_mps = target_car_speed_mps - ego_.speed_mph * CFG::kMphToMps;  // TODO: FIXIT: the trajectory end position and speed should be used
  // Distance from the target car where braking needs to be started
  double dist_to_start_breaking = (delta_speed_mps > 0.0) ?
    0.0 : delta_speed_mps * delta_speed_mps / CFG::kPreferredDeccelMpss;
  // Distance from ego car where braking needs to be started
  dist_to_start_breaking = target_car_dist - dist_to_start_breaking;

  // PID controller to smoothen follow distance                   // TODO: this cannot work since instatiation, only P
  constexpr double pid_kp = 0.1;                                  // TODO: reset it when?
  constexpr double pid_kd = 0.5;
  static double pid_prev_p = 0.0;
  double& pid_p = dist_to_start_breaking;
  double pid_out = abs(max(min(-pid_kp * pid_p - pid_kd * (pid_p-pid_prev_p), 1.0), -1.0));
  pid_prev_p = pid_p;
  double x_disp_accel = CFG::kPreferredDistPerFrameIncrement * x_ratio * pid_out;
  double x_disp_deccel = CFG::kMaxDistPerFrameDecrement * x_ratio * pid_out;
  //cout << "PID out:" << pid_out << " P:" << pid_p << " D:" << (pid_p - pid_prev_p)
  //  << " dist to start braking: " << dist_to_start_breaking << endl;
  
  
  // TODO: abs prevents errors... by other errors?
  //static PID follow_pid(0.1, 0.0, 0.5, -1.0, 1.0);
  //double pid_out = abs(follow_pid.Update(dist_to_start_breaking));
  //double x_disp_accel = CFG::kPreferredDistPerFrameIncrement * x_ratio * pid_out;
  //double x_disp_deccel = CFG::kMaxDistPerFrameDecrement * x_ratio * pid_out;

  
  // create trajectory nodes
  for (size_t i = kept_nodes_count; i < CFG::kTrajectoryNodeCount; ++i) {
    delta_speed_mps = target_car_speed_mps - ego_.speed_mph * CFG::kMphToMps;  // TODO: FIXIT: the trajectory end position and speed should be used
    // Distance from the target car where braking needs to be started
    dist_to_start_breaking = (delta_speed_mps > 0.0) ?
      0.0 : delta_speed_mps * delta_speed_mps / CFG::kPreferredDeccelMpss;
    // Distance from ego car where braking needs to be started
    dist_to_start_breaking = target_car_dist - dist_to_start_breaking;
    double x_displacement;

    //cout << i << " target speed:" << target_car_speed_mps << " ego speed:" << ego.speed_mph
    //  << " delta:" << delta_speed_mps << " start braking in " << dist_to_start_breaking << " meters" << endl;

    if (dist_to_start_breaking < 0.0) { //  TODO: improve
      // deccelerate
      x_displacement = max(last_x_displacement - x_disp_deccel, 0.0);
    } else {
      // accelerate or keep speed
      x_displacement = min(last_x_displacement + x_disp_accel, preferred_delta_x);
    }
    last_x_displacement = x_displacement;
    double x = x_progress + x_displacement;
    double y = spl(x);
    x_progress = x;

    vector<double> pos = TransformCoordFromRef(x, y);  // TODO: check if any point "turns back" when braking
    out_x_vals.push_back(pos[0]);
    out_y_vals.push_back(pos[1]);
  }
  if (CFG::kDebug)
    VerifyIsMonotonic(out_x_vals, out_y_vals);
}

size_t TrajectoryBuilder::NumKeptNodes(PrevPathFromSim const& sim_prev) {
  size_t prev_node_count = sim_prev.x_vals.size();
  size_t nodes_to_keep = min(prev_node_count, (size_t)CFG::kTrajectoryMinNodeCount);
  return nodes_to_keep;
}

bool TrajectoryBuilder::CanContinuePrevPath() const {
  if (sim_prev_.x_vals.size() < 3)
    return false;
  size_t last = sim_prev_.x_vals.size() - 1;
  constexpr double epsilon = 0.0001;
  return (Distance(sim_prev_.x_vals[last], sim_prev_.y_vals[last],
                   sim_prev_.x_vals[last - 1], sim_prev_.y_vals[last - 1]) > epsilon
          && Distance(sim_prev_.x_vals[last - 1], sim_prev_.y_vals[last - 1],
                      sim_prev_.x_vals[last - 2], sim_prev_.y_vals[last - 2]) > epsilon);
}

void TrajectoryBuilder::InitOutAndCopy(size_t nodes_to_copy_count,
                                       vector<double>& out_x_vals,
                                       vector<double>& out_y_vals) const {
  out_x_vals.clear();
  out_y_vals.clear();
  out_x_vals.reserve(CFG::kTrajectoryNodeCount);
  out_y_vals.reserve(CFG::kTrajectoryNodeCount);
  if (sim_prev_.x_vals.size() >= 3) {
    for (size_t i = 0; i < nodes_to_copy_count; ++i) {
      out_x_vals.push_back(sim_prev_.x_vals[i]);
      out_y_vals.push_back(sim_prev_.y_vals[i]);
    }
  }
}

vector<double> TrajectoryBuilder::TransformCoordFromRef(double x, double y) const {
  double delta_x = x;
  double delta_y = y;
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

void TrajectoryBuilder::SetRef() {
  if (CanContinuePrevPath()) {
    size_t last = NumKeptNodes(sim_prev_) - 1;
    ref_yaw_ = atan2(sim_prev_.y_vals[last] - sim_prev_.y_vals[last-1],
                     sim_prev_.x_vals[last] - sim_prev_.x_vals[last-1]);
    ref_x_ = sim_prev_.x_vals[last];
    ref_y_ = sim_prev_.y_vals[last];
    ref_displacement_ = Distance(sim_prev_.x_vals[last-1], sim_prev_.y_vals[last-1],
                                 sim_prev_.x_vals[last], sim_prev_.y_vals[last]);
  } else {
    ref_yaw_ = DegToRad(ego_.yaw);
    ref_x_ = ego_.x;
    ref_y_ = ego_.y;
    ref_displacement_ = 0.0;
  }
}
