#include "trajectory.h"
#include "helpers.h"
#include "config.h"
#include "tk_spline.h"
#include "pid.h"
#include "spline_def.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::vector;


// TODO: add GetYawAtEnd(), then continue from that angle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// TODO: try to generate 100% new trajectory each frame (instead of extending old one)
TrajectoryBuilder::TrajectoryBuilder(Map const& map,
                                     EgoCar const& ego,
                                     PrevPathFromSim const& sim_prev,
                                     bool force_restart)
  : map_(map),
    ego_(ego),
    sim_prev_(sim_prev),
    kept_prev_nodes_count_(NumNodesToKeep(force_restart)){

  SetReferencePose();

  if (CFG::kDebug) {
    static double dbg_last_dx = 0.0;
    cout << "size:" << sim_prev.x_vals.size() << " ego x:" << ego.x << " y:" << ego.y
      << " mph:" << MpsToMph(ego.speed) << " deg:" << RadToDeg(ego.yaw);
    if (sim_prev.x_vals.size() > 0) {
      cout<<" -> prev x[0]:"<<sim_prev.x_vals[0]<<" y[0]:"<<sim_prev.y_vals[0];
      cout << " dx:" << sim_prev.x_vals[0] - ego.x << " ddx:"<<ego.x-sim_prev.x_vals[0]-dbg_last_dx;
      cout<<" dy:"<<ego.y-sim_prev.y_vals[0];
      dbg_last_dx = ego.x - sim_prev.x_vals[0];
    }
    cout << endl;

    static double dbg_prev_yaw = ego.yaw;
    if (abs(RadToDeg(ego.yaw - dbg_prev_yaw)) > 5.0)
      cout << "change in ego.yaw greater than 5 deg: " << RadToDeg(ego.yaw-dbg_prev_yaw) << endl;
    dbg_prev_yaw = ego.yaw;

    VerifyIsMonotonic(sim_prev.x_vals, sim_prev.y_vals, ego.x, ego.y);
  }
}


bool TrajectoryBuilder::VerifyIsMonotonic(vector<double> const& xs, vector<double> const& ys,
                                          double cur_x, double cur_y) {
  size_t count = xs.size();
  int x_greater = 0;
  int x_smaller = 0;
  int y_greater = 0;
  int y_smaller = 0;
  if (count > 0) {
    if (cur_x < xs[0])
      ++x_smaller;
    else if (cur_x > xs[0])
      ++x_greater;
    if (cur_y < ys[0])
      ++y_smaller;
    else if (cur_y > ys[0])
      ++y_greater;
    for (size_t i = 0; i < count-1; ++i) {
      if (xs[i] < xs[i+1])
        ++x_smaller;
      else if (xs[i] > xs[i+1])
        ++x_greater;
      if (ys[i] < ys[i+1])
        ++y_smaller;
      if (ys[i] > ys[i+1])
        ++y_greater;
    }
  }
  if (count > 0 && x_smaller < count && x_greater < count && y_smaller < count && y_greater < count) {
    cout << "ERROR: TrajectoryBuilder::VerifyIsMonotonic(): x or y should be monotonously increasing or decreasing!" << endl;
    cout << "    Count: " << count << " x_smaller:" << x_smaller << " x_greater:" << x_greater;
    cout << " y_smaller:" << y_smaller << " y_greater:" << y_greater << endl;
    return false;
  } else {
    return true;
  }
}


bool TrajectoryBuilder::AreSpeedAccJerkOk(vector<double> const& xs,
                                          vector<double> const& ys,
  double cur_x,
  double cur_y,
  double cur_yaw,
  double cur_speed_mps) {
  bool speeds_ok = AreSpeedsOk(xs, ys, cur_x, cur_y);
  bool acc_jerk_ok = AreAccelerationsJerksOk(xs, ys, cur_x, cur_y, cur_yaw, cur_speed_mps);
  if (speeds_ok && acc_jerk_ok) {
    return true;
  }
  else {
    cout << "ERROR: TrajectoryBuilder::AreSpeedAccJerkOk(): ";
    if (!speeds_ok)
      cout << "some of the speeds are wrong!" << endl;
    else
      cout << "some of the accelerations or jerk values are wrong!" << endl;
    return false;
  }
}


bool TrajectoryBuilder::AreAccelerationsJerksOk(vector<double> const& xs,
                                                vector<double> const& ys,
  double cur_x,
  double cur_y,
  double cur_yaw,
  double cur_speed_mps) {
  // empty is Ok
  if (xs.size() == 0) {
    cout << "AreAccelerationsJerksOk(): vector is empty and that is OK." << endl;
    return true;
  }

  // measure the acceleration from ego car to the first (0th) element
  vector<double> prev_acc = Acceleration(cur_x, cur_y, cur_yaw, cur_speed_mps, xs[0], ys[0]);
  double& prev_acc_comb = prev_acc[0];
  double& prev_acc_tang = prev_acc[1];
  double& prev_acc_norm = prev_acc[2];
  double prev_speed = cur_speed_mps;
  double prev_yaw = cur_yaw;

  if (prev_acc_comb > CFG::kAccHardLimitMpss) {
    cout << "ERROR: AreAccelerationsJerksOk(): Acc from ego pose to the 0th node too high! AccT:"
      << prev_acc_tang << " AccN:" << prev_acc_norm << " AccTotal:" << prev_acc_comb << endl;
    return false;
  }

  // check accelerations between the nodes in input vectors
  double jerk_limit_pow2 = CFG::kJerkHardLimitMpsss * CFG::kJerkHardLimitMpsss;
  if (xs.size() > 1) {
    for (size_t cur = 1; cur < xs.size() - 1; ++cur) {
      const size_t prev = cur - 1;
      vector<double> acc = Acceleration(xs[prev], ys[prev], prev_yaw, prev_speed, xs[cur], ys[cur]);
      double& acc_comb = acc[0];
      double& acc_tang = acc[1];
      double& acc_norm = acc[2];

      if (acc_comb > CFG::kAccHardLimitMpss) {
        cout << "ERROR: AreAccelerationsJerksOk(): Acc to " << cur << "th node too high! AccT:"
          << acc_tang << " AccN:" << acc_norm << " AccTotal:" << acc_comb << endl;
        return false;
      }

      // check jerk
      double jerk_tang = (acc_tang - prev_acc_tang); // / CFG::kSimTimeStepS;
      double jerk_norm = (acc_norm - prev_acc_norm); // / CFG::kSimTimeStepS;
      double jerk_comb_pow2 = jerk_tang * jerk_tang + jerk_norm * jerk_norm;
      if (jerk_comb_pow2 > jerk_limit_pow2) {
        cout << "ERROR: AreAccelerationsJerksOk(): Jert to " << cur << "th node too high! JertT:"
          << jerk_tang << " JerkN:" << jerk_norm << " JerkTotal:" << sqrt(jerk_comb_pow2) << endl;
        return false;
      }

      // prepare for next iteration
      prev_acc_comb = acc_comb;
      prev_acc_tang = acc_tang;
      prev_acc_norm = acc_norm;
      prev_speed = Distance(xs[cur], ys[cur], xs[prev], ys[prev]) / CFG::kSimTimeStepS;
      prev_yaw = atan2(ys[cur] - ys[prev], xs[cur] - xs[prev]);
    }
  }
  return true;  // Everything is OK or there are not enough nodes to classify them wrong
}


tk::spline TrajectoryBuilder::DefineSpline(int target_lane) const {
  SplineDef spline_def;
  enum class DebugType { CONTINUE, START_NEW } dbgSplineCreationMethod;
  if (kept_prev_nodes_count_ >= 3) {
    spline_def = SplineDef(sim_prev_, kept_prev_nodes_count_);
    dbgSplineCreationMethod = DebugType::CONTINUE;
  } else {
    spline_def = SplineDef(ref_x_, ref_y_, ref_yaw_);
    dbgSplineCreationMethod = DebugType::START_NEW;
  }
  spline_def.Extend(target_lane, map_, ego_);  // TODO: test
  if (spline_def.xs[5] < 0.0)
    cout << "ERROR: Extend returned a negative value" << endl;
  vector<double> xDbgValsBeforeTransform(spline_def.xs.begin(), spline_def.xs.end());
  vector<double> yDbgValsBeforeTransform(spline_def.ys.begin(), spline_def.ys.end());
  TransformCoordsIntoRefSys(spline_def.xs, spline_def.ys);
  if (CFG::kDebug && spline_def.xs[5] < 0.0)
    cout << "ERROR: Extend or TransferCoordsIntoRefSys returned a negative value" << endl;
  tk::spline spl;
  spl.set_points(spline_def.xs, spline_def.ys);
  return spl;
}


size_t TrajectoryBuilder::NumNodesToKeep(bool force_restart) const {
  if (!force_restart && CanContinuePrevPath()) {
    const size_t prev_node_count = sim_prev_.x_vals.size();
    const size_t nodes_to_keep = min(prev_node_count, (size_t)CFG::kTrajectoryMinNodeCount);
    return nodes_to_keep;
  } else {
    return 0;
  }
}


size_t TrajectoryBuilder::Create(vector<double>& out_x_vals,
                                 vector<double>& out_y_vals,
                                 int target_lane,
                                 double front_car_dist,
                                 double front_car_speed_mps) {
  bool log = false;
  size_t nodes_added = 0;
  nodes_added += InitOutAndCopy(out_x_vals, out_y_vals);  // TODO: THIS IS IT!!! It copies even when restarting
  tk::spline spl = DefineSpline(target_lane);

  // calculate how to break up the spline points
  constexpr double target_x = 30.0;
  const double target_y = spl(target_x);
  const double target_dist = sqrt(target_x * target_x + target_y * target_y);
  const double split_pieces = target_dist / CFG::kPreferredDistPerFrame;
  const double preferred_delta_x = target_x / split_pieces;
  const double x_ratio = target_x / target_dist;
  
  // calculate target pose
  double target_car_dist, target_car_speed_mps;
  if (front_car_dist >= CFG::kInfinite) {
    target_car_dist = CFG::kInfinite;
    target_car_speed_mps = CFG::kPreferredSpeedMps;
  } else {
    target_car_dist = max(front_car_dist, CFG::kCarLength);
    target_car_speed_mps = front_car_speed_mps;
  }
  target_car_dist -= CFG::kCarLength + CFG::kBufferDist + LengthInMeters(out_x_vals, out_y_vals, ego_.x, ego_.y);

  double end_speed = GetEndSpeed(out_x_vals, out_y_vals, ego_.x, ego_.y, ego_.speed);
  double delta_speed_mps = target_car_speed_mps - end_speed;
  double dist_to_start_breaking = (delta_speed_mps > 0.0) ?
      target_car_dist
    : target_car_dist - delta_speed_mps / CFG::kPreferredDecelMpss; // TODO: verify!

  double x_progress = 0.0;
  double last_x_displacement = ref_speed_mps_ * CFG::kSimTimeStepS * x_ratio;

  // PID controller to smoothen the follow distance                 // TODO: this cannot work since instatiation, only P
  constexpr double pid_kp = 0.2;
  constexpr double pid_kd = 15.0;
  static double pid_prev_p = 0.0;
  double& pid_p = dist_to_start_breaking;
  double pid_out = abs(max(min(-pid_kp * pid_p - pid_kd * (pid_p - pid_prev_p), 1.0), -1.0));
  if (true) {  // log
    double pid_dbg = max(min(-pid_kp * pid_p - pid_kd * (pid_p - pid_prev_p), 1.0), -1.0);
    cout << std::setw(14) << pid_dbg << std::setw(6) << long long (dist_to_start_breaking) << endl;
  }
  pid_prev_p = pid_p;
  double x_disp_accel = CFG::kPreferredDistPerFrameIncrement * x_ratio * pid_out;
  double x_disp_decel = CFG::kMaxDistPerFrameDecrement * x_ratio * pid_out;
  //cout << "PID out:" << pid_out << " P:" << pid_p << " D:" << (pid_p - pid_prev_p)
  //  << " dist to start braking: " << dist_to_start_breaking << endl;

  // TODO: abs prevents errors... by other errors?
  //static PID follow_pid(0.1, 0.0, 0.5, -1.0, 1.0);  // TODO: reset it when?
  //double pid_out = abs(follow_pid.Update(dist_to_start_breaking));
  //double x_disp_accel = CFG::kPreferredDistPerFrameIncrement * x_ratio * pid_out;
  //double x_disp_decel = CFG::kMaxDistPerFrameDecrement * x_ratio * pid_out;

  // create trajectory nodes
  for (size_t i = kept_prev_nodes_count_; i < CFG::kTrajectoryNodeCount; ++i) {
    end_speed = GetEndSpeed(out_x_vals, out_y_vals, ego_.x, ego_.y, ego_.speed);
    delta_speed_mps = target_car_speed_mps - end_speed;
    dist_to_start_breaking = (delta_speed_mps > 0.0) ?
        target_car_dist   // TODO: use current value instead of the old one
      : target_car_dist - delta_speed_mps / CFG::kPreferredDecelMpss; // TODO: verify!
    if (log)
      cout << i << " targetdist:" << target_car_dist <<  " speed:" << target_car_speed_mps
        << " endspd:" << end_speed << " delta:" << delta_speed_mps << " brake in:"
        << dist_to_start_breaking << "m" << endl;
    const double x_displacement = (dist_to_start_breaking < 0.0) ?
        max(last_x_displacement - x_disp_decel, 0.0)                 // decelerate
      : min(last_x_displacement + x_disp_accel, preferred_delta_x);  // accelerate or keep speed

    if (x_displacement < 0.00000001)  // avoid having two points whith the same coords
      break;

    last_x_displacement = x_displacement;
    const double x = x_progress + x_displacement;
    const double y = spl(x);
    x_progress = x;
    target_car_dist -= x_displacement;  // good enough and fast approximation
    const vector<double> pos = TransformCoordFromRef(x, y);
    out_x_vals.push_back(pos[0]);
    out_y_vals.push_back(pos[1]);
    ++nodes_added;
  }
  if (CFG::kDebug) {
    VerifyIsMonotonic(out_x_vals, out_y_vals, ego_.x, ego_.y);
    AreAccelerationsJerksOk(out_x_vals, out_y_vals, ego_.x, ego_.y, ego_.yaw, ego_.speed);
  }
  return nodes_added;
}


double TrajectoryBuilder::LengthInMeters(vector<double> const& xs, vector<double> const& ys,
                                         double cur_x, double cur_y) {
  const size_t count = xs.size();
  if (count == 0)
    return 0.0;

  return Distance(cur_x, cur_y, xs[count-1], ys[count-1]);
}

double TrajectoryBuilder::GetEndSpeed(vector<double> const& xs, vector<double> const& ys,
                                      double cur_x, double cur_y, double cur_speed) {
  const size_t count = xs.size();
  if (count == 0)
    return cur_speed;
 
  const double dist = (count == 1) ? Distance(cur_x, cur_y, xs[0], ys[0])
                                    : Distance(xs[count-2], ys[count-2], xs[count-1], ys[count-1]);
  return dist / CFG::kSimTimeStepS;
}


bool TrajectoryBuilder::CanContinuePrevPath() const {
  if (sim_prev_.x_vals.size() < 3) {
    cout << "PrevPath too short (" << sim_prev_.x_vals.size() << ") - START NEW PATH" << endl;
    return false;
  }
  return true; //////////////////////////////////////////////////// TODO: remove this

  size_t last = sim_prev_.x_vals.size() - 1;
  constexpr double kEpsilon = 0.000001;
  if (Distance(sim_prev_.x_vals[last], sim_prev_.y_vals[last],
      sim_prev_.x_vals[last - 1], sim_prev_.y_vals[last - 1]) < kEpsilon) {
    cout << "PrevPath last two nodes too close - START NEW PATH" << endl;
    return false;
  }

  if (Distance(sim_prev_.x_vals[last - 1], sim_prev_.y_vals[last - 1],
      sim_prev_.x_vals[last - 2], sim_prev_.y_vals[last - 2]) < kEpsilon) {
    cout << "PrevPath penultimate and pre-penultimate nodes too close - START NEW PATH" << endl;
    return false;
  }

  const vector<double> xs(sim_prev_.x_vals.end() - 3, sim_prev_.x_vals.end());
  const vector<double> ys(sim_prev_.y_vals.end() - 3, sim_prev_.y_vals.end());
  bool monotonic = VerifyIsMonotonic(xs, ys, ego_.x, ego_.y);
  if (!monotonic) {
    cout << "WARNING: TrajectoryBuilder::CanContinuePrePath(): prev path is not monotonic! - START NEW PATH" << endl;
    return false;
  }

  bool kinematics_ok = AreSpeedAccJerkOk(sim_prev_.x_vals, sim_prev_.y_vals, ego_.x, ego_.y, 
                                         ego_.yaw, ego_.speed);
  if (!kinematics_ok) {
    cout << "WARNING: Speed, acc or jerk is wrong - START NEW PATH" << endl;
    return false;
  }
  
  return true;
}


size_t TrajectoryBuilder::InitOutAndCopy(vector<double>& out_x_vals,
                                         vector<double>& out_y_vals) const {
  size_t nodes_added = 0;
  out_x_vals.clear();
  out_y_vals.clear();
  out_x_vals.reserve(CFG::kTrajectoryNodeCount);
  out_y_vals.reserve(CFG::kTrajectoryNodeCount);
  if (kept_prev_nodes_count_ >= 3 && sim_prev_.x_vals.size() >= 3) {
    for (size_t i = 0; i < kept_prev_nodes_count_; ++i) {
      out_x_vals.push_back(sim_prev_.x_vals[i]);
      out_y_vals.push_back(sim_prev_.y_vals[i]);
      ++nodes_added;
    }
  } else if (kept_prev_nodes_count_ >= 3 && sim_prev_.x_vals.size() < 3) {
    cout << "ERROR: TrajectoryBuilder::InitOutAndCopy(): less than 3 nodes to keep!" << endl;
  }
  return nodes_added;
}


bool TrajectoryBuilder::AreSpeedsOk(vector<double> const& xs, vector<double> const& ys,
                                    double cur_x, double cur_y) {
  // empty is Ok
  if (xs.size() == 0) {
    cout << "AreSpeedsOk(): vector is empty and that is OK." << endl;
    return true;
  }
  
  // measure the distance between the ego car and the first (0th) element
  double d = Distance(cur_x, cur_y, xs[0], ys[0]);
  if (d > CFG::kSpeedHardLimitDistPerFrame) {
    cout << "AreSpeedsOk(): speed from ego car to 0th node too high: " << d << endl;
    return false;
  }
  
  // check distances between the nodes in input vectors
  if (xs.size() > 1) {
    for (size_t cur = 1; cur < xs.size() - 1; ++cur) {
      const size_t prev = cur - 1;
      d = Distance(xs[cur], ys[cur], xs[prev], ys[prev]);
      if (d > CFG::kSpeedHardLimitDistPerFrame) {
        cout << "AreSpeedsOk(): speed to " << cur << "th node too high: "
          << d / CFG::kSpeedHardLimitDistPerFrame << " (displacement:" << d << ")" << endl;
        return false;
      }
    }
  }
  return true;
}


vector<double> TrajectoryBuilder::Acceleration(double cur_x, double cur_y, double cur_yaw, double cur_speed_mps,
                                               double target_x, double target_y) {
  Eigen::Vector3d previous_speed;
  previous_speed << cur_speed_mps * cos(cur_yaw), cur_speed_mps* sin(cur_yaw), 0.0;
  Eigen::Vector3d next_speed;
  next_speed << (target_x - cur_x) / CFG::kSimTimeStepS, (target_y - cur_y) / CFG::kSimTimeStepS, 0.0;
  const double next_speed_length = next_speed.norm();
  const Eigen::Vector3d cross_prod = previous_speed.cross(next_speed);
  const double cross_length = cross_prod.norm();
  const double sin_angle = cross_length / cur_speed_mps / next_speed_length;
  const double angle = asin(sin_angle);
  
  // tangential acc
  const double parallel_speed_length = next_speed_length * cos(angle);
  const double tang_acc = (parallel_speed_length - cur_speed_mps) / CFG::kSimTimeStepS;

  // normal acc
  const double ortho_speed = next_speed_length * sin_angle;
  const double ortho_acc = ortho_speed / CFG::kSimTimeStepS;

  // combined acc
  const double combined_acc = sqrt(tang_acc * tang_acc + ortho_acc * ortho_acc);
  return { combined_acc, tang_acc, ortho_acc };
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
    ref_displacement = ego_.speed * CFG::kSimTimeStepS;
  }
  ref_speed_mps_ = ref_displacement / CFG::kSimTimeStepS;
}
