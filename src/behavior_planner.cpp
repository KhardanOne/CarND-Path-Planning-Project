#include "behavior_planner.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "config.h"
#include "helpers.h"
#include "sensor_fusion.h"
#include "trajectory.h"

#ifdef _WIN32
#include <Windows.h>
#endif

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::string;
using std::vector;

BehaviorPlanner::BehaviorPlanner()
    : state_names_({"INVALID", "keep lane", "go left", "go right"}),
      state_(kKeepLane) {}

bool BehaviorPlanner::ShouldConsiderLaneChanges(EgoCar const& ego,
                                                double front_car_dist,
                                                double front_car_speed,
                                                bool verbose) const {
  bool result = false;
  string log_msg = "  ";
  if (front_car_dist < 30.0) {
    // catching up a car
    const bool not_much_slower = ego.speed > front_car_speed - 1.5;
    const bool not_much_faster = ego.speed < front_car_speed + 3.0;
    const bool not_too_slow    = ego.speed > 8.0;
    const bool not_too_close   = front_car_dist > 17.0;
    result = not_much_slower && not_much_faster && not_too_slow && not_too_close;
    if (verbose) {
      log_msg.append("T ");
      log_msg.append(not_much_slower ? "r" : "R");
      log_msg.append(not_much_faster ? "f" : "F");
      log_msg.append(not_too_slow    ? "s" : "S");
      log_msg.append(not_too_close   ? "c" : "C");
    }
  } else {
    // no car ahead
    const bool not_too_slow    = ego.speed > 8.0;
    result = not_too_slow;
    if (verbose) {
      log_msg.append("| ..");
      log_msg.append(not_too_slow    ? "s." : "S.");
    }
  }
  if (verbose) {
    log_msg.append(result ? " check lanes" : " wait");
    cout << log_msg << endl;
  }
  return result;
}

void BehaviorPlanner::GetTrajectory(vector<double>& out_x_vals,
                                    vector<double>& out_y_vals,
                                    Map const& map,
                                    EgoCar const& ego,
                                    std::vector<std::vector<double>> const& sensor_fusion,
                                    PrevPathFromSim const& sim_prev) {
  bool log = false;

  ++frame_count_;
  if (log) cout << "\nfr:" << frame_count_ << " ";
  SensorFusion sf(sensor_fusion, map.max_s);
  const int lane               = ego.GetLane();
  const int front_car          = sf.GetCarInFront(ego.s, lane);
  const double front_car_dist  = front_car > -1 ? GetDistanceForward(ego.s, sf.cars_[front_car].raw[SF::S])
                                                : cfg::kInfinite;
  const double front_car_speed = front_car > -1 ? sf.GetSpeed(front_car) : cfg::kPreferredSpeed;

  switch (state_) {

    case kKeepLane: {
      if (log) cout << ": keep lane   ";
      bool rare_log = (frame_count_ % 20 == 0);
      if (rare_log) cout << "s:" << std::setw(9) << ego.s; // << " dist: " << front_car_dist << " dv: " << std::setw(12) << ego.speed - front_car_speed;
      if (ShouldConsiderLaneChanges(ego, front_car_dist, front_car_speed, rare_log)) {
        target_lane_ = sf.SelectTargetLane(ego, map);
        if (target_lane_ > lane) {
          SwitchTo(kGoRight);
        } else if (target_lane_ < lane) {
          SwitchTo(kGoLeft);
        }
      }
      break;
    }

    case kGoLeft: {
      if (log) cout << "go left     ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        SwitchTo(kKeepLane);
      }
      break;
    }

    case kGoRight: {
      if (log) cout << "go_right    ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        SwitchTo(kKeepLane);
      }
      break;
    }

    default: {
      if (log) cout << "INVALID     ";
    }
  }

#ifdef _WIN32
  // handle keys for testing: NUMPAD 1, 2, 3 for lanes 0, 1, 2
  if (GetKeyState(0x61)) { target_lane_ = 0; cout << "force lane: 0" << endl; }
  if (GetKeyState(0x62)) { target_lane_ = 1; cout << "force lane: 1" << endl; }
  if (GetKeyState(0x63)) { target_lane_ = 2; cout << "force lane: 2" << endl; }
#endif

  if (log) {
    PrintStats(ego, map);
    sf.PrintLaneChangeInfo(ego, map);
  }

  // prepare info for CreateTrajectory
  double target_dist, target_speed;
  int target_front_car = sf.GetCarInFront(ego.s, target_lane_);
  if (target_front_car == -1) {
    target_dist = cfg::kInfinite;
    target_speed = cfg::kInfinite;
  } else {
    vector<double> const& raw = sf.cars_[target_front_car].raw;
    target_dist = GetDistanceForward(ego.s, raw[SF::S]);
    target_speed = Speed(raw[SF::VX], raw[SF::VY]);
  }

  // taking over with big speed difference is dangerous... avoid it
  if (target_lane_ != lane && front_car_dist < 40.0) {
    target_speed = min(target_speed, max(0.0, front_car_speed) - cfg::kTakeOverSpeedDiff);
  }

  TrajectoryBuilder trajectory_builder(map, ego, sim_prev);
  nodes_added_ += trajectory_builder.Create(out_x_vals, out_y_vals, target_lane_, target_dist, target_speed);
}

void BehaviorPlanner::SwitchTo(State state) {
  bool log = true;
  state_ = state;
  if (log) cout << "state: " << state_names_[state_] << endl;
}

void BehaviorPlanner::PrintStats(EgoCar const & ego,
                                 Map const & map) {
  // cout << /*"\n" << */ std::fixed << std::showpoint << std::setprecision(1);
  static size_t next_s_index = 0;
  static size_t s_size = map.waypoints_s.size();
  double s = fmod(ego.s, map.max_s);
  if (s > map.waypoints_s[next_s_index]) {
    while (next_s_index < s_size && map.waypoints_s[next_s_index] < s) {
      ++next_s_index;
    }
    next_s_index %= s_size;
    cout << "\nmap.s[" << (next_s_index - 1) % s_size
      << "] at " << map.waypoints_s[(next_s_index - 1) % s_size]
      << " passed, s=" << s << " d=" << ego.d
      << " x=" << ego.x << " y=" << ego.y
      << " yaw=" << RadToDeg(ego.yaw) << "deg "
      << " speed=" << MpsToMph(ego.speed) << "mph" << endl;
  }
  cout << "ego lane:" << ego.GetLane() << " s:" << ego.s << " (" << ego.x << "," << ego.y
    << "@" << RadToDeg(ego.yaw) << ") speed:" << MpsToMph(ego.speed) << "mph";
}
