#include "behavior_planner.h"

#include <cmath>
#include <iomanip>
#include <iostream>

#include "config.h"
#include "helpers.h"
#include "pid.h"
#include "sensor_fusion.h"
#include "trajectory.h"

#ifdef _WIN32
#include <Windows.h>
#endif

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::vector;

BehaviorPlanner::BehaviorPlanner()
    : state_names_({"INVALID", "STARTING", "KEEP_LANE", "GO_LEFT", "GO_RIGHT"}),
      state_(kKeepLane) {}

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
      if (log) cout << ": KEEP_LANE   ";
      if (ego.speed > front_car_speed - 1.5 && ego.speed < front_car_speed + 5.0 && front_car_dist > 8.0)  // for safety
        target_lane_ = sf.SelectTargetLane(ego, map);
      if (target_lane_ > lane) {
        state_ = kGoRight;
      } else if (target_lane_ < lane) {
        state_ = kGoLeft;
      }
      break;
    }

    case kGoLeft: {
      if (log) cout << "GO_LEFT     ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        state_ = kKeepLane;
      }
      break;
    }

    case kGoRight: {
      if (log) cout << "GO_RIGHT    ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        state_ = kKeepLane;
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
