#include "behavior_planner.h"
#include "helpers.h"
#include "config.h"
#include "trajectory.h"
#include "sensor_fusion.h"
#include "pid.h"
#include <cmath>
#include <iostream>
#include <iomanip>

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
  // print debug info
  bool log = false;

  static size_t nodes_added = 0;
  static size_t frame_count = 0;
  ++frame_count;
  cout << "\nfr:" << frame_count << " ";
  TrajectoryBuilder trajectory_builder(map, ego, sim_prev);
  SensorFusion sf(sensor_fusion, map.max_s);
  int lane = ego.GetLane();

  switch (state_) {

    case kKeepLane: {
      if (log) 
        cout << std::setw(7) << frame_count << ": KEEP_LANE   ";
      target_lane_ = sf.GetTargetLane(ego, map);
      if (target_lane_ > lane) {
        state_ = kGoRight;
      } else if (target_lane_ < lane) {
        state_ = kGoLeft;
      }
      break;
    }

    case kGoLeft: {
      if (log)
        cout << "GO_LEFT     ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        state_ = kKeepLane;
      }
      break;
    }

    case kGoRight: {
      if (log)
        cout << "GO_RIGHT    ";
      if (IsInLaneCenter(ego.d, target_lane_)) {
        state_ = kKeepLane;
      }
      break;
    }

    default: {
      if (log)
        cout << "INVALID     ";
    }
  }

#ifdef _WIN32
  // handle keys for testing: NUMPAD 1,2,3 for lanes 0, 1, 2
  if (GetKeyState(0x61)) { target_lane_ = 0; cout << "force lane: 0" << endl; }
  if (GetKeyState(0x62)) { target_lane_ = 1; cout << "force lane: 1" << endl; }
  if (GetKeyState(0x63)) { target_lane_ = 2; cout << "force lane: 2" << endl; }
#endif

  if (log) {
    // cout << /*"\n" << */ std::fixed << std::showpoint << std::setprecision(1);
    PrintStats(ego, map);
    cout << "ego lane:" << lane << " s:" << ego.s << " (" << ego.x << "," << ego.y
      << "@" << RadToDeg(ego.yaw) << ") speed:" << MpsToMph(ego.speed) << "mph";
    sf.PrintLaneChangeInfo(ego, map);
  }

  // prepare inf for CreateTrajectory
  double target_dist, target_speed;
  int front_car = sf.GetCarInFront(ego.s, target_lane_);
  if (front_car == -1) {
    target_dist = cfg::kInfinite;
    target_speed = cfg::kInfinite;
  } else {
    vector<double> const& raw = sf.cars_[front_car].raw;
    target_dist = GetDistanceForward(ego.s, raw[SF::S]);
    target_speed = Speed(raw[SF::VX], raw[SF::VY]);
  }
  nodes_added += trajectory_builder.Create(out_x_vals, out_y_vals, target_lane_, target_dist, target_speed);
}

void BehaviorPlanner::PrintStats(EgoCar const & ego_loc,
                                 Map const & map) {
  static size_t next_s_index = 0;
  static size_t s_size = map.waypoints_s.size();
  double s = fmod(ego_loc.s, map.max_s);
  if (s > map.waypoints_s[next_s_index]) {
    while (next_s_index < s_size && map.waypoints_s[next_s_index] < s) {
      ++next_s_index;
    }
    next_s_index %= s_size;
    cout << "\nmap.s[" << (next_s_index - 1) % s_size
      << "] at " << map.waypoints_s[(next_s_index - 1) % s_size]
      << " passed, s=" << s << " d=" << ego_loc.d
      << " x=" << ego_loc.x << " y=" << ego_loc.y
      << " yaw=" << RadToDeg(ego_loc.yaw) << "deg "
      << " speed=" << MpsToMph(ego_loc.speed) << "mph" << endl;
  }
}
