#include "behavior_planner.h"
#include "helpers.h"
#include "config.h"
#include "trajectory.h"
#include "sensor_fusion.h"
#include <cmath>
#include <iostream>

using std::min;
using std::max;

void BehaviorPlanner::GetTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double> & /* out */ out_y_vals,
  Map const & map,
  LocalizationData const & ego_loc,
  vector <vector<double>> const & sensor_fusion,
  PreviousPath const & prev_path) {

  if (CFG::verbose >= CFG::Verbose::All) {
    //PrintStats(ego_loc, map);
  }

  double front_car_dist, front_car_speed;
  int lane = DToLane(ego_loc.d);
  SensorFusion sf(sensor_fusion, map.max_s);
  int front_car_id = sf.GetCarInFront(ego_loc.s, lane);
  if (front_car_id == -1) {
    front_car_dist = CFG::infinite;
    front_car_speed = CFG::infinite;
  }
  else {
    vector<double> const & raw = sf.cars[front_car_id].raw;
    front_car_dist = distance(ego_loc.x, ego_loc.y, raw[SF::X], raw[SF::Y]);
    front_car_speed = sqrt(raw[SF::VX] * raw[SF::VX] + raw[SF::VY] * raw[SF::VY]);
  }
  std::cout << "front car:" << front_car_id << " dist:" << front_car_dist
    << " speed:" << front_car_speed << std::endl;

  // TODO: tmp, remove
  // if (front_car_dist < 50.0 && front_car_speed < CFG::preferred_speed_mph)
  //  lane = 0;

  CreateTrajectory(out_x_vals, out_y_vals, lane, front_car_dist, front_car_speed,
    map, ego_loc, prev_path);

}

void BehaviorPlanner::PrintStats(LocalizationData const & ego_loc,
  Map const & map) {

  static size_t next_s_index = 0;
  static size_t s_size = map.waypoints_s.size();
  double s = fmod(ego_loc.s, map.max_s);
  if (s > map.waypoints_s[next_s_index % s_size]) {
    for (; s > map.waypoints_s[next_s_index % s_size]; ++next_s_index);
    next_s_index %= s_size;
    std::cout << "map.s[" << (next_s_index - 1) % s_size
      << "] at " << map.waypoints_s[(next_s_index - 1) % s_size]
      << " passed, s=" << s << ", d=" << ego_loc.d
      << ", x=" << ego_loc.x << ", y=" << ego_loc.y
      << ", yaw=" << ego_loc.yaw << std::endl;
  }
}
