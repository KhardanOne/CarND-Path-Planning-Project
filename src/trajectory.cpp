#include "trajectory.h"
#include "helpers.h"
#include "config.h"
#include "spline.h"
#include <cmath>
#include <iostream>
using std::min;
using std::max;

void CreateTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double>& /* out */ out_y_vals,
  int target_lane,
  double front_car_dist,
  double front_car_speed_mps,
  Map const& map,
  LocalizationData const& ego,
  PreviousPath const& prev_path) {

  int prev_node_count = (int)prev_path.x_vals.size();
  long long nodes_to_keep = min(prev_node_count, CFG::trajectory_min_node_count);
  double ref_yaw = 0.0;
  double prev_displacement = 0.0;  // distance between last and penultimate positions

  // define the spline
  vector<double> spline_def_x;
  vector<double> spline_def_y;
  constexpr double small_dist = 2.0;  // to define another point just in front of us
  if (prev_node_count >= 3) {
    // use the penultimate point as a reference
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 3]);
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 2]);
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 1]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 3]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 2]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 1]);
    ref_yaw = atan2(spline_def_y[2] - spline_def_y[1], spline_def_x[2] - spline_def_x[1]);
    --nodes_to_keep;  // without this the last node will be added twice later, causing problems
    prev_displacement = distance(spline_def_x[1], spline_def_y[1], spline_def_x[2], spline_def_y[2]);
  }
  else {
    // the car is the reference point
    // calculate a past point and a future point
    double const & car_x_present = ego.x;
    double const & car_y_present = ego.y;
    ref_yaw = deg2rad(ego.yaw);
    double delta_x = small_dist * cos(ref_yaw);
    double delta_y = small_dist * sin(ref_yaw);

    // add past, present and future points
    spline_def_x.push_back(car_x_present - delta_x);
    spline_def_x.push_back(car_x_present);
    spline_def_x.push_back(car_x_present + delta_x);
    spline_def_y.push_back(car_y_present - delta_y);
    spline_def_y.push_back(car_y_present);
    spline_def_y.push_back(car_y_present + delta_y);
    // the car was standing still
    prev_displacement = 0.0;
  }

  // build trajectory from here
  double ref_x = spline_def_x[1];
  double ref_y = spline_def_y[1];

  // add 3 more points in the distance
  double target_car_dist, target_car_speed_mps;
  if (front_car_dist >= CFG::infinite) {
    target_car_dist = CFG::infinite;
    target_car_speed_mps = CFG::preferred_speed_mps;
  }
  else {
    target_car_dist = max(front_car_dist, CFG::car_length);
    target_car_speed_mps = front_car_speed_mps;
  }

  vector<double> far_wp0 = getXY(ego.s + 40.0, LaneToD(target_lane), map);
  vector<double> far_wp1 = getXY(ego.s + 60.0, LaneToD(target_lane), map);
  vector<double> far_wp2 = getXY(ego.s + 80.0, LaneToD(target_lane), map);
  spline_def_x.push_back(far_wp0[0]);
  spline_def_x.push_back(far_wp1[0]);
  spline_def_x.push_back(far_wp2[0]);
  spline_def_y.push_back(far_wp0[1]);
  spline_def_y.push_back(far_wp1[1]);
  spline_def_y.push_back(far_wp2[1]);

  // transform coordinates into car coordinate system
  for (size_t i = 0; i < spline_def_x.size(); ++i) {
    double shift_x = spline_def_x[i] - ref_x;
    double shift_y = spline_def_y[i] - ref_y;
    spline_def_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    spline_def_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }

  tk::spline spl;
  spl.set_points(spline_def_x, spline_def_y);

  // store the coordinates in the output vector, starting with the previous ones
  // keep only the first nodes_to_keep positions
  out_x_vals.clear();
  out_y_vals.clear();
  out_x_vals.reserve(CFG::trajectory_node_count);
  out_y_vals.reserve(CFG::trajectory_node_count);
  if (prev_node_count >= 3) {
    for (int i = 0; i < nodes_to_keep; ++i) {
      out_x_vals.push_back(prev_path.x_vals[i]);
      out_y_vals.push_back(prev_path.y_vals[i]);
    }
  }

  // calculate how to break up the spline points
  constexpr double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_progress = 0.0;

  double split_num = target_dist / CFG::preferred_dist_per_frame;
  double preferred_delta_x = target_x / split_num;
  double x_ratio = target_x / target_dist;
  double last_x_displacement = prev_displacement * x_ratio;

  if (target_car_speed_mps >= ego.speed_mph * CFG::MPH_MPS || target_car_dist > 30.0) { //  TODO: improve
    // accelerate or keep speed

    // preferred_delta_x *= target_car_speed_mps / CFG::preferred_speed_mph;
    double x_disp_accel = CFG::preferred_dist_per_frame_increment * x_ratio;
    
    for (size_t i = out_x_vals.size(); i < CFG::trajectory_node_count; ++i) {
      double x_displacement = min(last_x_displacement + x_disp_accel, preferred_delta_x);

      last_x_displacement = x_displacement;
      double x = x_progress + x_displacement;
      double y = spl(x);
      x_progress = x;

      // transform coordinates back
      double delta_x = x;
      double delta_y = y;
      x = delta_x * cos(ref_yaw) - delta_y * sin(ref_yaw);
      y = delta_x * sin(ref_yaw) + delta_y * cos(ref_yaw);
      x += ref_x;
      y += ref_y;

      out_x_vals.push_back(x);
      out_y_vals.push_back(y);
    }
  }
  else {
    // deccelerate

    double delta_speed_mps = target_car_speed_mps - ego.speed_mph * CFG::MPH_MPS;
    // distance from target car where braking needs to be started
    double dist_to_start_braking = delta_speed_mps * delta_speed_mps / CFG::preferred_deccel_mpss;
    // distance from ego car where braking needs to be started
    dist_to_start_braking = target_car_dist - dist_to_start_braking;
    double x_disp_deccel = CFG::preferred_dist_per_frame_decrement * x_ratio;

    for (size_t i = out_x_vals.size(); i < CFG::trajectory_node_count; ++i) {
      double x_displacement = (x_progress > dist_to_start_braking) ? 
        x_displacement = last_x_displacement-x_disp_deccel : last_x_displacement;

      last_x_displacement = x_displacement;
      double x = x_progress + x_displacement;
      double y = spl(x);
      x_progress = x;

      // transform coordinates back
      double delta_x = x;
      double delta_y = y;
      x = delta_x * cos(ref_yaw) - delta_y * sin(ref_yaw);
      y = delta_x * sin(ref_yaw) + delta_y * cos(ref_yaw);
      x += ref_x;
      y += ref_y;

      out_x_vals.push_back(x);
      out_y_vals.push_back(y);
    }
  }
}
