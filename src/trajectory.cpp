#include "trajectory.h"
#include "helpers.h"
#include "config.h"
#include "spline.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using std::min;
using std::max;

void CreateTrajectory(vector<double>& /* out */ out_x_vals,
  vector<double>& /* out */ out_y_vals,
  int target_lane,
  double target_delta_s,
  Map const& map,
  LocalizationData const& ego,
  PreviousPath const& prev_path) {

  int prev_node_count = (int)prev_path.x_vals.size();
  long long nodes_to_keep = min(prev_node_count, CFG::trajectory_min_node_count);
  double ref_yaw = 0.0;

  // define the spline
  vector<double> spline_def_x;
  vector<double> spline_def_y;
  if (prev_node_count >= 3) {  // use the penultimate point as ref
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 3]);
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 2]);
    spline_def_x.push_back(prev_path.x_vals[nodes_to_keep - 1]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 3]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 2]);
    spline_def_y.push_back(prev_path.y_vals[nodes_to_keep - 1]);
    ref_yaw = atan2(spline_def_y[2] - spline_def_y[1], spline_def_x[2] - spline_def_x[1]);
    --nodes_to_keep;  // without this the last node will be added twice, causing problems
  }
  else {  // the car is the ref point
    // calculate one past and one future point
    double const& car_x_present = ego.x;
    double const& car_y_present = ego.y;
    ref_yaw = deg2rad(ego.yaw);
    constexpr double dist = 2.0;
    double delta_x = dist * cos(ref_yaw);
    double delta_y = dist * sin(ref_yaw);

    // add past, present and future points
    spline_def_x.push_back(car_x_present - delta_x);
    spline_def_x.push_back(car_x_present);
    spline_def_x.push_back(car_x_present + delta_x);
    spline_def_y.push_back(car_y_present - delta_y);
    spline_def_y.push_back(car_y_present);
    spline_def_y.push_back(car_y_present + delta_y);
  }

  double ref_x = spline_def_x[1];
  double ref_y = spline_def_y[1];

  // add 3 more points in the distance
  vector<double> far_wp0 = getXY(ego.s + 30.0, LaneToD(target_lane), map);
  vector<double> far_wp1 = getXY(ego.s + 60.0, LaneToD(target_lane), map);
  vector<double> far_wp2 = getXY(ego.s + 90.0, LaneToD(target_lane), map);
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

  tk::spline s;
  s.set_points(spline_def_x, spline_def_y);

  // store the coordinates in the output vector, starting with the prev ones
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
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;
  double N = target_dist / CFG::preferred_dist_per_frame;

  for (size_t i = out_x_vals.size(); i < 50; ++i) {
    double x = x_add_on + (target_x / N);
    double y = s(x);
    x_add_on = x;

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

  double & debug_dummy_for_brakepoint = target_y;
}

//void CreateTrajectory(vector<double>& /* out */ out_x_vals,
//  vector<double>& /* out */ out_y_vals,
//  int target_lane,
//  Map const& map,
//  LocalizationData const& ego,
//  PreviousPath const* prev_path) {
//  
//  out_x_vals.clear();
//  out_y_vals.clear();
//  out_x_vals.reserve(CFG::trajectory_node_count);
//  out_y_vals.reserve(CFG::trajectory_node_count);
//  int prev_node_count = (int)prev_path->x_vals.size();
//  int nodes_to_keep = min(prev_node_count, CFG::trajectory_min_node_count);
//  int i = 0;
//  if (prev_path) {
//    while (i < nodes_to_keep) {
//      out_x_vals.push_back(prev_path->x_vals[i]);
//      out_y_vals.push_back(prev_path->y_vals[i]);
//      ++i;
//    }
//  }
//
//  // TODO: this is temporary! Causes jump!
//  double target_d = double(target_lane) * CFG::lane_width + CFG::half_lane_width;
//
//  if (CFG::verbose >= CFG::All)
//    std::cout << "kept nodes: " << i << std::endl;
//
//  while (i < CFG::trajectory_node_count - 1) {
//    double next_s = ego.s + ((long long)i + 1) * CFG::preferred_dist_per_frame;
//    double next_d = target_d;
//    vector<double> xy = getXY(next_s, next_d, map.waypoints_s, map.waypoints_x, map.waypoints_y);
//
//    out_x_vals.push_back(xy[0]);
//    out_y_vals.push_back(xy[1]);
//    ++i;
//  }
//}

//Coeffs JMT(Sigmas const & start, Sigmas const & end, double time) {
//  double & t = time;
//  double t2 = t * t;
//  double t3 = t * t2;
//  double t4 = t * t3;
//  double t5 = t * t4;
//
//  Eigen::Matrix3d a;
//  a << t3,    t4,    t5,
//     3*t2,  4*t3,  5*t4,
//     6*t , 12*t2, 20*t3;
//
//  Eigen::Vector3d b;
//  b << end[0] - (start[0] +   start[1]*t + 0.5*start[2]*t2),
//       end[1] - (             start[1]   +     start[2]*t),
//       end[2] - (                              start[2]);
//
//  Eigen::Vector3d c;
//  c = a.inverse() * b;
//
//  Coeffs result;
//  result << start[0], start[1], 0.5*start[2], c.data()[0], c.data()[1], c.data()[2];
//
//  return result;
//}
