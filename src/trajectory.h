#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "vehicle.h"
#include "map.h"
#include <vector>

using std::vector;

/*
 * Received from the simulator.
 */
struct PreviousPath {
  vector<double> x_vals;
  vector<double> y_vals;
  double end_s;
  double end_d;
};

/* 
 * Creates trajectory in form of two vectors, one for x and one for y coords.
 * @param out_x_vals OUTPUT vector of x coordinates
 * @param out_y_vals OUTPUT vector of y coordinates
 * @param target_lane lane index, where the leftmost is 0, rightmost is 2
 * @param front_car_dist the Distance in Frenet coordinate system in meters
 * @param front_car_speed_mps speed of the car in m/s
 * @param map used for frenet conversion, back and forth
 * @param ego the own car positional data received from the sim
 * @param prev_path path vectors not yet consumet, received from the sim
 */
void CreateTrajectory(vector<double>& out_x_vals,
  vector<double>& out_y_vals,
  int target_lane,
  double front_car_dist,
  double front_car_speed_mps,
  Map const& map,
  LocalizationData const& ego,
  PreviousPath const& prev_path);

#endif  // TRAJECTORY_H
