#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "vehicle.h"
#include "map.h"
#include <vector>
#include <Eigen/Dense>
using std::vector;

typedef Eigen::Vector3d Sigmas;
typedef Eigen::Array<double,6,1> Coeffs;

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
 * Calculates Jerk Minimizing Trajectory. One dimension only.
 * @param start pos, speed, acceleration
 * @param end   pos, speed, acceleration
 * @param time  when shall be end point reached, in seconds
 * @output 6 coefficients for a quintic polynomial
 */
// Coeffs JMT(Sigmas const & start, Sigmas const & end, double time);

/* 
 * Creates trajectory in form of two vectors, one for x and one for y coords.
 * @param out_x_vals OUTPUT vector of x coordinates
 * @param out_y_vals OUTPUT vector of y coordinates
 * @param target_lane lane index, where the leftmost is 0, rightmost is 2
 * @param target_delta_s the distance in Frenet coordinate system in meters
 * @param map used for frenet conversion, back and forth
 * @param ego the own car positional data received from the sim
 * @param prev_path path vectors not yet consumet, received from the sim
 */
void CreateTrajectory(vector<double> & /* out */ out_x_vals,
  vector<double> & /* out */ out_y_vals,
  int target_lane,
  double target_delta_s,
  Map const & map,
  LocalizationData const & ego,
  PreviousPath const & prev_path);

#endif //  TRAJECTORY_H
