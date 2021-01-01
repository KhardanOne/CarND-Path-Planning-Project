#pragma once
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "trajectory.h"
#include "vehicle.h"
#include "map.h"

/**
  * SensorFusion
  */
enum class SF {ID, X, Y, VX, VY, S, D};

class BehaviorPlanner {

 public:
  BehaviorPlanner() = default;
  virtual ~BehaviorPlanner() = default;

  void GetTrajectory(vector<double> & /* out */ out_x_vals,
    vector<double> & /* out */ out_y_vals,
    Map const & map,
    LocalizationData const & ego_loc,
    vector<vector<double>> const & sensor_fusion,
    PreviousPath const & prev_path
  ) const;
};

#endif //  BEHAVIOR_PLANNER_H
