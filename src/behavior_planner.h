#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "trajectory.h"
#include "vehicle.h"
#include "map.h"

class BehaviorPlanner {
 public:
  BehaviorPlanner() = default;
  virtual ~BehaviorPlanner() = default;

  static void GetTrajectory(vector<double>& /* out */ out_x_vals,
                            vector<double>& /* out */ out_y_vals,
                            Map const& map,
                            LocalizationData const& ego_loc,
                            vector<vector<double>> const& sensor_fusion,
                            PreviousPath const& prev_path);


 private:
  static void PrintStats(LocalizationData const& ego_loc, Map const& map);
};

#endif  // BEHAVIOR_PLANNER_H
