#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "trajectory.h"
#include "localization.h"
#include "map.h"
#include "sensor_fusion.h"
#include <string>

class BehaviorPlanner {
 public:
  enum State {kInvalid, kStarting, kKeepLane, kGoLeft, kGoRight};

  BehaviorPlanner();
  virtual ~BehaviorPlanner() = default;

  void GetTrajectory(vector<double>& out_x_vals,
                     vector<double>& out_y_vals,
                     Map const& map,
                     LocalizationData const& ego_loc,
                     vector<vector<double>> const& sensor_fusion,
                     PrevPathFromSim const& prev_path);

 private:
  static void PrintStats(LocalizationData const& ego_loc, Map const& map);

  vector<string> state_names_;
  int state_;
  int target_lane_ = 1;
};

#endif  // BEHAVIOR_PLANNER_H
