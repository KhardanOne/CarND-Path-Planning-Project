#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <string>

#include "ego_car.h"
#include "map.h"
#include "sensor_fusion.h"
#include "trajectory.h"

class BehaviorPlanner {
 public:
  enum State {kInvalid, kKeepLane, kGoLeft, kGoRight};

  BehaviorPlanner();
  virtual ~BehaviorPlanner() = default;

  void GetTrajectory(std::vector<double>& out_x_vals,
                     std::vector<double>& out_y_vals,
                     Map const& map,
                     EgoCar const& ego_loc,
                     std::vector<std::vector<double>> const& sensor_fusion,
                     PrevPathFromSim const& prev_path);

  void SwitchTo(State state);

 private:
  static void PrintStats(EgoCar const& ego_loc, Map const& map);

  std::vector<std::string> state_names_;
  int state_;
  int target_lane_ = 1;
  size_t nodes_added_ = 0;
  size_t frame_count_ = 0;
};

#endif  // BEHAVIOR_PLANNER_H
