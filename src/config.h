#ifndef CONFIG_H
#define CONFIG_H

namespace CFG {
  enum Verbose {OFF, Important, All};
  constexpr int verbose = All;

  constexpr double trajectory_length_s = 2.0;
  constexpr double trajectory_min_length_s = 0.2; // don't drop node closer to this
  constexpr double sim_time_step_s = 0.02;
  constexpr double preferred_speed_mph = 49.0;
  constexpr double preferred_accel_mpss = 9.0;
  constexpr double max_accel_mpss = 9.8;
  constexpr int lane_count = 3;
  constexpr double lane_width = 4.0;
  
  // calculated values

  constexpr double preferred_speed_mps = preferred_speed_mph * 0.44704;
  constexpr double preferred_dist_per_frame = preferred_speed_mps * sim_time_step_s;
  constexpr int trajectory_node_count = int(trajectory_length_s / sim_time_step_s);
  constexpr int trajectory_min_node_count = int(trajectory_min_length_s / sim_time_step_s);
  constexpr double half_lane_width = lane_width / 2.0;
};

#endif //  CONFIG_H
