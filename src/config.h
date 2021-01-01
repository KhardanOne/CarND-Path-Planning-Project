#ifndef CONFIG_H
#define CONFIG_H

namespace CFG {
  constexpr double trajectory_length_sec = 2.0;
  constexpr double sim_time_step_sec = 0.02;
  constexpr double preferred_speed_mph = 49.5;
  constexpr double preferred_speed_mps = preferred_speed_mph * 0.44704;
  constexpr double preferred_dist_per_frame = preferred_speed_mps * sim_time_step_sec;
  constexpr int trajectory_node_count = int(trajectory_length_sec / sim_time_step_sec);
};

#endif //  CONFIG_H
