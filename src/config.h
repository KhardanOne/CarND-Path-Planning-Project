#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <limits>
#include <float.h>
using std::vector;

namespace CFG {
  enum Verbose {OFF, Important, All};

  constexpr double MPH_MPS = 0.44704;

  constexpr int    verbose = All;

  constexpr double trajectory_length_s = 2.0;
  constexpr double trajectory_min_length_s = 0.2; // don't drop node closer to this

  constexpr double sim_time_step_s = 0.02;

  constexpr double preferred_speed_mph = 49.5;
  constexpr double preferred_speed_mps = preferred_speed_mph * MPH_MPS;
  // constexpr double max_speed_mph = 50.0;
  constexpr double max_accel_mpss = 9.0;
  constexpr double preferred_accel_mpss = 5.0;
  constexpr double preferred_deccel_mpss = 3.0;
  // constexpr double max_jerk_mpsss = 9.0;

  constexpr int    lane_count = 3;
  constexpr double lane_width = 4.0;

  //constexpr double sigma_s[3] = { preferred_speed_mps, 6.0, 6.0 };
  //constexpr double sigma_d[3] = { 3.0, 6.0, 6.0 };

  // calculated values

  // constexpr double max_speed_mps = max_speed_mph * 0.44704;
  constexpr double preferred_dist_per_frame = preferred_speed_mps * sim_time_step_s;
  constexpr double preferred_dist_per_frame_increment = preferred_accel_mpss * sim_time_step_s * sim_time_step_s;
  constexpr double preferred_dist_per_frame_decrement = preferred_deccel_mpss * sim_time_step_s * sim_time_step_s;
  constexpr int    trajectory_node_count = int(trajectory_length_s / sim_time_step_s);
  constexpr int    trajectory_min_node_count = int(trajectory_min_length_s / sim_time_step_s);
  constexpr double half_lane_width = lane_width / 2.0;

  constexpr double lane_buffer = 14.0; //  distance to follow the forward car from, car center to car center
  constexpr double infinite = DBL_MAX;
  constexpr double car_length = 5.0;
};

#endif //  CONFIG_H
