#ifndef SPLINE_H
#define SPLINE_H

#include <vector>

using std::vector;

struct PrevPathFromSim;
struct LocalizationData;
struct Map;

/* 
 * Definitin of a spline.
 * The xs and ys values are the inputs for tk_spline set_points().
 */
struct SplineDef {
  SplineDef() = default;
  
  // Needs at least 3 x and y values in sim_prev to be used as starting tangent.
  SplineDef(PrevPathFromSim const& sim_prev, size_t nodes_to_keep);
  
  // Create 2 points in front of the car, and use them as starting tangent.
  SplineDef(LocalizationData const& ego);

  // Add 3 more points in the distance for a smooth spine.
  void Extend(int target_lane, Map const& map, LocalizationData const& ego
              /*double ref_x, double ref_y, double ref_yaw*/);

  vector<double> xs;
  vector<double> ys;
};

#endif  // SPLINE_H
