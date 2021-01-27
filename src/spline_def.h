#ifndef SPLINE_H
#define SPLINE_H

#include <vector>

struct PrevPathFromSim;
struct EgoCar;
struct Map;

/* 
 * Definitin of a spline.
 * The xs and ys values serve as inputs for tk_spline set_points().
 */
struct SplineDef {
  SplineDef() = default;
  
  /*
   * Needs at least 3 x and y values in sim_prev to be used 
   * as starting tangent.
   */
  SplineDef(PrevPathFromSim const& sim_prev, size_t nodes_to_keep);
  
  /*
   * Create a point in the front of the car, and a point behind the car.
   * Use those points to construct the spline definition.
   */
  SplineDef(double x, double y, double yaw);

  /*
   * Add 3 more points in the distance for a smooth transition.
   */
  void Extend(int target_lane, Map const& map, EgoCar const& ego);

  std::vector<double> xs;
  std::vector<double> ys;
};

#endif  // SPLINE_H
