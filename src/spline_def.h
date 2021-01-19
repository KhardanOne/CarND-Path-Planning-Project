#ifndef SPLINE_H
#define SPLINE_H

#include <vector>

using std::vector;

struct PrevPathFromSim;
struct LocalizationData;
struct Map;

struct SplineDef {
  SplineDef() = default;
  
  // Needs at least 3 x and y values in input
  SplineDef(PrevPathFromSim const& sim_prev);
  
  // Create 2 points in front of the car. The 1st new point is the ref.
  SplineDef(LocalizationData const& ego);

  // add 3 more points in the distance for a smooth spine
  void Extend(int target_lane, const Map& map,
              double ref_x, double ref_y, double ref_yaw);

  vector<double> xs;
  vector<double> ys;
};

#endif  // SPLINE_H
