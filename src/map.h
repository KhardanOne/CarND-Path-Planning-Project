#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

struct Map {
  Map(std::string const& file_fqn, double lap_length);

  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;
  double max_s;
};

#endif //  MAP_H
