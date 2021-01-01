#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
using std::vector;
using std::string;

struct Map {
  Map(string const & file_fqn, double lap_length);

  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
  double max_s;
};

#endif //  MAP_H
