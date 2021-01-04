#include "map.h"
#include "config.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

Map::Map(string const & file_fqn, double lap_length)
    : max_s_(lap_length) {
  std::ifstream in_map(file_fqn.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    waypoints_x.push_back(x);
    waypoints_y.push_back(y);
    waypoints_s.push_back(s);
    waypoints_dx.push_back(d_x);
    waypoints_dy.push_back(d_y);
  }

  if (waypoints_x.size() == 0) {
    std::cout << "ERROR: missing map data. Check your path." << std::endl;
  } else if (CFG::kVerbose >= CFG::kImportant) {
    std::cout << "Map initialized with " << waypoints_x.size() << " elements." << std::endl;
  }
}
