#include "sensor_fusion.h"
#include "helpers.h"
#include "map.h"
#include "config.h"
#include <vector>
#include <iostream>
using std::vector;
using std::min;

SensorFusion::SensorFusion(vector<vector<double>> const & input, double lap_length)
: lanes(3, vector<int>()), max_s(lap_length) {

  int debug_wrong_input = 0;

  for (size_t c = 0; c < input.size(); ++c) {
    vector<double> const & raw = input[c];
    int lane = DToLane(raw[SF::D]);
    if (lane > -1) {
      SFCar car{ raw, lane };
      cars.push_back(car);
      lanes[lane].push_back(int(raw[SF::ID]));
    }
    else {
      ++debug_wrong_input;
    }
  }

  if (debug_wrong_input > 0) {
    std::cout << "WARNING: " << debug_wrong_input << " cars have wrong d value" << std::endl;
  }
}

int SensorFusion::GetCarInFront(double ego_s, int lane) {
  double min_dist = CFG::infinite;
  int id = -1;
  for (int c = 0; c < lanes[lane].size(); ++c) {
    double target_s = cars[c].raw[SF::S];
    target_s = (target_s < ego_s) ? target_s : target_s+max_s; //  handle lap restarts
    min_dist = min(min_dist, target_s-ego_s);
    id = lanes[lane][c];
  }
  return id;
}
