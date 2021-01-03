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

int SensorFusion::GetCarInFront(double const & ego_s, int lane) {
  double min_dist = CFG::infinite;
  int result = -1;
  vector<int> & current_lane = lanes[lane];

  for (size_t i_in_lane = 0; i_in_lane < current_lane.size(); ++i_in_lane) {
    int target_idx = current_lane[i_in_lane];
    double target_s = cars[target_idx].raw[SF::S];
    target_s = (target_s < ego_s) ? target_s+max_s : target_s; //  handle lap restarts
    double dist = target_s - ego_s;
    if (dist < min_dist) {
      min_dist = dist;
      result = target_idx;
    }
  }
  return result;
}
