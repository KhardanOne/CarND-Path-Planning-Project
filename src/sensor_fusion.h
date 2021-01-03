#pragma once
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "map.h"
#include <vector>
using std::vector;

struct SFCar {
  vector<double> const & raw;
  int lane = -1;
};

struct SF {
  enum Keys { ID, X, Y, VX, VY, S, D };
};

class SensorFusion {
 public:

  SensorFusion(vector<vector<double>> const & input, double lap_length);
  virtual ~SensorFusion() = default;

  /*
   * Returns the ID of the closest car in front of the ego car
   * in the given lane.
   * @param ego_s the s coord of the own car
   * @param lane consider cars only in this lane
   * @output the ID of the closes car in front or -1 if the lane is empty
   */
  int GetCarInFront(double ego_s, int lane);

  vector<SFCar> cars;
  vector<vector<int>> lanes;  // car ids for each lane
  double max_s;
};

#endif //  SENSOR_FUSION_H
