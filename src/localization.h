#ifndef VEHICLE_H
#define VEHICLE_H

#include "helpers.h"

struct LocalizationData {
  inline int GetLane() const { return DToLane(d); }
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed_mph;
};
  
#endif  // VEHICLE_H
