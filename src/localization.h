#ifndef VEHICLE_H
#define VEHICLE_H

#include "helpers.h"

struct LocalizationData {
  inline int GetLane() const { return DToLane(d); }
  double x;
  double y;
  double s;
  double d;
  
  // radians
  double yaw;
  
  // meters per second
  double speed;
};
  
#endif  // VEHICLE_H
