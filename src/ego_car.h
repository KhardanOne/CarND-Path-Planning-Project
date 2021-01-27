#ifndef EGO_CAR_H
#define EGO_CAR_H

#include "helpers.h"

struct EgoCar {
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
  
#endif  // EGO_CAR_H
