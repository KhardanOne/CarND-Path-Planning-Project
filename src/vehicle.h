#pragma once
#ifndef VEHICLE_H
#define VEHICLE_H

struct LocalizationData {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  int GetLane() const;
};

#endif //  VEHICLE_H
