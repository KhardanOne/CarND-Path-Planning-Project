#include "vehicle.h"
#include "config.h"
#include <cmath>
#include <iostream>
using std::abs;

int LocalizationData::GetLane() const {
  for (int i = 0; i < CFG::lane_count; ++i)
    if (abs(this->d - (CFG::half_lane_width + i * CFG::lane_width)) < CFG::half_lane_width)
      return i;
  
  std::cout << "WARGNING: no lane number found. Defaulting to 1." << std::endl;
  return 1;
}
