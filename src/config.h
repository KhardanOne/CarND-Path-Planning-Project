#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <limits>
#include <float.h>

/*
 * Postfixes:
 * s - seconds
 * mph - miles per hour
 * mps - meters per seconds
 * mpss - meters per second^2
 * mpsss - meters per second^3
 */
namespace CFG {
  enum Verbose {kOff, kImportant, kAll};
  constexpr int    kVerbose = kAll;
  constexpr bool   kDebug = false;

  // basic constants used in calculations in this file
  constexpr double kMphToMps = 0.44704;
  constexpr double kSimTimeStepS = 0.02;
  constexpr double kLapLength = 6945.554;
  constexpr double kInfinite = DBL_MAX;
  constexpr double kSpeedHardLimitMph = 49.8;
  constexpr double kAccHardLimit = 9.8;
  constexpr double kJerkHardLimit = 9.8;

  // tuning parameters
  constexpr double kBufferDist = 8.0; //  Distance to follow the forward car from, car center to car center
  constexpr double kCarLength = 5.0;
  constexpr double kLaneWindowHalfLength = 15.0;  // space required for lane change
  
  // the window must be open for this long, will be sampled every second
  constexpr double kLaneChangeDuration = 3.0;
  constexpr double kTrajectoryLengthS = 2.0;
  constexpr double kTrajectoryMinLengthS = 0.4;
  constexpr double kPreferredSpeedMph = 49.45;
  constexpr double kPreferredSpeed = kPreferredSpeedMph * kMphToMps;
  constexpr double kMaxAccel = 9.0;
  constexpr double kPreferredAccel = 4.0;
  constexpr double kPreferredDecel = 3.0;
  constexpr double kMaxDecel = 5.0;
  constexpr double kKeepLaneAboveFreeDist = 200.0;
  // constexpr double kMaxJerk = 9.0;
  
  // car might be changing lanes if it is further from lane center
  constexpr double kLaneCenterOffsetLimit = 1.0;

  constexpr int    kLaneCount = 3;
  constexpr double kLaneWidth = 4.0;

  // calculated values

  // constexpr double kMaxSpeed = kMaxSpeedMph * kMphToMps;
  constexpr double kSpeedHardLimit = kSpeedHardLimitMph * kMphToMps;
  constexpr double kSpeedHardLimitDistPerFrame = kSpeedHardLimit * kSimTimeStepS;
  constexpr double kPreferredDistPerFrame = kPreferredSpeed * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameIncrement = kPreferredAccel * kSimTimeStepS * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameDecrement = kPreferredDecel * kSimTimeStepS * kSimTimeStepS;
  constexpr double kMaxDistPerFrameDecrement = kMaxDecel * kSimTimeStepS * kSimTimeStepS; // TODO: !!! check these squared values
  constexpr int    kTrajectoryNodeCount = int(kTrajectoryLengthS / kSimTimeStepS);
  constexpr int    kTrajectoryMinNodeCount = int(kTrajectoryMinLengthS / kSimTimeStepS);
  constexpr double kHalfLaneWidth = kLaneWidth / 2.0;
  
};

#endif  // CONFIG_H
