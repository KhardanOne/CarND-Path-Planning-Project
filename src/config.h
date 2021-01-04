#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <limits>
#include <float.h>
using std::vector;

/*
 * Postfixes:
 * S - seconds
 * Mph - miles per hour
 * Mps - meters per seconds
 * Mpss - meters per seconds squared
 * Mpsss - meters per second^3
 */
namespace CFG {
  enum Verbose {kOff, kImportant, kAll};
  constexpr int    kVerbose = kAll;

  // basic constants used in calculations in this file
  constexpr double kMphToMps = 0.44704;
  constexpr double kSimTimeStepS = 0.02;
  constexpr double kLapLength = 6945.554;
  constexpr double kInfinite = DBL_MAX;

  // tuning parameters
  constexpr double kBufferDist = 10.0; //  Distance to follow the forward car from, car center to car center
  constexpr double kCarLength = 5.0;
  constexpr double kLaneWindowHalfLength = 10.0;  // space required for lane change
  
  // the window must be open for this long, will be sampled every second
  constexpr double kLaneChangeDuration = 2.0;
  constexpr double kTrajectoryLengthS = 2.0;
  constexpr double kTrajectoryMinLengthS = 0.2;
  constexpr double kPreferredSpeedMph = 49.45;
  constexpr double kPreferredSpeedMps = kPreferredSpeedMph * kMphToMps;
  // constexpr double kMaxSpeedMph = 50.0;
  constexpr double kMaxAccelMpss = 9.0;
  constexpr double kPreferredAccelMpss = 5.0;
  constexpr double kPreferredDeccelMpss = 3.0;
  constexpr double kMaxDeccelMpss = 7.0;
  constexpr double kKeepLaneAboveFreeDist = 200.0;
  // constexpr double kMaxJerkMpsss = 9.0;

  constexpr int    kLaneCount = 3;
  constexpr double kLaneWidth = 4.0;

  // calculated values

  // constexpr double kMaxSpeedMps = kMaxSpeedMph * kMphToMps;
  constexpr double kPreferredDistPerFrame = kPreferredSpeedMps * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameIncrement = kPreferredAccelMpss * kSimTimeStepS * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameDecrement = kPreferredDeccelMpss * kSimTimeStepS * kSimTimeStepS;
  constexpr double kMaxDistPerFrameDecrement = kMaxDeccelMpss * kSimTimeStepS * kSimTimeStepS;
  constexpr int    kTrajectoryNodeCount = int(kTrajectoryLengthS / kSimTimeStepS);
  constexpr int    kTrajectoryMinNodeCount = int(kTrajectoryMinLengthS / kSimTimeStepS);
  constexpr double kHalfLaneWidth = kLaneWidth / 2.0;

};

#endif  // CONFIG_H
