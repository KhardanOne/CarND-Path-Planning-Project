#ifndef CONFIG_H
#define CONFIG_H

#include <float.h>

#include <limits>
#include <vector>

/*
 * Config
 * Postfixes:
 * s - seconds
 * mph - miles per hour
 * mps - meters per seconds
 * mpss - meters per second^2
 * mpsss - meters per second^3
 */
namespace cfg {
  enum Verbose {kOff, kImportant, kAll};
  constexpr int    kVerbose = kImportant;
  constexpr bool   kDebug   = false;

  // basic hard parameters
  constexpr double kMphToMps      = 0.44704;
  constexpr double kSimTimeStepS  = 0.02;
  constexpr double kLapLength     = 6945.554;
  constexpr double kInfinite      = DBL_MAX;
  constexpr double kMaxSpeedMph   = 49.8;
  constexpr double kMaxAccel      = 9.8;
  constexpr double kMaxDecel      = 9.8;
  constexpr double kMaxJerk       = 9.8;
  constexpr double kCarLength     = 5.0;
  constexpr int    kLaneCount     = 3;
  constexpr double kLaneWidth     = 4.0;

  // basic tuning parameters
  constexpr double kTrajectoryLengthSec     = 1.0;    // determines the number of nodes in trajectory
  constexpr double kTrajectoryMinLengthSec  = 0.2;    // determines the amount of nodes kept from previous frame
  constexpr double kPreferredSpeedMph       = 49.45;
  constexpr double kPreferredAccel          = 3.0;
  constexpr double kPreferredDecel          = 6.0;
  constexpr double kKeepLaneAboveFreeDist   = 200.0;
  constexpr double kBufferDist              = 14.0;   // Distance to follow the forward car from, car center to car center
  constexpr double kLaneWindowHalfLength    = 20.0;   // space required for lane change
  constexpr double kLaneChangeDuration      = 6.0;    // the window must be open for this long, will be sampled every second
  constexpr double kLaneCenterOffsetLimit   = 0.4;    // car is considered "inside" lane if it is closer than this:
  constexpr double kTakeOverSpeedDiff       = 4.0;    // if take over requires a lane-change, then don't take-over with big speed difference

  // calculated values
  constexpr double kMaxSpeed = kMaxSpeedMph * kMphToMps;
  constexpr double kPreferredSpeed = kPreferredSpeedMph * kMphToMps;
  constexpr double kMaxSpeedDistPerFrame = kMaxSpeed * kSimTimeStepS;
  constexpr double kPreferredDistPerFrame = kPreferredSpeed * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameIncrement = kPreferredAccel * kSimTimeStepS * kSimTimeStepS;
  constexpr double kPreferredDistPerFrameDecrement = kPreferredDecel * kSimTimeStepS * kSimTimeStepS;
  constexpr double kMaxDistPerFrameDecrement = kMaxDecel * kSimTimeStepS * kSimTimeStepS; // TODO: !!! check these squared values
  constexpr int    kTrajectoryNodeCount = int(kTrajectoryLengthSec / kSimTimeStepS);
  constexpr int    kTrajectoryMinNodeCount = int(kTrajectoryMinLengthSec / kSimTimeStepS);
  constexpr double kHalfLaneWidth = kLaneWidth / 2.0;  
};

#endif  // CONFIG_H
