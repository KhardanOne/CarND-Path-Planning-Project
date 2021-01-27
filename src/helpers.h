#ifndef HELPERS_H
#define HELPERS_H

#include "map.h"
#include <math.h>
#include <string>
#include <vector>

#define TWO_PI        (2 * M_PI)
#define TWO_PI_NEG    (-TWO_PI)

// for convenience
using std::string;
using std::vector;

// received data from the simulator
struct PrevPathFromSim {
  vector<double> x_vals;
  vector<double> y_vals;
  double end_s;
  double end_d;
};

// calculates center d value from lane number
double LaneToD(int lane);

// calculates lane number from d
int DToLane(double d);

// Check if the d coordinate is right in the center of the lane.
// (Otherwise it might be changing lanes.)
bool IsInLaneCenter(double d);
bool IsInLaneCenter(double d, int lane);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string HasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double DegToRad(double x);
double RadToDeg(double x);


// Returns the distance to the s coordinate in forward direction
//  considering that the map is circular.
// @param from_s is the s coordinate from which we are looking forward (e.g. car)
// @param to_s s coordinate we are looking at
// @output distance
double GetDistanceForward(double from_s, double to_s);

// Calculate Distance between two points
double Distance(double x1, double y1, double x2, double y2);

// Calculate the power of two of Distance between two points. Faster than Distance().
double DistancePow2(double x1, double y1, double x2, double y2);

// Calculate the speed from its two speed components
double Speed(double vx, double vy);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const Map & map);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const Map & map);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> GetFrenet(double x, double y, double theta, const Map & map);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> GetXY(double s, double d, const Map & map);

#endif  // HELPERS_H
