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

// calculates center d value from lane number
double LaneToD(int lane);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate the power of two of distance between two points. Faster than distance().
double distance2(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const Map & map);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const Map & map);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const Map & map);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const Map & map);

#endif  // HELPERS_H