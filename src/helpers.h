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

// calculates lane number from d
int DToLane(double d);

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

// Calculate Distance between two points
double Distance(double x1, double y1, double x2, double y2);

// Calculate the power of two of Distance between two points. Faster than Distance().
double DistancePow2(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const Map & map);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const Map & map);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> GetFrenet(double x, double y, double theta, const Map & map);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> GetXY(double s, double d, const Map & map);

#endif  // HELPERS_H
