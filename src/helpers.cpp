#include "helpers.h"
#include "config.h"
#include "map.h"
#include <iostream>

double LaneToD(int lane) {
  return CFG::half_lane_width + lane * CFG::lane_width;
}

int DToLane(double d) {
  if (d < CFG::lane_width)
    return 0;
  else if (d > 2.0 * CFG::lane_width)
    return 2;
  else
    return 1;

  //for (int i = 0; i < CFG::lane_count; ++i)
  //  if (abs(d - (CFG::half_lane_width + i * CFG::lane_width)) < CFG::half_lane_width)
  //    return i;

  //// std::cout << "WARGNING: no lane number found. Returning -1." << std::endl;
  //return -1;
}

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double distance2(double x1, double y1, double x2, double y2) {
  return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

int ClosestWaypoint(double x, double y, const Map & map) {
  const vector<double> & maps_x = map.waypoints_x;
  const vector<double> & maps_y = map.waypoints_y;

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const Map & map) {
  int closestWaypoint = ClosestWaypoint(x, y, map);

  const vector<double>& maps_x = map.waypoints_x;
  const vector<double>& maps_y = map.waypoints_y;

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(TWO_PI - angle, angle);

  if (angle > M_PI_2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const Map & map) {
  int next_wp = NextWaypoint(x, y, theta, map);

  const vector<double>& maps_x = map.waypoints_x;
  const vector<double>& maps_y = map.waypoints_y;

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = int(maps_x.size()) - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (long long i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };
}

vector<double> getXY(double s, double d, const Map & map) {
  const vector<double>& maps_x = map.waypoints_x;
  const vector<double>& maps_y = map.waypoints_y;
  const vector<double>& maps_s = map.waypoints_s;
  size_t s_size = maps_s.size();
 
  bool log = false; // (CFG::verbose >= CFG::Verbose::All);

  long long prev_wp = -1;
  s = fmod(s, map.max_s);
  if (log)
    std::cout << "getXY> s=" << s;

  while (s > maps_s[(prev_wp+1) % s_size] && (prev_wp < ((long long)s_size - 1))) {
    ++prev_wp;
  }
  prev_wp %= s_size;

  long long wp2 = (prev_wp + 1) % s_size;
  if (log)
    std::cout << " prew_wp=" << prev_wp << " wp2=" << wp2;

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
    (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
  if (log)
    std::cout << " seg_s=" << seg_s << " seg_x=" << seg_x << " seg_y=" << seg_y;

  double perp_heading = heading - M_PI_2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);
  if (log)
    std::cout << " x=" << x << " y=" << y << std::endl;

  return { x,y };
}

