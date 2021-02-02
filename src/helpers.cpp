#include "helpers.h"

#include <algorithm>
#include <iostream>

#include "config.h"
#include "map.h"

using std::min;
using std::max;
using std::cout;
using std::endl;
using std::string;
using std::vector;

double Crop(double low_limit, double x, double high_limit) {
  return min(high_limit, max(low_limit, x));
}

double LaneToD(int lane) {
  return cfg::kHalfLaneWidth + lane * cfg::kLaneWidth;
}

int DToLane(double d) {
  if (d < cfg::kLaneWidth) {
    return 0;
  } else if (d > 2.0 * cfg::kLaneWidth) {
    return 2;
  } else {
    return 1;
  }
}

bool IsInLaneCenter(double d) {
  double offset = fmod(d - cfg::kHalfLaneWidth, cfg::kLaneWidth);
  return offset <= cfg::kLaneCenterOffsetLimit;
}

bool IsInLaneCenter(double d, int lane) {
  double center = LaneToD(lane);
  return abs(center - d) < cfg::kLaneCenterOffsetLimit;
}

string HasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double DegToRad(double x) { return x * M_PI / 180; }
double RadToDeg(double x) { return x * 180 / M_PI; }

double MphToMps(double x) { return x * 0.44704; }
double MpsToMph(double x) { return x / 0.44704; }

double GetDistanceForward(double from_s, double to_s) {
  double distance = to_s - from_s;
  if (distance < 0.0)
    distance += cfg::kLapLength;  // handle lap restarts
  return distance;
}

bool IsInFront(double ego_s, double target_s) {
  const double dist = GetDistanceForward(ego_s, target_s);
  return dist > 0 && dist < cfg::kLapLength / 2.0;
}

double Distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Speed(double vx, double vy) {
  return sqrt(vx*vx + vy*vy);
}

double DistancePow2(double x1, double y1, double x2, double y2) {
  return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

double AccelDistance(double accel, double cur_speed, double end_speed) {
  // using 4th kinematic formula:
  // v^2 = v0^2 + 2a*dx --> 2a*dx = v^2 - v0^2 --> dx = (v^2-v0^2)/2a
  return (end_speed*end_speed-cur_speed*cur_speed) / 2.0 / accel;
}

int ClosestWaypoint(double x, double y, Map const& map) {
  const vector<double>& maps_x = map.waypoints_x;
  const vector<double>& maps_y = map.waypoints_y;

  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = Distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, Map const& map) {
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

vector<double> GetFrenet(double x, double y, double theta, Map const& map) {
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

  double frenet_d = Distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = Distance(center_x, center_y, x_x, x_y);
  double centerToRef = Distance(center_x, center_y, proj_x, proj_y);
  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (long long i = 0; i < prev_wp; ++i) {
    frenet_s += Distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  frenet_s += Distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };
}

vector<double> GetXY(double s, double d, Map const& map) {
  const vector<double>& maps_x = map.waypoints_x;
  const vector<double>& maps_y = map.waypoints_y;
  const vector<double>& maps_s = map.waypoints_s;
  size_t s_size = maps_s.size();
 
  bool log = true;
  static long long log_prev_wp = 0;

  long long prev_wp = -1;
  s = fmod(s, map.max_s);
  //if (log)
  //  cout << "GetXY: s=" << s;

  while (s >= maps_s[(prev_wp+1) % s_size] && (prev_wp < ((long long)s_size - 1))) {
    ++prev_wp;
  }
  prev_wp %= s_size;

  long long wp2 = (prev_wp + 1) % s_size;
  //if (log && (wp2 < 2 || prev_wp >= 179))
  //  cout << " prew_wp=" << prev_wp << " wp2=" << wp2;

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);
  if (seg_s < 0.0) {
    if (log && (wp2 < 2 || prev_wp >= 179)) {
      cout << "info: GetXY() seg_s is negative, incrementing from " << seg_s
        << " to " << seg_s + map.max_s << endl;
    }
    seg_s += map.max_s;
  }

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
  //if (log && wp2 != log_prev_wp && (wp2 == 180 || wp2 == 0)) {
  //  cout << s << "GetXY(): wp2:" << wp2 << endl;
  //  log_prev_wp = wp2;
  //}
  //if (log && (wp2 < 2 || prev_wp >= 179))
  //  cout << " seg_s=" << seg_s << " seg_x=" << seg_x << " seg_y=" << seg_y << " heading: " << heading;

  double perp_heading = heading - M_PI_2;
  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);
  //if (log && (wp2 < 2 || prev_wp >= 179))
  //  cout << " x=" << x << " y=" << y << endl;

  return { x, y };
}
