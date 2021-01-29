#include "sensor_fusion.h"
#include "helpers.h"
#include "ego_car.h"
#include "map.h"
#include "config.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <stdio.h>

using std::vector;
using std::min;
using std::max;
using std::cout;
using std::endl;

SFCar::SFCar(vector<double> const& raw_values, int lane_num)
  : raw(raw_values),
    lane(lane_num) {}

SensorFusion::SensorFusion(vector<vector<double>> const& input, double lap_length)
    : lanes_(3, vector<int>()), max_s(lap_length) {
  int debug_wrong_input = 0;
  for (size_t c = 0; c < input.size(); ++c) {
    vector<double> const& raw = input[c];
    int lane = DToLane(raw[SF::D]);
    if (lane > -1) {
      SFCar car{raw, lane};
      cars_.push_back(car);
      lanes_[lane].push_back(int(raw[SF::ID]));
    } else {
      ++debug_wrong_input;
    }
  }
  if (debug_wrong_input > 0) {
    cout << "WARNING: " << debug_wrong_input << " cars have wrong d value" << endl;
  }
}

/*
 * Distance calculation with edge cases due to circular map:
 * a.) target car is in front of the ego car
 * s=0       [es=100]        [ts=180]       s=max   --> dist = ts-es
 * b.) target car is behind ego car:
 * s=0       [ts=100]        [es=200]       s=max   --> dist = ts-es+max
 */
int SensorFusion::GetCarInFront(double ego_s, int lane) {
  double min_dist = cfg::kInfinite;
  int result = -1;
  vector<int>& lane_cars = lanes_[lane];
  for (size_t i_in_lane = 0; i_in_lane < lane_cars.size(); ++i_in_lane) {
    int target_idx = lane_cars[i_in_lane];
    double target_s = cars_[target_idx].raw[SF::S];
    double dist = GetDistanceForward(ego_s, target_s);
    if (dist < min_dist) {
      min_dist = dist;
      result = target_idx;
    }
  }
  return result;
}

vector<double> SensorFusion::GetPredictedPos(int car_id, double time) {
  vector<double> const& raw = cars_[car_id].raw;
  const double x = raw[SF::X];
  const double y = raw[SF::Y];
  const double vx = raw[SF::VX];
  const double vy = raw[SF::VY];
  vector<double> result;
  result.push_back(x + time * vx);
  result.push_back(y + time * vy);
  return result;
}

vector<double> SensorFusion::GetPredictedPos(EgoCar const& ego, double time) {
  vector<double> result;
  result.push_back(ego.x + ego.speed * cos(ego.yaw));
  result.push_back(ego.y + ego.speed * sin(ego.yaw));
  return result;
}

double SensorFusion::GetLaneSpeed(double from_s, int lane) {
  const int car_idx = GetCarInFront(from_s, lane);
  if (car_idx == -1) {
    return cfg::kPreferredSpeed;
  } else {
    vector<double> const& raw = cars_[car_idx].raw;
    return Speed(raw[SF::VX], raw[SF::VY]);
  }
}

double SensorFusion::GetPredictedS(EgoCar const& ego, double time) {
  return ego.s + time * ego.speed;
}

bool SensorFusion::IsLaneOpen(int lane, EgoCar const& ego, Map const& map) {
  vector<int> const& lane_cars = lanes_[lane];
  size_t cars_count = lane_cars.size();
  double dist_limit_pow2 = cfg::kLaneWindowHalfLength * cfg::kLaneWindowHalfLength;
  for (double time = 0.0; time <= cfg::kLaneChangeDuration; time += 1.0) {
    vector<double> ego_pred_pos = GetPredictedPos(ego, time);
    for (int car_idx_lane = 0; car_idx_lane < cars_count; ++car_idx_lane) {
      int car_idx_sf = lane_cars[car_idx_lane];
      vector<double> const& raw = cars_[car_idx_sf].raw;
      vector<double> pred_pos = GetPredictedPos(car_idx_sf, time);
      double dist_pow2 = DistancePow2(ego_pred_pos[0], ego_pred_pos[1],
                                      pred_pos[0], pred_pos[1]);
      if (dist_pow2 < dist_limit_pow2)
        return false;
    }
  }
  return true;
}

/*
 * cout format:
 *  ctr:12345.6<<
 *  ctr:12345.6   cur:12345.6<< enough
 *  ctr:12345.6   cur:12345.6<< better
 *  ctr:12345.6<< cur:12345.6   worse
 *  ctr:12345.6   cur:12345.6<< blocked
 *                                    12345.6<< 12345.6<< 12345.6<<
 *  ^0                                ^42                          ^84
 */
int SensorFusion::GetTargetLane(EgoCar const& ego, Map const& map) {
  int center = 1;
  int best = center;
  double center_dist = GetPredictedDistanceBeforeObstructed(center, ego, map);
  double max_free_dist = center_dist;
  int current = DToLane(ego.d);  // the current lane is the fallback option
  bool log = false;
  if (log) {
    cout << std::setw(11) << std::setprecision(2) << "center:" << center_dist;
  }
    
  
  // consider center lane first
  if (max_free_dist > cfg::kKeepLaneAboveFreeDist
      && (current == center || IsLaneOpen(center, ego, map))) {
    if (log)
      cout << "<<" << endl;
    return center;
  }

  // consider current lane next if it is not the center
  // only these two lanes can be valid choices so far  // TODO: add some path finding
  double cur_dist = GetPredictedDistanceBeforeObstructed(current, ego, map);
  vector<double> dists(3, -1.0);  // for logging
  dists[1] = center_dist;
  dists[ego.GetLane()] = cur_dist;
  if (current != center) {
    if (cur_dist > cfg::kKeepLaneAboveFreeDist) {
      if (log)
        cout << "   cur:" << cur_dist << "<< enough" << endl;
      return current;
    } else if (cur_dist > center_dist) {
      if (log)
        cout << "   cur:" << cur_dist << "<< better" << endl;
      return current;
    } else {
      if (IsLaneOpen(center, ego, map)) {
        if (log)
          cout << "<< cur:" << cur_dist << "   worse";
        return center;
      } else {
        if (log)
          cout << "   cur:" << cur_dist << "<< blocked" << endl;
        return current;
      }
    }
  } else {  // current == center but not good enough, check side lanes
    for (int lane = max(current - 1, 0); lane <= min(current + 1, cfg::kLaneCount - 1); lane += 2) {
      if (dists[lane] > -1)  // already has value
        continue;
      double free_dist = GetPredictedDistanceBeforeObstructed(lane, ego, map);
      if (free_dist > max_free_dist && IsLaneOpen(lane, ego, map)) {
        dists[lane] = free_dist;
        best = lane;
        max_free_dist = free_dist;
      }
    }

    if (log) {
      switch (best) {
       case 0:
         cout << "                                   "
           << dists[0] << "<< " << dists[1] << "   " << dists[2] << "   " << endl;
         break;
       case 1:
         cout << "                                   "
           << dists[0] << "   " << dists[1] << "<< " << dists[2] << "   " << endl;
         break;
       case 2:
         cout << "                                   "
           << dists[0] << "   " << dists[1] << "   " << dists[2] << "<< " << endl;
         break;
       default:
         cout << "invalid result:                      :" << std::setw(3) << best;
         break;
      }
    }
    return best;
  }
}

double SensorFusion::GetPredictedDistanceBeforeObstructed(
    int lane,
    EgoCar const& ego,
    Map const& map) {
  double max_free_dist;
  int front_id = GetCarInFront(ego.s, lane);
  if (front_id == -1) {
    max_free_dist = cfg::kInfinite;
  } else {
    double front_s = cars_[front_id].raw[SF::S];
    double distance = GetDistanceForward(ego.s, front_s)
                      - cfg::kBufferDist - cfg::kCarLength;
    if (distance > 0.0) {
      double front_speed = GetLaneSpeed(ego.s, lane);
      double delta_speed = cfg::kPreferredSpeed - front_speed;
      double time_to_catch;
      if (delta_speed > 0.01) {
        time_to_catch = distance / delta_speed;
      } else {
        time_to_catch = cfg::kInfinite;
      }
      max_free_dist = cfg::kPreferredSpeed* time_to_catch;
    } else {
      max_free_dist = 0.0;
    }
  }
  return max_free_dist;
}

/*
 * Prints lane info for debugging purposes
 * Legend:
 *    o   o - this lane would be selected by GetTargetLane()
 *    >   < - current lane
 *    !! !! - blocked lane
 *    123.3 - free distance, considering speeds
 * Examples:
 *    [   324324.344   ]    [    234234234    ]     [   34324.3434234    ]
 *    [!! 324324.344 !!]  o>[!!  234234234  !!]<o   [!! 34324.3434234  !!]
 */
void SensorFusion::PrintLaneChangeInfo(EgoCar const& ego, Map const& map) {
  vector<double> dists;
  dists.push_back(GetPredictedDistanceBeforeObstructed(0, ego, map));
  dists.push_back(GetPredictedDistanceBeforeObstructed(1, ego, map));
  dists.push_back(GetPredictedDistanceBeforeObstructed(2, ego, map));
  vector<bool> opens;
  opens.push_back(IsLaneOpen(0, ego, map));
  opens.push_back(IsLaneOpen(1, ego, map));
  opens.push_back(IsLaneOpen(2, ego, map));
  int lane = DToLane(ego.d);
  int lane_choise = GetTargetLane(ego, map);

  cout << std::setw(5) << int(ego.s) << " "
    << std::setw(7) << fmod(ego.d, cfg::kLaneWidth) - cfg::kHalfLaneWidth
    << std::setw(7);

  for (int i = 0; i < 3; ++i) {
    constexpr int kLength = 40;
    char buffer[kLength];
    char c  = (lane_choise == i) ? 'o' : ' ';
    char ll = (lane == i)        ? '>' : ' ';
    char lr = (lane == i)        ? '<' : ' ';
    char b  = !opens[i]          ? '!' : ' ';

    int r = snprintf(buffer, kLength,
      "     %c%c[%c%c %10g %c%c]%c%c", c, ll, b, b, dists[i], b, b, lr, c);
    if (r < 0 || r > kLength)
      cout << "ERROR in PrintLaneChangeInfo()" << endl;

    cout << buffer;
  }
  cout << endl;
}
