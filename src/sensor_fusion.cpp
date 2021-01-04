#include "sensor_fusion.h"
#include "helpers.h"
#include "vehicle.h"
#include "map.h"
#include "config.h"
#include <vector>
#include <iostream>

using std::vector;
using std::min;
using std::max;

SFCar::SFCar(vector<double> const& raw_values, int lane_num)
  : raw(raw_values),
    lane(lane_num) {}

SensorFusion::SensorFusion(vector<vector<double>> const& input, double lap_length)
    : lanes_(3, vector<int>()), max_s_(lap_length) {
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
    std::cout << "WARNING: " << debug_wrong_input << " cars have wrong d value" << std::endl;
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
  double min_dist = CFG::kInfinite;
  int result = -1;
  vector<int>& current_lane = lanes_[lane];

  for (size_t i_in_lane = 0; i_in_lane < current_lane.size(); ++i_in_lane) {
    int target_idx = current_lane[i_in_lane];
    double target_s = cars_[target_idx].raw[SF::S];
    target_s = (target_s < ego_s) ? target_s + max_s_ : target_s;  // handle lap restarts
    double dist = target_s - ego_s;
    if (dist < min_dist) {
      min_dist = dist;
      result = target_idx;
    }
  }
  return result;
}

vector<double> SensorFusion::GetPredictedPos(int car_id, double time) {
  vector<double> const& raw = cars_[car_id].raw;
  double x = raw[SF::X];
  double y = raw[SF::Y];
  double vx = raw[SF::VX];
  double vy = raw[SF::VY];
  vector<double> result;
  result.push_back(x + time * vx);
  result.push_back(y + time * vy);
  return result;
}

double SensorFusion::GetLaneSpeedMps(double from_s, int lane) {
  int car_idx = GetCarInFront(from_s, lane);
  if (car_idx == -1) {
    return CFG::kPreferredSpeedMps;
  } else {
    vector<double> const& raw = cars_[car_idx].raw;
    return sqrt(raw[SF::VX] * raw[SF::VX] + raw[SF::VY] * raw[SF::VY]);
  }
}

double SensorFusion::GetPredictedS(LocalizationData const& ego, double time) {
  double speed_mps = ego.speed_mph * CFG::kMphToMps;
  return ego.s + time * speed_mps;
}

bool SensorFusion::IsLaneOpen(int lane, LocalizationData const& ego, Map const& map) {
  vector<int> const& lane_vect = lanes_[lane];
  size_t lane_size = lane_vect.size();
  double dist_limit_pow2 = CFG::kLaneWindowHalfLength * CFG::kLaneWindowHalfLength;
  for (double time = 0.0; time <= CFG::kLaneChangeDuration; time += 1.0) {
    double predicted_s = GetPredictedS(ego, time);
    vector<double> target_pos = GetXY(predicted_s, LaneToD(lane), map);
    for (int i_in_lane = 0; i_in_lane < lane_size; ++i_in_lane) {
      int car_idx = lane_vect[i_in_lane];
      vector<double> const& raw = cars_[car_idx].raw;
      vector<double> predicted_pos = GetPredictedPos(car_idx, time);
      double dist_pow2 = DistancePow2(target_pos[0], target_pos[1],
        predicted_pos[0], predicted_pos[1]);
      if (dist_pow2 < dist_limit_pow2)
        return false;
    }
  }
  return true;
}

int SensorFusion::GetTargetLane(LocalizationData const& ego, Map const& map) {
  int center = 1;
  int best = center;
  double max_free_dist = GetPredictedDistanceBeforeObstructed(center, ego, map);
  int current = DToLane(ego.d);  // the current lane is the fallback
  std::cout << current << " free lane distances: center: " << max_free_dist;

  // consider center lane first
  if (max_free_dist > CFG::kKeepLaneAboveFreeDist) {
    std::cout << " it is enough, not considering others. Choise >>> " << center << std::endl;
    return center;
  }

  // consider current lane next if it is not the center
  // only these two lanes are valid choices so far  // TODO: add some path finding
  if (current != center) {
    double dist = GetPredictedDistanceBeforeObstructed(current, ego, map);
    if (dist > CFG::kKeepLaneAboveFreeDist) {
      std::cout << ", current: " << dist << " is enough, not considering others. Choise >>> "
        << current << std::endl;
      return current;
    } else if (dist > max_free_dist) {
      std::cout << ", current: " << dist << " is better than center. Choise >>> "
        << current << std::endl;
      return current;
    } else {
      std::cout << ", current: " << dist << " is worse than center. Choise >>> "
        << center << std::endl;
      return center;
    }
  } else {  // current == center but not good enough, check side lanes
    for (int lane = max(current - 1, 0); lane <= min(current + 1, CFG::kLaneCount - 1); ++lane) {
      if (lane == current || lane == center)
        continue;
      double free_dist = GetPredictedDistanceBeforeObstructed(lane, ego, map);
      std::cout << "   " << lane << ": " << int(free_dist);
      if (free_dist > max_free_dist && IsLaneOpen(lane, ego, map)) {
        best = lane;
        max_free_dist = free_dist;
      }
    }
    std::cout << " best dist: " << max_free_dist << " choice >>> " << best << std::endl;
    return best;
  }
}

double SensorFusion::GetPredictedDistanceBeforeObstructed(
    int lane,
    LocalizationData const& ego,
    Map const& map) {
  int front_id = GetCarInFront(ego.s, lane);
  double max_free_dist;
  if (front_id == -1) {
    max_free_dist = CFG::kInfinite;
  }
  else {
    double front_speed_mps = GetLaneSpeedMps(ego.s, lane);
    double delta_speed_mps = CFG::kPreferredSpeedMps - front_speed_mps;
    double time_to_catch;
    if (delta_speed_mps > 0.01) {
      double front_s = cars_[front_id].raw[SF::S];
      double distance = front_s - ego.s;
      distance = (distance >= 0.0) ? distance : distance + CFG::kLapLength;  // handle lap restarts
      time_to_catch = distance / delta_speed_mps;
    } else {
      time_to_catch = CFG::kInfinite;
    }
    max_free_dist = CFG::kPreferredSpeedMps * time_to_catch;
  }
  return max_free_dist;
}


//int SensorFusion::GetTargetLane(LocalizationData const& ego, Map const& map) {
//  int current = DToLane(ego.d);  // the current lane is the fallback
//  int best = current;
//  double max_speed_mps = GetLaneSpeedMps(ego.s, current);
//  for (int lane = max(current-1, 0); lane <= min(current+1, CFG::kLaneCount-1); ++lane) {
//    if (lane == current)
//      continue;
//    double lane_speed = GetLaneSpeedMps(ego.s, lane);
//    if (lane_speed > max_speed_mps && IsLaneOpen(lane, ego, map)) {
//      best = lane;
//      max_speed_mps = lane_speed;
//    }
//  }
//  return best;
//}
