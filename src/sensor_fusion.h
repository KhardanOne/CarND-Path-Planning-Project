#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "map.h"
#include "ego_car.h"
#include <vector>


struct SFCar {
  SFCar(std::vector<double> const& raw, int lane);

  std::vector<double> const& raw;
  int lane = -1;
};


/*
 * Keys for SensorFusion vectors of 7 elements.
 */
struct SF {
  enum Keys {ID, X, Y, VX, VY, S, D};
};


class SensorFusion {
 public:
  SensorFusion(std::vector<std::vector<double>> const& input, double lap_length);
  virtual ~SensorFusion() = default;

  /*
   * Returns the ID of the closest car in front of the ego car
   * in the given lane.
   * @param ego_s the s coord of the own car
   * @param lane consider cars only in this lane
   * @output the ID of the closes car in front or -1 if the lane is empty
   */
  int GetCarInFront(double ego_s, int lane);

  /*
   * Returns the x, y coordinates of a car.
   * It predicts a linear trajectory.
   * @param car_id which car to predict for
   * @param time how much time in the future to look at (delta T) 
   * @output vector<double> where [0] is x, [1] is y
   */
  std::vector<double> GetPredictedPos(int car_id, double time);
  std::vector<double> GetPredictedPos(EgoCar const& ego, double time);

  double GetPredictedS(EgoCar const& ego, double time);

  double GetLaneSpeedMps(double from_s, int lane);
  bool IsLaneOpen(int lane, EgoCar const& ego, Map const& map);
  int GetTargetLane(EgoCar const& ego, Map const& map);

  /*
   * Calculates the distance that can be driven before the ego car 
   * would catch up the car in front.
   * The ego car speed is calculated with its preferred speed.
   */
  double GetPredictedDistanceBeforeObstructed(
      int lane,
      EgoCar const& ego,
      Map const& map);

  void PrintLaneChangeInfo(EgoCar const& ego, Map const& map);

  std::vector<SFCar> cars_;
  std::vector<std::vector<int>> lanes_;  // car ids for each lane
  double max_s;
};

#endif  // SENSOR_FUSION_H
