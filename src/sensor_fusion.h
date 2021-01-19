#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "map.h"
#include "localization.h"
#include <vector>

using std::vector;

struct SFCar {
  SFCar(vector<double> const& raw, int lane);
  vector<double> const& raw;
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
  SensorFusion(vector<vector<double>> const& input, double lap_length);
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
  vector<double> GetPredictedPos(int car_id, double time);
  vector<double> GetPredictedPos(LocalizationData const& ego, double time);

  double GetPredictedS(LocalizationData const& ego, double time);

  double GetLaneSpeedMps(double from_s, int lane);
  bool IsLaneOpen(int lane, LocalizationData const& ego, Map const& map);
  int GetTargetLane(LocalizationData const& ego, Map const& map);

  /*
   * Calculates the distance that can be driven before the ego car 
   * would catch up the car in front with the given speed of the target car.
   * The ego car speed is calculated with the preferred speed.
   */
  double GetPredictedDistanceBeforeObstructed(
      int lane,
      LocalizationData const& ego,
      Map const& map);

  void PrintLaneChangeInfo(LocalizationData const& ego, Map const& map);

  vector<SFCar> cars_;
  vector<vector<int>> lanes_;  // car ids for each lane
  double max_s_;
};

#endif  // SENSOR_FUSION_H
