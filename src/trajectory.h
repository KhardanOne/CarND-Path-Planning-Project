#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "localization.h"
#include "map.h"
#include "pid.h"
#include "tk_spline.h"
#include "helpers.h"
#include <vector>

using std::vector;

class TrajectoryBuilder {
 public:
  /*
   * Constructor
   * @param map used for frenet conversion, back and forth
   * @param ego the own car positional data received from the sim
   * @param prev_path path vectors not yet consumet, received from the sim
   */
  TrajectoryBuilder(Map const& map,
                    LocalizationData const& ego,
                    PrevPathFromSim const& sim_prev);

  static bool VerifyIsMonotonic(vector<double> const& xs, vector<double> const& ys,
                                double cur_x, double cur_y);
  static bool AreSpeedAccJerkOk(vector<double> const& xs,
                                vector<double> const& ys,
                                double cur_x,
                                double cur_y,
                                double cur_yaw,
                                double cur_speed_mps);

  tk::spline DefineSpline(int target_lane) const;
  static size_t NumKeptNodes(PrevPathFromSim const& sim_prev);
  /*
   * Creates trajectory in form of two vectors, one for x and one for y coords.
   * @param out_x_vals OUTPUT vector of x coordinates
   * @param out_y_vals OUTPUT vector of y coordinates
   * @param target_lane lane index, where the leftmost is 0, rightmost is 2
   * @param front_car_dist the Distance in Frenet coordinate system in meters
   * @param front_car_speed_mps speed of the car in m/s
   * @returns the number of nodes added
   */
  size_t Create(vector<double>& out_x_vals,
                vector<double>& out_y_vals,
                int target_lane,
                double front_car_dist,
                double front_car_speed_mps);
  /*
   * Returns the distance from the last node from the car.
   * Return value is approximate but precise enough.
   * Calculates a linear distance to the last node, not node by node.
   */
  static double LengthInMeters(vector<double> const& xs, vector<double> const& ys,
                               double cur_x, double cur_y);
  
  /*
   * Returns the speed at the end of the trajectory.
   * Calculated from the distance between the penultimate and last node.
   * Returns ego speed if there are no nodes.
   * Calculates the speed from ego car to the first (0th) node if only one
   * node exists.
   */
  static double GetEndSpeed(vector<double> const& xs, vector<double> const& ys, 
                            double cur_x, double cur_y, double cur_speed);

 protected:
  /*
   * Checks if:
   * - prev path is at least 3 nodes long
   * - there are no duplicate nodes (their dist > epsilon)
   */
  bool CanContinuePrevPath() const;
  
  /*
   * Copies previous nodes from sim_prev_ to out_x_vals and out_y_vals.
   * Returns the number of nodes copied.
   */
  size_t InitOutAndCopy(size_t nodes_to_copy_count,
                        vector<double>& out_x_vals,
                        vector<double>& out_y_vals) const;
  static bool AreSpeedsOk(vector<double> const& xs, vector<double> const& ys,
                          double cur_x, double cur_y);
  static bool AreAccelerationsJerksOk(vector<double> const& xs,
                                      vector<double> const& ys,
                                      double cur_x,
                                      double cur_y,
                                      double cur_yaw,
                                      double cur_speed_mps);
  /*
   * Calculates the acceleration between the current and the target point.
   * Returns the combined acceleration in meters per second^2.
   * @param cur_yaw radians
   * @returns the combined acceleration, tangential acc, normal acc
   */
  static vector<double> Acceleration(double cur_x,
                                     double cur_y,
                                     double cur_yaw,
                                     double cur_speed_mps,
                                     double target_x,
                                     double target_y);
  /* 
   * Transform a single x,y coordinate back to the map coord-sys.
   * from the coordinate system defined by ref_x_, ref_y_ and ref_yaw_.
   */
  vector<double> TransformCoordFromRef(double x, double y) const;

  /*
   * Transform all coordinates from [x|y]_in_out_vals to the
   * coordinate system defined by ref_x_, ref_y_ and ref_yaw_.
   */
  void TransformCoordsIntoRefSys(vector<double>& x_in_out_vals,
                                 vector<double>& y_in_out_vals) const;
  /*
   * Finds the last point, where we should look forward.
   * It finds among the simulator's previous nodes if exist.
   * Otherwise it is the car's position.
   */
  void SetRef();

 private:
  Map const& map_;
  LocalizationData const& ego_;
  PrevPathFromSim const& sim_prev_;

  double ref_x_ = -1.0;  // the last node, continue from here
  double ref_y_ = -1.0;
  double ref_yaw_ = -1.0;
  double ref_displacement_ = 0.0;  // the distance between the last two nodes
};

#endif  // TRAJECTORY_H
