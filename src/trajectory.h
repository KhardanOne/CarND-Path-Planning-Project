#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "vehicle.h"
#include "map.h"
#include "tk_spline.h"
#include <vector>

using std::vector;

/*
 * Received from the simulator.
 */
struct PrevPathFromSim {
  vector<double> x_vals;
  vector<double> y_vals;
  double end_s;
  double end_d;
};

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

  tk::spline DefineSpline(int target_lane) const;

  /*
   * Creates trajectory in form of two vectors, one for x and one for y coords.
   * @param out_x_vals OUTPUT vector of x coordinates
   * @param out_y_vals OUTPUT vector of y coordinates
   * @param target_lane lane index, where the leftmost is 0, rightmost is 2
   * @param front_car_dist the Distance in Frenet coordinate system in meters
   * @param front_car_speed_mps speed of the car in m/s
   */
  void Create(vector<double>& out_x_vals,
              vector<double>& out_y_vals,
              int target_lane,
              double front_car_dist,
              double front_car_speed_mps);

 protected:
  static size_t NumKeptNodes(PrevPathFromSim const& sim_prev);
  bool CanContinuePrevPath() const;

  void InitOutAndCopy(size_t nodes_to_copy_count,
                      vector<double>& out_x_vals,
                      vector<double>& out_y_vals) const;
  vector<double> TransformCoordsFromRef(double x, double y) const;

  /*
   * Finds the last point, where we should look forward.
   * It finds among the simulator's previous nodes if exist.
   * Otherwise it is the car's position.
   */
  void SetRef();

  struct SplineDef {
    SplineDef() = default;
    SplineDef(PrevPathFromSim const& sim_prev);
    SplineDef(LocalizationData const& ego);

    // add 3 more points in the distance for a smooth spine
    void Extend(int target_lane, const Map& map,
                double ref_x, double ref_y, double ref_yaw);
    void TransformCoordsIntoRefSys(double ref_x, double ref_y, double ref_yaw);

    vector<double> x;
    vector<double> y;
  };

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
