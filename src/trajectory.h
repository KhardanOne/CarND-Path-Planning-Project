#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "ego_car.h"
#include "helpers.h"
#include "map.h"
#include "pid.h"
#include "tk_spline.h"

using Spline = tk::spline;

class TrajectoryBuilder {
 public:
  
   /*
   * Constructor
   * @param map used for frenet conversion, back and forth
   * @param ego the own car positional data received from the sim
   * @param prev_path path vectors not yet consumet, received from the sim
   * @param force_restart == true causes trajectory to be generated without
   *  taking previous trajectory into consideration; otherwise it is 
   *  determined automatically whether or not to restart.
   */
  TrajectoryBuilder(Map const& map,
                    EgoCar const& ego,
                    PrevPathFromSim const& sim_prev,
                    bool force_restart = false);
  Spline DefineSpline(int target_lane) const;
  size_t NumNodesToKeep(bool force_restart) const;
  
  /*
   * Creates trajectory in form of two vectors, one for x and one for y coords.
   * @param out_x_vals OUTPUT vector of x coordinates
   * @param out_y_vals OUTPUT vector of y coordinates
   * @param target_lane lane index, where the leftmost is 0, rightmost is 2
   * @param front_car_dist the Distance in Frenet coordinate system in meters
   * @param front_car_speed speed of the car in m/s
   * @returns the number of nodes added
   */
  size_t Create(std::vector<double>& out_x_vals,
                std::vector<double>& out_y_vals,
                int target_lane,
                double front_car_dist,
                double front_car_speed);

 protected:
  /*
   * Checks if:
   * - prev path is at least 3 nodes long
   * - there are no duplicate nodes (their dist > epsilon)
   */
  bool CanContinuePrevPath() const;
  
  /*
   * Clears and reserves memory for the output vectors.
   */
  void InitOutput(std::vector<double>& out_x_vals,
                  std::vector<double>& out_y_vals) const;
  /*
   * Copies a part of the previous nodes from sim_prev_
   * to out_x_vals and out_y_vals.
   * Returns the number of nodes copied.
   */
  size_t CopyPrevious(std::vector<double>& out_x_vals,
                      std::vector<double>& out_y_vals) const;

  /* 
   * Transform a single x,y coordinate back to the map coord-sys.
   * from the coordinate system defined by ref_x_, ref_y_ and ref_yaw_.
   */
  std::vector<double> TransformCoordFromRef(double x, double y) const;

  /*
   * Transform all coordinates from [x|y]_in_out_vals to the
   * coordinate system defined by ref_x_, ref_y_ and ref_yaw_.
   */
  void TransformCoordsIntoRefSys(std::vector<double>& x_in_out_vals,
                                 std::vector<double>& y_in_out_vals) const;
  /*
   * Finds the last point, where we should move forward from.
   * It finds it among the simulator's previous nodes if exists,
   * otherwise it is the car's pose.
   */
  void SetReferencePose();

 private:
  Map const& map_;
  EgoCar const& ego_;
  PrevPathFromSim const& sim_prev_;
  size_t kept_prev_nodes_count_;

  // The last previous node. Continue the trajectory from here.
  // Coordinate transformation is also done to/from ref system.
  double ref_x_ = -1.0;
  double ref_y_ = -1.0;
  double ref_yaw_ = -1.0;
  double ref_speed_ = 0.0;
};

#endif  // TRAJECTORY_H
