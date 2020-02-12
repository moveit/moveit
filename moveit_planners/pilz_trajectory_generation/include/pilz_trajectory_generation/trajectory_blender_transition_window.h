/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRAJECTORY_BLENDER_TRANSITION_WINDOW_H
#define TRAJECTORY_BLENDER_TRANSITION_WINDOW_H

#include "pilz_trajectory_generation/trajectory_functions.h"
#include "pilz_trajectory_generation/trajectory_blender.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/cartesian_trajectory.h"
#include "pilz_trajectory_generation/cartesian_trajectory_point.h"

namespace pilz {

/**
 * @brief Trajectory blender implementing transition window algorithm
 *
 * See doc/MotionBlendAlgorithmDescription.pdf for a mathematical description of the algorithmn.
 */
class TrajectoryBlenderTransitionWindow : public TrajectoryBlender
{
public:
  TrajectoryBlenderTransitionWindow(const LimitsContainer& planner_limits)
    :TrajectoryBlender::TrajectoryBlender(planner_limits)
  {
  }

  virtual ~TrajectoryBlenderTransitionWindow(){}


  /**
   * @brief Blend two trajectories using transition window. The trajectories have to be equally and uniformly
   * discretized.
   * @param req: following fields need to be filled for a valid request:
   *    - group_name : name of the planning group
   *    - link_name : name of the target link
   *    - first_trajectory: Joint trajectory stops at end point.
   *                        The last point must be the same as the first point of the second trajectory.
   *    - second trajectory: Joint trajectory stops at end point.
   *                         The first point must be the same as the last point of the first trajectory.
   *    - blend_radius: The blend radius determines a sphere with the intersection point of the two trajectories
   *                    as the center. Trajectory blending happens inside of this sphere.
   * @param res: following fields are returned as response by the blend algorithm
   *    - group_name : name of the planning group
   *    - first_trajectory: Part of the first original trajectory which is outside of the blend sphere.
   *    - blend_trajectory: Joint trajectory connecting the first and second trajectories without stop.
   *                        The first waypoint has non-zero time from start.
   *    - second trajectory: Part of the second original trajectory which is outside of the blend sphere.
   *                         The first waypoint has non-zero time from start.
   * error_code: information of failed blend
   * @return true if succeed
   */
  virtual bool blend(const pilz::TrajectoryBlendRequest& req,
                     pilz::TrajectoryBlendResponse& res) override;

private:
  /**
   * @brief validate trajectory blend request
   * @param req
   * @param sampling_time: get the same sampling time of the two input trajectories
   * @param error_code
   * @return
   */
  bool validateRequest(const pilz::TrajectoryBlendRequest& req,
                       double &sampling_time,
                       moveit_msgs::MoveItErrorCodes& error_code) const;
  /**
   * @brief searchBlendPoint
   * @param req: trajectory blend request
   * @param first_interse_index: index of the first point of the first trajectory that is inside the blend sphere
   * @param second_interse_index: index of the last point of the second trajectory that is still inside the blend sphere
   */
  bool searchIntersectionPoints(const pilz::TrajectoryBlendRequest& req,
                                std::size_t& first_interse_index,
                                std::size_t& second_interse_index) const;

  /**
   * @brief Determine how the second trajectory should be aligned with the first trajectory for blend.
   * Let tau_1 be the time of the first trajectory from the first_interse_index to the end and tau_2 the time of the
   * second trajectory from the beginning to the second_interse_index:
   *  - if tau_1 > tau_2:<br>
   *    align the end of the first trajectory with second_interse_index
   *    <pre>
   *    first traj:  |-------------|--------!--------------|
   *    second traj:                        |--------------|-------------------|
   *    blend phase:               |-----------------------|
   *    </pre>
   *  - else:<br>
   *    align the first_interse_index with the beginning of the second trajectory
   *    <pre>
   *    first traj:  |-------------|-----------------------|
   *    second traj:               |-----------------------!----------|-------------------|
   *    blend phase:               |----------------------------------|
   *    </pre>
   *
   * @param req: trajectory blend request
   * @param first_interse_index: index of the intersection point between first trajectory and blend sphere
   * @param second_interse_index: index of the intersection point between second trajectory and blend sphere
   * @param blend_align_index: index on the first trajectory, to which the first point on the second trajectory should
   * be aligned to for motion blend. It is now always same as first_interse_index
   * @param blend_time: time of the motion blend period
   */
  void determineTrajectoryAlignment(const pilz::TrajectoryBlendRequest& req,
                                    std::size_t first_interse_index,
                                    std::size_t second_interse_index,
                                    std::size_t& blend_align_index) const;

  /**
   * @brief blend two trajectories in Cartesian space, result in a MultiDOFJointTrajectory which consists
   * of a list of transforms for the blend phase.
   * @param req
   * @param first_interse_index
   * @param second_interse_index
   * @param blend_begin_index
   * @param sampling_time
   * @param trajectory: the resulting blend trajectory inside the blending sphere
   */
  void blendTrajectoryCartesian(const pilz::TrajectoryBlendRequest& req,
                                const std::size_t first_interse_index,
                                const std::size_t second_interse_index,
                                const std::size_t blend_align_index,
                                double sampling_time,
                                pilz::CartesianTrajectory &trajectory) const;

private: // static members
  // Constant to check for equality of values.
  static constexpr double epsilon = 1e-4;
};

}
#endif // TRAJECTORY_BLENDER_TRANSITION_WINDOW_H
