/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng */

#pragma once

#include <Eigen/Core>
#include <list>
#include <unordered_map>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ruckig/ruckig.hpp>

namespace trajectory_processing
{
class RuckigSmoothing
{
public:
  /**
   * \brief Apply jerk-limited smoothing to a trajectory
   * \param max_velocity_scaling_factor Scale all joint velocity limits by this factor. Usually 1.0.
   * \param max_acceleration_scaling_factor Scale all joint acceleration limits by this factor. Usually 1.0.
   * \param mitigate_overshoot If true, overshoot is mitigated by extending trajectory duration.
   * \param overshoot_threshold If an overshoot is greater than this, duration is extended (radians, for a single joint)
   * \return true if successful.
   */
  static bool applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                             const double max_velocity_scaling_factor = 1.0,
                             const double max_acceleration_scaling_factor = 1.0, const bool mitigate_overshoot = false,
                             const double overshoot_threshold = 0.01);

  /**
   * \brief Apply jerk-limited smoothing to a trajectory
   * \param velocity_limits Joint names and velocity limits in rad/s
   * \param acceleration_limits Joint names and acceleration limits in rad/s^2
   * \param jerk_limits Joint names and jerk limits in rad/s^3
   * \param mitigate_overshoot If true, overshoot is mitigated by extending trajectory duration.
   * \param overshoot_threshold If an overshoot is greater than this, duration is extended (radians, for a single joint)
   * \return true if successful.
   */
  static bool applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                             const std::unordered_map<std::string, double>& velocity_limits,
                             const std::unordered_map<std::string, double>& acceleration_limits,
                             const std::unordered_map<std::string, double>& jerk_limits,
                             const bool mitigate_overshoot = false, const double overshoot_threshold = 0.01);

private:
  /**
   * \brief A utility function to check if the group is defined.
   * \param trajectory      Trajectory to smooth.
   */
  [[nodiscard]] static bool validateGroup(const robot_trajectory::RobotTrajectory& trajectory);

  /**
   * \brief A utility function to get bounds from a JointModelGroup and save them for Ruckig.
   * \param max_velocity_scaling_factor       Scale all joint velocity limits by this factor. Usually 1.0.
   * \param max_acceleration_scaling_factor      Scale all joint acceleration limits by this factor. Usually 1.0.
   * \param group      The RobotModel and the limits are retrieved from this group.
   * \param[out] ruckig_input     The limits are stored in this ruckig::InputParameter, for use in Ruckig.
   */
  [[nodiscard]] static bool getRobotModelBounds(const double max_velocity_scaling_factor,
                                                const double max_acceleration_scaling_factor,
                                                moveit::core::JointModelGroup const* const group,
                                                ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input);

  /**
   * \brief Feed previous output back as input for next iteration. Get next target state from the next waypoint.
   * \param current_waypoint    The nominal current state
   * \param next_waypoint       The nominal, desired state at the next waypoint
   * \param joint_group         The MoveIt JointModelGroup of interest
   * \param[out] ruckig_input   The Rucking parameters for the next iteration
   */
  static void getNextRuckigInput(const moveit::core::RobotStateConstPtr& current_waypoint,
                                 const moveit::core::RobotStateConstPtr& next_waypoint,
                                 const moveit::core::JointModelGroup* joint_group,
                                 ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input);

  /**
   * \brief Initialize Ruckig position/vel/accel. This initializes ruckig_input and ruckig_output to the same values
   * \param first_waypoint  The Ruckig input/output parameters are initialized to the values at this waypoint
   * \param joint_group     The MoveIt JointModelGroup of interest
   * \param[out] ruckig_input   Input parameters to Ruckig. Initialized here.
   * \param[out] ruckig_output  Output from the Ruckig algorithm. Initialized here.
   */
  static void initializeRuckigState(const moveit::core::RobotState& first_waypoint,
                                    const moveit::core::JointModelGroup* joint_group,
                                    ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input);

  /**
   * \brief Break the `trajectory` parameter into batches of reasonable size (~100), run Ruckig on each of them, then
   * re-combine into a single trajectory again.
   * runRuckig() stretches all input waypoints in time until all kinematic limits are obeyed. This works but it can
   * slow the trajectory more than necessary. It's better to feed in just a few waypoints at once, so that only the
   * waypoints needing it get stretched.
   * Here, break the trajectory waypoints into batches so the output is closer to time-optimal.
   * There is a trade-off between time-optimality of the output trajectory and runtime of the smoothing algorithm.
   * \param[in, out] trajectory      Trajectory to smooth.
   * \param[in, out] ruckig_input    Necessary input for Ruckig smoothing. Contains kinematic limits (vel, accel, jerk)
   * \param[in]      batch_size      Minimum number of waypoints to include within a batch
   */
  static std::optional<robot_trajectory::RobotTrajectory>
  runRuckigInBatches(const robot_trajectory::RobotTrajectory& trajectory,
                     ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input, size_t batch_size = 100);

  /**
   * \brief A utility function to instantiate and run Ruckig for a series of waypoints.
   * \param[in, out] trajectory      Trajectory to smooth.
   * \param[in, out] ruckig_input    Necessary input for Ruckig smoothing. Contains kinematic limits (vel, accel, jerk)
   * \param mitigate_overshoot If true, overshoot is mitigated by extending trajectory duration.
   * \param overshoot_threshold If an overshoot is greater than this, duration is extended (radians, for a single joint)
   */
  [[nodiscard]] static bool runRuckig(robot_trajectory::RobotTrajectory& trajectory,
                                      ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                      const bool mitigate_overshoot = false, const double overshoot_threshold = 0.01);

  /**
   * \brief Extend the duration of every trajectory segment
   * \param[in] duration_extension_factor A number greater than 1. Extend every timestep by this much.
   * \param[in] num_waypoints Number of waypoints in the trajectory.
   * \param[in] num_dof Degrees of freedom in the manipulator.
   * \param[in] move_group_idx For accessing the joints of interest out of the full RobotState.
   * \param[in] original_trajectory Durations are extended based on the data in this original trajectory.
   * \param[in, out] trajectory This trajectory will be returned with modified waypoint durations.
   */
  static void extendTrajectoryDuration(const double duration_extension_factor, size_t num_waypoints,
                                       const size_t num_dof, const std::vector<int>& move_group_idx,
                                       const robot_trajectory::RobotTrajectory& original_trajectory,
                                       robot_trajectory::RobotTrajectory& trajectory);

  /** \brief Check if a trajectory out of Ruckig overshoots the target state */
  static bool checkOvershoot(ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::StandardVector>& ruckig_trajectory,
                             const size_t num_dof, ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                             const double overshoot_threshold);
};
}  // namespace trajectory_processing
