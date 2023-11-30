/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Re:Build AppliedLogix, LLC
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

/* Author: Andy Zelenak */
/* Description: Time-parameterize a trajectory with Time Optimal Trajectory Generation, then iterate until torque limits are obeyed. */

#pragma once

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace trajectory_processing
{
class IterativeTorqueLimitParameterization
{
public:
  IterativeTorqueLimitParameterization(const double path_tolerance = 0.1, const double resample_dt = 0.1,
                                       const double min_angle_change = 0.001);

  /**
   * \brief Compute a trajectory with waypoints spaced equally in time (according to resample_dt_).
   * Resampling the trajectory doesn't change the start and goal point,
   * and all re-sampled waypoints will be on the path of the original trajectory (within path_tolerance_).
   * However, controller execution is separate from MoveIt and may deviate from the intended path between waypoints.
   * path_tolerance_ is defined in configuration space, so the unit is rad for revolute joints,
   * meters for prismatic joints.
   * \param[in,out] trajectory A path which needs time-parameterization. It's OK if this path has already been
   * time-parameterized; this function will re-time-parameterize it.
   * \param gravity_vector For example, (0, 0, -9.81). Units are m/s^2
   * \param external_link_wrenches Externally-applied wrenches on each link. TODO(andyz): what frame is this in?
   * \param joint_torque_limits Torque limits for each joint in N*m. Should all be >0.
   * \param accel_limit_decrement_factor Typically in the range [0.01-0.1].
   * This affects how fast acceleration limits are decreased while searching for a solution. Time-optimality
   * of the output is accurate to approximately 100*accel_limit_decrement_factor %.
   * For example, if accel_limit_decrement_factor is 0.1, the output should be within 10% of time-optimal.
   * \param max_velocity_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param max_acceleration_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   */
  bool computeTimeStampsWithTorqueLimits(
      robot_trajectory::RobotTrajectory& trajectory, const geometry_msgs::Vector3& gravity_vector,
      const std::vector<geometry_msgs::Wrench>& external_link_wrenches, const std::vector<double>& joint_torque_limits,
      double accel_limit_decrement_factor, const std::unordered_map<std::string, double>& velocity_limits,
      const std::unordered_map<std::string, double>& acceleration_limits, const double max_velocity_scaling_factor,
      const double max_acceleration_scaling_factor) const;

private:
  TimeOptimalTrajectoryGeneration totg_;
};
}  // namespace trajectory_processing
