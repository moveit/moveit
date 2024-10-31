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

#include <moveit/dynamics_solver/dynamics_solver.h>
#include "moveit/trajectory_processing/iterative_torque_limit_parameterization.h"

namespace trajectory_processing
{
namespace
{
const std::string LOGNAME = "trajectory_processing.iterative_torque_limit_parameterization";
}

IterativeTorqueLimitParameterization::IterativeTorqueLimitParameterization(const double path_tolerance,
                                                                           const double resample_dt,
                                                                           const double min_angle_change)
  : totg_(path_tolerance, resample_dt, min_angle_change)
{
}

bool IterativeTorqueLimitParameterization::computeTimeStampsWithTorqueLimits(
    robot_trajectory::RobotTrajectory& trajectory, const geometry_msgs::Vector3& gravity_vector,
    const std::vector<geometry_msgs::Wrench>& external_link_wrenches, const std::vector<double>& joint_torque_limits,
    double accel_limit_decrement_factor, const std::unordered_map<std::string, double>& velocity_limits,
    const std::unordered_map<std::string, double>& acceleration_limits, const double max_velocity_scaling_factor,
    const double max_acceleration_scaling_factor) const
{
  // 1. Call computeTimeStamps() to time-parameterize the trajectory with given vel/accel limits.
  // 2. Run forward dynamics to check if torque limits are violated at any waypoint.
  // 3. If a torque limit was violated, decrease the acceleration limit for that joint and go back to Step 1.

  if (trajectory.empty())
    return true;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGNAME, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  // Create a mutable copy of acceleration_limits
  std::unordered_map<std::string, double> mutable_accel_limits = acceleration_limits;

  size_t dof = group->getActiveJointModels().size();

  if (joint_torque_limits.size() != dof)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Joint torque limit vector size (" << joint_torque_limits.size()
                                                                       << ") does not match the DOF of the RobotModel ("
                                                                       << dof << ")");
    return false;
  }

  if (accel_limit_decrement_factor < 0.01 || accel_limit_decrement_factor > 0.2)
  {
    ROS_ERROR_NAMED(LOGNAME, "The accel_limit_decrement_factor is outside the typical range [0.01, 0.2]");
    return false;
  }

  dynamics_solver::DynamicsSolver dynamics_solver(trajectory.getRobotModel(), group->getName(), gravity_vector);

  // Copy the waypoints so we can modify them while iterating
  moveit_msgs::RobotTrajectory original_traj;
  trajectory.getRobotTrajectoryMsg(original_traj);
  moveit::core::RobotState initial_state = trajectory.getFirstWayPoint();

  bool iteration_needed = true;
  size_t num_iterations = 0;
  const size_t max_iterations = 10;

  while (iteration_needed && num_iterations < max_iterations)
  {
    ++num_iterations;
    iteration_needed = false;

    totg_.computeTimeStamps(trajectory, velocity_limits, mutable_accel_limits, max_velocity_scaling_factor,
                            max_acceleration_scaling_factor);

    std::vector<double> joint_positions(dof);
    std::vector<double> joint_velocities(dof);
    std::vector<double> joint_accelerations(dof);
    std::vector<double> joint_torques(dof);

    const std::vector<const moveit::core::JointModel*>& joint_models = group->getActiveJointModels();

    // Check if any torque limits are violated
    for (size_t waypoint_idx = 0; waypoint_idx < trajectory.getWayPointCount(); ++waypoint_idx)
    {
      moveit::core::RobotStatePtr& waypoint = trajectory.getWayPointPtr(waypoint_idx);
      waypoint->copyJointGroupPositions(group->getName(), joint_positions);
      waypoint->copyJointGroupVelocities(group->getName(), joint_velocities);
      waypoint->copyJointGroupAccelerations(group->getName(), joint_accelerations);

      if (!dynamics_solver.getTorques(joint_positions, joint_velocities, joint_accelerations, external_link_wrenches,
                                      joint_torques))
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Dynamics computation failed.");
        return false;
      }

      // For each joint, check if torque exceeds the limit
      for (size_t joint_idx = 0; joint_idx < joint_torque_limits.size(); ++joint_idx)
      {
        if (std::fabs(joint_torques.at(joint_idx)) > joint_torque_limits.at(joint_idx))
        {
          // We can't always just decrease acceleration to decrease joint torque.
          // There are some edge cases where decreasing acceleration could actually increase joint torque. For example,
          // if gravity is accelerating the joint. In that case, the joint would be fighting against gravity more.
          // There is also a small chance that changing acceleration has no effect on joint torque, for example:
          // centripetal acceleration caused by velocity of another joint. This should be uncommon on serial manipulators
          // because their torque limits are high enough to withstand issues like that (or it just wouldn't work at all...)

          // Reset
          waypoint->copyJointGroupAccelerations(group->getName(), joint_accelerations);

          // Check if decreasing acceleration of this joint actually decreases joint torque. Else, increase acceleration.
          double previous_torque = joint_torques.at(joint_idx);
          joint_accelerations.at(joint_idx) *= (1 + accel_limit_decrement_factor);
          if (!dynamics_solver.getTorques(joint_positions, joint_velocities, joint_accelerations,
                                          external_link_wrenches, joint_torques))
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Dynamics computation failed.");
            return false;
          }
          if (std::fabs(joint_torques.at(joint_idx)) < std::fabs(previous_torque))
          {
            mutable_accel_limits.at(joint_models.at(joint_idx)->getName()) *= (1 + accel_limit_decrement_factor);
          }
          else
          {
            mutable_accel_limits.at(joint_models.at(joint_idx)->getName()) *= (1 - accel_limit_decrement_factor);
          }

          mutable_accel_limits.at(joint_models.at(joint_idx)->getName()) *= (1 - accel_limit_decrement_factor);
          iteration_needed = true;
        }
      }  // for each joint
      if (iteration_needed)
      {
        // Start over from the first waypoint
        break;
      }
    }  // for each waypoint

    if (iteration_needed)
    {
      // Reset
      trajectory.setRobotTrajectoryMsg(initial_state, original_traj);
    }
  }  // while (iteration_needed && num_iterations < max_iterations)

  return true;
}
}  // namespace trajectory_processing
