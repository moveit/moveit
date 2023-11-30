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

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <ros/ros.h>
#include <vector>

namespace trajectory_processing
{
namespace
{
const std::string LOGNAME = "moveit_trajectory_processing.ruckig_traj_smoothing";
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 100;         // rad/s^3
constexpr double MAX_DURATION_EXTENSION_FACTOR = 50.0;
constexpr double DURATION_EXTENSION_FRACTION = 1.1;
constexpr double OVERSHOOT_CHECK_PERIOD = 0.01;  // sec
}  // namespace

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor, const bool mitigate_overshoot,
                                     const double overshoot_threshold)
{
  if (!validateGroup(trajectory))
  {
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Trajectory does not have enough points to smooth with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  // Kinematic limits (vels/accels/jerks) from RobotModel
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ num_dof };
  if (!getRobotModelBounds(max_velocity_scaling_factor, max_acceleration_scaling_factor, group, ruckig_input))
  {
    ROS_ERROR_NAMED(LOGNAME, "Error while retrieving kinematic limits (vel/accel/jerk) from RobotModel.");
    return false;
  }

  return runRuckig(trajectory, ruckig_input, mitigate_overshoot, overshoot_threshold);
}

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const std::unordered_map<std::string, double>& velocity_limits,
                                     const std::unordered_map<std::string, double>& acceleration_limits,
                                     const std::unordered_map<std::string, double>& jerk_limits,
                                     const bool mitigate_overshoot, const double overshoot_threshold)
{
  if (!validateGroup(trajectory))
  {
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Trajectory does not have enough points to smooth with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  // Set default kinematic limits (vels/accels/jerks)
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ num_dof };
  double max_velocity_scaling_factor = 1.0;
  double max_acceleration_scaling_factor = 1.0;
  if (!getRobotModelBounds(max_velocity_scaling_factor, max_acceleration_scaling_factor, group, ruckig_input))
  {
    ROS_ERROR_NAMED(LOGNAME, "Error while retrieving kinematic limits (vel/accel/jerk) from RobotModel.");
    return false;
  }

  // Check if custom limits were supplied as arguments to overwrite the defaults
  const std::vector<std::string>& vars = group->getVariableNames();
  const unsigned num_joints = group->getVariableCount();
  for (size_t j = 0; j < num_joints; ++j)
  {
    // Velocity
    auto it = velocity_limits.find(vars[j]);
    if (it != velocity_limits.end())
    {
      ruckig_input.max_velocity.at(j) = it->second;
    }
    // Acceleration
    it = acceleration_limits.find(vars[j]);
    if (it != acceleration_limits.end())
    {
      ruckig_input.max_acceleration.at(j) = it->second;
    }
    // Jerk
    it = jerk_limits.find(vars[j]);
    if (it != jerk_limits.end())
    {
      ruckig_input.max_jerk.at(j) = it->second;
    }
  }

  return runRuckig(trajectory, ruckig_input, mitigate_overshoot, overshoot_threshold);
}

std::optional<robot_trajectory::RobotTrajectory>
RuckigSmoothing::runRuckigInBatches(const robot_trajectory::RobotTrajectory& trajectory,
                                    ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input, size_t batch_size)
{
  // We take the batch size as the lesser of 0.1*num_waypoints or batch_size,
  // to keep a balance between run time and time-optimality.
  // TODO(andyz): parameterize as MIN_BATCH_SIZE and BATCH_SCALING_FACTOR or something like that
  const size_t num_waypoints = trajectory.getWayPointCount();
  const size_t temp_batch_size = std::min(size_t(0.1 * num_waypoints), size_t(100));
  // We need at least 2 waypoints
  batch_size = std::max(size_t(2), temp_batch_size);

  size_t batch_start_idx = 0;
  size_t batch_end_idx = batch_size - 1;
  const size_t full_traj_final_idx = num_waypoints - 1;
  // A deep copy is not needed since the waypoints are cleared immediately
  robot_trajectory::RobotTrajectory sub_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, false /* deep copy */);
  robot_trajectory::RobotTrajectory output_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, false /* deep copy */);
  output_trajectory.clear();

  while (batch_end_idx <= full_traj_final_idx)
  {
    sub_trajectory.clear();
    for (size_t waypoint_idx = batch_start_idx; waypoint_idx <= batch_end_idx; ++waypoint_idx)
    {
      sub_trajectory.addSuffixWayPoint(trajectory.getWayPoint(waypoint_idx),
                                       trajectory.getWayPointDurationFromPrevious(waypoint_idx));
    }

    // When starting a new batch, set the last Ruckig output equal to the new, starting robot state
    bool first_point_previously_smoothed = false;
    if (output_trajectory.getWayPointCount() > 0)
    {
      sub_trajectory.addPrefixWayPoint(output_trajectory.getLastWayPoint(), 0);
      first_point_previously_smoothed = true;
    }

    if (!runRuckig(sub_trajectory, ruckig_input))
    {
      return std::nullopt;
    }

    // Skip appending the first waypoint in sub_trajectory if it was smoothed in
    // the previous iteration
    size_t first_new_waypoint = first_point_previously_smoothed ? 1 : 0;

    // Add smoothed waypoints to the output
    for (size_t waypoint_idx = first_new_waypoint; waypoint_idx < sub_trajectory.getWayPointCount(); ++waypoint_idx)
    {
      output_trajectory.addSuffixWayPoint(sub_trajectory.getWayPoint(waypoint_idx),
                                          sub_trajectory.getWayPointDurationFromPrevious(waypoint_idx));
    }

    batch_start_idx += batch_size;
    batch_end_idx += batch_size;
  }

  return std::make_optional<robot_trajectory::RobotTrajectory>(output_trajectory, true /* deep copy */);
}

bool RuckigSmoothing::validateGroup(const robot_trajectory::RobotTrajectory& trajectory)
{
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGNAME, "The planner did not set the group the plan was computed for");
    return false;
  }
  return true;
}

bool RuckigSmoothing::getRobotModelBounds(const double max_velocity_scaling_factor,
                                          const double max_acceleration_scaling_factor,
                                          moveit::core::JointModelGroup const* const group,
                                          ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = group->getVariableCount();
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      ROS_WARN_STREAM_ONCE_NAMED(LOGNAME,
                                 "Joint velocity limits are not defined. Using the default "
                                     << DEFAULT_MAX_VELOCITY
                                     << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      ROS_WARN_STREAM_ONCE_NAMED(
          LOGNAME, "Joint acceleration limits are not defined. Using the default "
                       << DEFAULT_MAX_ACCELERATION
                       << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
    // Read jerk limits from parameters since bounds.jerk_bounded_ was never implemented for MoveIt1
    double jerk_limit = DEFAULT_MAX_JERK;
    std::string jerk_param = "ruckig/" + vars.at(i) + "/jerk_limit";
    if (ros::param::get(jerk_param, jerk_limit))
    {
      ruckig_input.max_jerk.at(i) = jerk_limit;
    }
    else
    {
      ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;
      ROS_WARN_STREAM_NAMED(LOGNAME, "Joint jerk limit for joint " + vars.at(i) + " was not defined. Using the default "
                                         << DEFAULT_MAX_JERK
                                         << " rad/s^3. You can define a jerk limit with parameter " + jerk_param);
    }
  }

  return true;
}

bool RuckigSmoothing::runRuckig(robot_trajectory::RobotTrajectory& trajectory,
                                ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                const bool mitigate_overshoot, const double overshoot_threshold)
{
  const size_t num_waypoints = trajectory.getWayPointCount();
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  const size_t num_dof = group->getVariableCount();
  ruckig::OutputParameter<ruckig::DynamicDOFs> ruckig_output{ num_dof };

  // This lib does not work properly when angles wrap, so we need to unwind the path first
  trajectory.unwind();

  // Initialize the smoother
  ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(num_dof, trajectory.getAverageSegmentDuration());
  initializeRuckigState(*trajectory.getFirstWayPointPtr(), group, ruckig_input);

  // Cache the trajectory in case we need to reset it
  robot_trajectory::RobotTrajectory original_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, true /* deep copy */);

  ruckig::Result ruckig_result;
  double duration_extension_factor = 1;
  bool smoothing_complete = false;
  size_t waypoint_idx = 0;
  while ((duration_extension_factor <= MAX_DURATION_EXTENSION_FACTOR) && !smoothing_complete)
  {
    while (waypoint_idx < num_waypoints - 1)
    {
      moveit::core::RobotStatePtr curr_waypoint = trajectory.getWayPointPtr(waypoint_idx);
      moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);

      getNextRuckigInput(curr_waypoint, next_waypoint, group, ruckig_input);

      // Run Ruckig
      ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::StandardVector> ruckig_trajectory(num_dof);
      ruckig_result = ruckig.calculate(ruckig_input, ruckig_trajectory);

      // Step through the trajectory at the given OVERSHOOT_CHECK_PERIOD and check for overshoot.
      // We will extend the duration to mitigate it.
      bool overshoots = false;
      if (mitigate_overshoot)
      {
        overshoots = checkOvershoot(ruckig_trajectory, num_dof, ruckig_input, overshoot_threshold);
      }

      // The difference between Result::Working and Result::Finished is that Finished can be reached in one
      // Ruckig timestep (constructor parameter). Both are acceptable for trajectories.
      // (The difference is only relevant for streaming mode.)

      // If successful and at the last trajectory segment
      if (!overshoots && (waypoint_idx == num_waypoints - 2) &&
          (ruckig_result == ruckig::Result::Working || ruckig_result == ruckig::Result::Finished))
      {
        trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1, ruckig_trajectory.get_duration());
        smoothing_complete = true;
        break;
      }

      // Extend the trajectory duration if Ruckig could not reach the waypoint successfully
      if (overshoots || (ruckig_result != ruckig::Result::Working && ruckig_result != ruckig::Result::Finished))
      {
        duration_extension_factor *= DURATION_EXTENSION_FRACTION;

        const std::vector<int>& move_group_idx = group->getVariableIndexList();
        extendTrajectoryDuration(duration_extension_factor, waypoint_idx, num_dof, move_group_idx, original_trajectory,
                                 trajectory);

        initializeRuckigState(*trajectory.getFirstWayPointPtr(), group, ruckig_input);
        // Continue the loop from failed segment, but with increased duration extension factor
        break;
      }
      ++waypoint_idx;
    }
  }

  if (duration_extension_factor > MAX_DURATION_EXTENSION_FACTOR)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Ruckig extended the trajectory duration to its maximum and still did not find a solution");
  }

  if (ruckig_result != ruckig::Result::Finished && ruckig_result != ruckig::Result::Working)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Ruckig trajectory smoothing failed. Ruckig error: " << ruckig_result);
    return false;
  }

  return true;
}

void RuckigSmoothing::extendTrajectoryDuration(const double duration_extension_factor, size_t waypoint_idx,
                                               const size_t num_dof, const std::vector<int>& move_group_idx,
                                               const robot_trajectory::RobotTrajectory& original_trajectory,
                                               robot_trajectory::RobotTrajectory& trajectory)
{
  trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1,
                                             duration_extension_factor *
                                                 original_trajectory.getWayPointDurationFromPrevious(waypoint_idx + 1));
  // re-calculate waypoint velocity and acceleration
  auto target_state = trajectory.getWayPointPtr(waypoint_idx + 1);
  const auto prev_state = trajectory.getWayPointPtr(waypoint_idx);

  double timestep = trajectory.getWayPointDurationFromPrevious(waypoint_idx + 1);

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    target_state->setVariableVelocity(move_group_idx.at(joint),
                                      (1 / duration_extension_factor) *
                                          target_state->getVariableVelocity(move_group_idx.at(joint)));

    double prev_velocity = prev_state->getVariableVelocity(move_group_idx.at(joint));
    double curr_velocity = target_state->getVariableVelocity(move_group_idx.at(joint));
    target_state->setVariableAcceleration(move_group_idx.at(joint), (curr_velocity - prev_velocity) / timestep);
  }
}

void RuckigSmoothing::initializeRuckigState(const moveit::core::RobotState& first_waypoint,
                                            const moveit::core::JointModelGroup* joint_group,
                                            ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);

  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = first_waypoint.getVariablePosition(idx.at(i));
    current_velocities_vector.at(i) = first_waypoint.getVariableVelocity(idx.at(i));
    current_accelerations_vector.at(i) = first_waypoint.getVariableAcceleration(idx.at(i));
    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    current_velocities_vector.at(i) =
        std::clamp(current_velocities_vector.at(i), -ruckig_input.max_velocity.at(i), ruckig_input.max_velocity.at(i));
    current_accelerations_vector.at(i) = std::clamp(
        current_accelerations_vector.at(i), -ruckig_input.max_acceleration.at(i), ruckig_input.max_acceleration.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
}

void RuckigSmoothing::getNextRuckigInput(const moveit::core::RobotStateConstPtr& current_waypoint,
                                         const moveit::core::RobotStateConstPtr& next_waypoint,
                                         const moveit::core::JointModelGroup* joint_group,
                                         ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input)
{
  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    ruckig_input.current_position.at(joint) = current_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.current_velocity.at(joint) = current_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.current_acceleration.at(joint) = current_waypoint->getVariableAcceleration(idx.at(joint));

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));

    // Clamp velocities/accelerations in case they exceed the limit due to small numerical errors
    ruckig_input.current_velocity.at(joint) =
        std::clamp(ruckig_input.current_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.current_acceleration.at(joint) =
        std::clamp(ruckig_input.current_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
    ruckig_input.target_velocity.at(joint) =
        std::clamp(ruckig_input.target_velocity.at(joint), -ruckig_input.max_velocity.at(joint),
                   ruckig_input.max_velocity.at(joint));
    ruckig_input.target_acceleration.at(joint) =
        std::clamp(ruckig_input.target_acceleration.at(joint), -ruckig_input.max_acceleration.at(joint),
                   ruckig_input.max_acceleration.at(joint));
  }
}

bool RuckigSmoothing::checkOvershoot(ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::StandardVector>& ruckig_trajectory,
                                     const size_t num_dof, ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                                     const double overshoot_threshold)
{
  // For every timestep
  for (double time_from_start = OVERSHOOT_CHECK_PERIOD; time_from_start < ruckig_trajectory.get_duration();
       time_from_start += OVERSHOOT_CHECK_PERIOD)
  {
    std::vector<double> new_position(num_dof);
    std::vector<double> new_velocity(num_dof);
    std::vector<double> new_acceleration(num_dof);
    ruckig_trajectory.at_time(time_from_start, new_position, new_velocity, new_acceleration);
    // For every joint
    for (size_t joint = 0; joint < num_dof; ++joint)
    {
      // If the sign of the error changed and the threshold difference was exceeded
      double error = new_position[joint] - ruckig_input.target_position.at(joint);
      if (((error / (ruckig_input.current_position.at(joint) - ruckig_input.target_position.at(joint))) < 0) &&
          std::fabs(error) > overshoot_threshold)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace trajectory_processing
