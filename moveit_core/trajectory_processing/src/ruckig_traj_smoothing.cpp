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

/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace trajectory_processing
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_trajectory_processing.ruckig_traj_smoothing");
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3
}  // namespace

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor)
{
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_ERROR(LOGGER, "Trajectory does not have enough points to smooth with Ruckig");
    return false;
  }

  const size_t num_dof = group->getVariableCount();

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // Instantiate the smoother
  double timestep = trajectory.getAverageSegmentDuration();
  ruckig::Ruckig<0> ruckig{ num_dof, timestep };
  ruckig::InputParameter<0> ruckig_input{ num_dof };
  ruckig::OutputParameter<0> ruckig_output{ num_dof };

  // Initialize the smoother
  //  ruckig_input.synchronization = ruckig::Synchronization::Time;
  //  ruckig_input.interface = ruckig::Interface::Velocity;
  const std::vector<int>& idx = group->getVariableIndexList();
  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);
  moveit::core::RobotStatePtr waypoint = trajectory.getFirstWayPointPtr();
  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = waypoint->getVariablePosition(idx.at(i));
    current_velocities_vector.at(i) = waypoint->getVariableVelocity(idx.at(i));
    current_accelerations_vector.at(i) = waypoint->getVariableAcceleration(idx.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;

  // Kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    // TODO(andyz): read this from the joint group if/when jerk limits are added to the JointModel
    ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;

    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
  }

  ruckig::Result ruckig_result;
  for (size_t waypoint_idx = 0; waypoint_idx < num_waypoints - 1; ++waypoint_idx)
  {
    moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);

    getNextCurrentTargetStates(ruckig_input, ruckig_output, next_waypoint, num_dof, idx);

    // Run Ruckig
    ruckig_result = ruckig.update(ruckig_input, ruckig_output);

    // If the requested velocity is too great, a joint can actually "move backward" to give itself more time to
    // accelerate to the target velocity. Iterate and decrease velocities until that behavior is gone.
    bool backward_motion_detected = checkForLaggingMotion(num_dof, ruckig_input, ruckig_output);

    double minimum_velocity_magnitude = 0.01;  // rad/s
    double velocity_magnitude = getTargetVelocityMagnitude(ruckig_input, num_dof);
    while (backward_motion_detected && rclcpp::ok() && (velocity_magnitude > minimum_velocity_magnitude))
    {
      // decrease target velocity
      for (size_t joint = 0; joint < num_dof; ++joint)
      {
        ruckig_input.target_velocity.at(joint) *= 0.9;
        // Propogate the change in velocity to position and acceleration, too.
        // This derives from a first-order approximation, e.g. x_i+1 = x_i + v_i * delta_t
        // and 0.9 * x_i+1 = 0.9 * (x_i + v_i * delta_t)
        ruckig_input.target_position.at(joint) -= (0.9 * 0.9 - 1) * timestep * ruckig_input.target_velocity.at(joint);
        ruckig_input.target_acceleration.at(joint) =
            (ruckig_input.target_velocity.at(joint) - ruckig_output.new_velocity.at(joint)) / timestep;
      }
      velocity_magnitude = getTargetVelocityMagnitude(ruckig_input, num_dof);
      // Run Ruckig
      ruckig_result = ruckig.update(ruckig_input, ruckig_output);
      // check for backward motion
      backward_motion_detected = checkForLaggingMotion(num_dof, ruckig_input, ruckig_output);
    }

    if (ruckig_result != ruckig::Result::Working)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Ruckig trajectory smoothing failed at waypoint " << waypoint_idx);
      RCLCPP_ERROR_STREAM(LOGGER, "Ruckig error: " << ruckig_result);
      return false;
    }

    // Overwrite pos/vel/accel of the target waypoint
    for (size_t joint = 0; joint < num_dof; ++joint)
    {
      next_waypoint->setVariablePosition(idx.at(joint), ruckig_output.new_position.at(joint));
      next_waypoint->setVariableVelocity(idx.at(joint), ruckig_output.new_velocity.at(joint));
      next_waypoint->setVariableAcceleration(idx.at(joint), ruckig_output.new_acceleration.at(joint));
    }
    next_waypoint->update();
  }

  return true;
}

double RuckigSmoothing::getTargetVelocityMagnitude(const ruckig::InputParameter<0>& ruckig_input, size_t num_dof)
{
  double vel_magnitude = 0;
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    vel_magnitude += ruckig_input.target_velocity.at(joint) * ruckig_input.target_velocity.at(joint);
  }
  return sqrt(vel_magnitude);
}

bool RuckigSmoothing::checkForLaggingMotion(const size_t num_dof, const ruckig::InputParameter<0>& ruckig_input,
                                            const ruckig::OutputParameter<0>& ruckig_output)
{
  // Check for backward motion of any joint
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    // This indicates the jerk-limited output lags the target output
    if ((ruckig_output.new_velocity.at(joint) / ruckig_input.target_velocity.at(joint)) < 1)
    {
      return true;
    }
  }
  return false;
}

void RuckigSmoothing::getNextCurrentTargetStates(ruckig::InputParameter<0>& ruckig_input,
                                                 ruckig::OutputParameter<0>& ruckig_output,
                                                 const moveit::core::RobotStatePtr& next_waypoint, size_t num_dof,
                                                 const std::vector<int>& idx)
{
  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    // Feed output from the previous timestep back as input
    ruckig_input.current_position.at(joint) = ruckig_output.new_position.at(joint);
    ruckig_input.current_velocity.at(joint) = ruckig_output.new_velocity.at(joint);
    ruckig_input.current_acceleration.at(joint) = ruckig_output.new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));
  }
}
}  // namespace trajectory_processing
