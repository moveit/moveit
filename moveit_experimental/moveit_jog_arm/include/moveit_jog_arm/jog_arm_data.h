/*******************************************************************************
 *      Title     : jog_arm_data.h
 *      Project   : moveit_jog_arm
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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

#pragma once

// Eigen
#include <Eigen/Geometry>

// ROS
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

// moveit_jog_arm
#include "status_codes.h"

namespace moveit_jog_arm
{
// Variables to share between threads
// Be careful to not read-modify-write any of these because they are not
// protected by a mutex.
struct JogArmShared
{
  double collision_velocity_scale = 1;

  // The jog thread communicates the minimum stopping time to the collision check thread via this variable
  double worst_case_stop_time = std::numeric_limits<double>::max();

  // Flag a valid incoming Cartesian command having nonzero velocities
  bool have_nonzero_cartesian_cmd = false;

  // Flag a valid incoming joint angle command having nonzero velocities
  bool have_nonzero_joint_cmd = false;

  // Indicates that we have not received a new command in some time
  bool command_is_stale = false;

  // Indicates no collision, etc, so outgoing commands can be sent
  bool ok_to_publish = false;

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
  bool drift_dimensions[6] = { false, false, false, false, false, false };

  // Status of the jogger. 0 for no warning. The meaning of nonzero values can be seen in status_codes.h
  StatusCode status = NO_WARNING;

  // Pause/unpause jog threads - threads are not paused by default
  bool paused = false;

  // Stop jog loop threads - threads are not stopped by default
  bool stop_requested = false;

  // The dimesions to control. In the command frame. [x, y, z, roll, pitch, yaw]
  bool control_dimensions[6] = { true, true, true, true, true, true };
};

// ROS params to be read. See the yaml file in /config for a description of each.
struct JogArmParameters
{
  std::string move_group_name;
  std::string joint_topic;
  std::string cartesian_command_in_topic;
  std::string robot_link_command_frame;
  std::string command_out_topic;
  std::string planning_frame;
  std::string status_topic;
  std::string joint_command_in_topic;
  std::string command_in_type;
  std::string command_out_type;
  double linear_scale;
  double rotational_scale;
  double joint_scale;
  double lower_singularity_threshold;
  double hard_stop_singularity_threshold;
  double low_pass_filter_coeff;
  double publish_period;
  double incoming_command_timeout;
  double joint_limit_margin;
  int num_outgoing_halt_msgs_to_publish;
  bool use_gazebo;
  bool publish_joint_positions;
  bool publish_joint_velocities;
  bool publish_joint_accelerations;
  // Collision checking
  bool check_collisions;
  std::string collision_check_type;
  double collision_check_rate;
  double scene_collision_proximity_threshold;
  double self_collision_proximity_threshold;
  double collision_distance_safety_factor;
  double min_allowable_collision_distance;
};

}  // namespace moveit_jog_arm
