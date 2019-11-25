/*
     Title     : jog_arm_data.h
     Project   : moveit_jog_arm
     Created   : 1/11/2019
     Author    : Brian O'Neil, Andy Zelenak, Blake Anderson

BSD 3-Clause License

Copyright (c) 2019, Los Alamos National Security, LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <trajectory_msgs/JointTrajectory.h>

namespace moveit_jog_arm
{
static const std::string LOGNAME = "jog_server";
static const double WHILE_LOOP_WAIT = 0.001;

// Variables to share between threads, and their mutexes
struct JogArmShared
{
  geometry_msgs::TwistStamped command_deltas;

  control_msgs::JointJog joint_command_deltas;

  sensor_msgs::JointState joints;

  double collision_velocity_scale = 1;

  // Indicates that an incoming Cartesian command is all zero velocities
  bool zero_cartesian_cmd_flag = true;

  // Indicates that an incoming joint angle command is all zero velocities
  bool zero_joint_cmd_flag = true;

  // Indicates that we have not received a new command in some time
  bool command_is_stale = false;

  // The new command which is calculated
  trajectory_msgs::JointTrajectory outgoing_command;

  // Timestamp of incoming commands
  ros::Time latest_nonzero_cmd_stamp = ros::Time(0.);

  // Indicates no collision, etc, so outgoing commands can be sent
  bool ok_to_publish = false;
};

// ROS params to be read. See the yaml file in /config for a description of each.
struct JogArmParameters
{
  std::string move_group_name, joint_topic, cartesian_command_in_topic, command_frame, command_out_topic,
      planning_frame, warning_topic, joint_command_in_topic, command_in_type, command_out_type;
  double linear_scale, rotational_scale, joint_scale, lower_singularity_threshold, hard_stop_singularity_threshold,
      collision_proximity_threshold, low_pass_filter_coeff, publish_period, incoming_command_timeout,
      joint_limit_margin, collision_check_rate;
  int num_halt_msgs_to_publish;
  bool use_gazebo, check_collisions, publish_joint_positions, publish_joint_velocities, publish_joint_accelerations;
};
}
