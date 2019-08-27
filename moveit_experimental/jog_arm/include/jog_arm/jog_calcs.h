///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_calcs.h
//      Project   : jog_arm
//      Created   : 1/11/2019
//      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
//
// BSD 3-Clause License
//
// Copyright (c) 2019, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <jog_arm/jog_arm_data.h>
#include <jog_arm/low_pass_filter.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace jog_arm
{
class JogCalcs
{
public:
  JogCalcs(const JogArmParameters& parameters, JogArmShared& shared_variables, pthread_mutex_t& mutex,
           const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr);

protected:
  ros::NodeHandle nh_;

  sensor_msgs::JointState incoming_jts_;

  bool cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables, pthread_mutex_t& mutex);

  bool jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables);

  // Parse the incoming joint msg for the joints of our MoveGroup
  bool updateJoints();

  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  // Scale the delta theta to match joint velocity limits. Uniform scaling
  void enforceJointVelocityLimits(Eigen::VectorXd& calculated_joint_velocity);

  // Reset the data stored in low-pass filters so the trajectory won't jump when jogging is resumed.
  void resetVelocityFilters();

  // Suddenly halt for a joint limit or other critical issue.
  // Is handled differently for position vs. velocity control.
  void suddenHalt(trajectory_msgs::JointTrajectory& jt_traj);

  void publishWarning(bool active) const;

  bool checkIfJointsWithinURDFBounds(trajectory_msgs::JointTrajectory_<std::allocator<void>>& new_jt_traj);

  // Possibly calculate a velocity scaling factor, due to proximity of
  // singularity and direction of motion
  double decelerateForSingularity(const Eigen::VectorXd& commanded_velocity,
                                  const Eigen::JacobiSVD<Eigen::MatrixXd>& svd);

  // Apply velocity scaling for proximity of collisions and singularities
  bool applyVelocityScaling(JogArmShared& shared_variables, pthread_mutex_t& mutex,
                            trajectory_msgs::JointTrajectory& new_jt_traj, const Eigen::VectorXd& delta_theta,
                            double singularity_scale);

  trajectory_msgs::JointTrajectory composeOutgoingMessage(sensor_msgs::JointState& joint_state) const;

  void lowPassFilterVelocities(const Eigen::VectorXd& joint_vel);

  void lowPassFilterPositions();

  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const;

  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;

  sensor_msgs::JointState jt_state_, original_jt_state_;
  std::map<std::string, std::size_t> jt_state_name_map_;
  trajectory_msgs::JointTrajectory outgoing_command_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<LowPassFilter> velocity_filters_;
  std::vector<LowPassFilter> position_filters_;

  ros::Publisher warning_pub_;

  JogArmParameters parameters_;

  // For jacobian calculations
  Eigen::MatrixXd jacobian_, pseudo_inverse_, matrix_s_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::VectorXd delta_theta_;

  const int gazebo_redundant_message_count_ = 30;

  uint num_joints_;
};
}  // namespace jog_arm
