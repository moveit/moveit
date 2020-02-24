/*******************************************************************************
 *      Title     : jog_calcs.h
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

#include <atomic>
#include "jog_arm_data.h"
#include "low_pass_filter.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/Bool.h>

namespace moveit_jog_arm
{
class JogCalcs
{
public:
  JogCalcs(const JogArmParameters& parameters, const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr);

  void startMainLoop(JogArmShared& shared_variables, std::mutex& mutex);

  void stopMainLoop();

  void haltOutgoingJogCmds();

  /** \brief Check if the robot state is being updated and the end effector transform is known
   *  @return true if initialized properly
   */
  bool isInitialized();

protected:
  ros::NodeHandle nh_;

  // Loop termination flag
  std::atomic<bool> stop_jog_loop_requested_;

  // Flag that outgoing commands to the robot should not be published
  std::atomic<bool> halt_outgoing_jog_cmds_;

  // Flag that robot state is up to date, end effector transform is known
  std::atomic<bool> is_initialized_;

  sensor_msgs::JointState incoming_joints_;

  bool cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables, std::mutex& mutex);

  bool jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables);

  // Parse the incoming joint msg for the joints of our MoveGroup
  bool updateJoints(std::mutex& mutex, const JogArmShared& shared_variables);

  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  // Suddenly halt for a joint limit or other critical issue.
  // Is handled differently for position vs. velocity control.
  void suddenHalt(trajectory_msgs::JointTrajectory& joint_traj);

  void publishWarning(bool active) const;

  // Scale the delta theta to match joint velocity limits
  bool enforceSRDFJointBounds(trajectory_msgs::JointTrajectory& new_joint_traj);

  // Possibly calculate a velocity scaling factor, due to proximity of
  // singularity and direction of motion
  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& jacobian, const Eigen::MatrixXd& pseudo_inverse);

  /**
   * Slow motion down if close to singularity or collision.
   * @param shared_variables data shared between threads, tells how close we are to collision
   * @param mutex locks shared data
   * @param delta_theta motion command, used in calculating new_joint_tray
   * @param singularity_scale tells how close we are to a singularity
   * @return false if very close to collision or singularity
   */
  bool applyVelocityScaling(const JogArmShared& shared_variables, std::mutex& mutex, Eigen::ArrayXd& delta_theta,
                            double singularity_scale);

  trajectory_msgs::JointTrajectory composeJointTrajMessage(sensor_msgs::JointState& joint_state) const;

  void lowPassFilterPositions(sensor_msgs::JointState& joint_state);

  void calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta);

  /** \brief Convert joint deltas to an outgoing JointTrajectory command.
    * This happens for joint commands and Cartesian commands.
    */
  bool convertDeltasToOutgoingCmd();

  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, unsigned int row_to_remove);

  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;

  sensor_msgs::JointState joint_state_, original_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;
  trajectory_msgs::JointTrajectory outgoing_command_;

  std::vector<LowPassFilter> position_filters_;

  ros::Publisher warning_pub_;

  // Flag that a warning should be published
  bool has_warning_ = false;

  JogArmParameters parameters_;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;

  Eigen::Isometry3d tf_moveit_to_cmd_frame_;

  const int gazebo_redundant_message_count_ = 30;

  uint num_joints_;

  ros::Rate default_sleep_rate_;
};
}  // namespace moveit_jog_arm
