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

// System
#include <atomic>

// ROS
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/Int8.h>

// moveit_jog_arm
#include "jog_arm_data.h"
#include "low_pass_filter.h"
#include "status_codes.h"

namespace moveit_jog_arm
{
class JogCalcs
{
public:
  JogCalcs(const JogArmParameters& parameters, const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr);

  void startMainLoop(JogArmShared& shared_variables);

  /** \brief Check if the robot state is being updated and the end effector transform is known
   *  @return true if initialized properly
   */
  bool isInitialized();

protected:
  ros::NodeHandle nh_;

  // Flag that robot state is up to date, end effector transform is known
  std::atomic<bool> is_initialized_;

  /** \brief Do jogging calculations for Cartesian twist commands. */
  bool cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables);

  /** \brief Do jogging calculations for direct commands to a joint. */
  bool jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables);

  /** \brief Update the stashed status so it can be retrieved asynchronously */
  void updateCachedStatus(JogArmShared& shared_variables);

  /** \brief Parse the incoming joint msg for the joints of our MoveGroup */
  bool updateJoints(JogArmShared& shared_variables);

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
  void suddenHalt(trajectory_msgs::JointTrajectory& joint_traj);
  void suddenHalt(Eigen::ArrayXd& delta_theta);

  /** \brief Publish the status of the jogger to a ROS topic */
  void publishStatus() const;

  /** \brief  Scale the delta theta to match joint velocity/acceleration limits */
  void enforceSRDFAccelVelLimits(Eigen::ArrayXd& delta_theta);

  /** \brief Avoid overshooting joint limits */
  bool enforceSRDFPositionLimits(trajectory_msgs::JointTrajectory& new_joint_traj);

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& jacobian, const Eigen::MatrixXd& pseudo_inverse);

  /**
   * Slow motion down if close to singularity or collision.
   * @param shared_variables data shared between threads, tells how close we are to collision
   * @param delta_theta motion command, used in calculating new_joint_tray
   * @param singularity_scale tells how close we are to a singularity
   */
  void applyVelocityScaling(JogArmShared& shared_variables, Eigen::ArrayXd& delta_theta, double singularity_scale);

  /** \brief Compose the outgoing JointTrajectory message */
  trajectory_msgs::JointTrajectory composeJointTrajMessage(sensor_msgs::JointState& joint_state) const;

  /** \brief Smooth position commands with a lowpass filter */
  void lowPassFilterPositions(sensor_msgs::JointState& joint_state);

  /** \brief Convert an incremental position command to joint velocity message */
  void calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta);

  /** \brief Convert joint deltas to an outgoing JointTrajectory command.
    * This happens for joint commands and Cartesian commands.
    */
  bool convertDeltasToOutgoingCmd();

  /** \brief Gazebo simulations have very strict message timestamp requirements.
   * Satisfy Gazebo by stuffing multiple messages into one.
   */
  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, unsigned int row_to_remove);

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr kinematic_state_;

  // incoming_joint_state_ is the incoming message. It may contain passive joints or other joints we don't care about.
  // internal_joint_state_ is used in jog calculations. It shouldn't be relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_ except it only contains the joints jog_arm acts on.
  sensor_msgs::JointState incoming_joint_state_, internal_joint_state_, original_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;
  trajectory_msgs::JointTrajectory outgoing_command_;

  std::vector<LowPassFilter> position_filters_;

  ros::Publisher status_pub_;

  StatusCode status_ = NO_WARNING;

  JogArmParameters parameters_;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;
  Eigen::ArrayXd prev_joint_velocity_;

  Eigen::Isometry3d tf_moveit_to_cmd_frame_;

  const int gazebo_redundant_message_count_ = 30;

  uint num_joints_;

  ros::Rate default_sleep_rate_;
};
}  // namespace moveit_jog_arm
