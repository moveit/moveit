///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_ros_interface.h
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Blake Anderson, Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
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

// Server node for arm jogging with MoveIt.

#ifndef JOG_ARM_JOG_ROS_INTERFACE_H
#define JOG_ARM_JOG_ROS_INTERFACE_H

#include <Eigen/Eigenvalues>
#include <control_msgs/JointJog.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <trajectory_msgs/JointTrajectory.h>

namespace jog_arm
{
// Variables to share between threads, and their mutexes
struct JogArmShared
{
  pthread_mutex_t shared_variables_mutex;

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

  // The new trajectory which is calculated
  trajectory_msgs::JointTrajectory new_traj;

  // Timestamp of incoming commands
  ros::Time incoming_cmd_stamp = ros::Time(0.);

  bool ok_to_publish = false;
};

// ROS params to be read. See the yaml file in /config for a description of each.
struct JogArmParameters
{
  std::string move_group_name, joint_topic, cartesian_command_in_topic, command_frame, command_out_topic,
      planning_frame, warning_topic, joint_command_in_topic, command_in_type, command_out_type;
  double linear_scale, rotational_scale, joint_scale, lower_singularity_threshold, hard_stop_singularity_threshold,
      lower_collision_proximity_threshold, hard_stop_collision_proximity_threshold, low_pass_filter_coeff,
      publish_period, publish_delay, incoming_command_timeout, joint_limit_margin, collision_check_rate;
  bool use_gazebo, check_collisions, publish_joint_positions, publish_joint_velocities, publish_joint_accelerations;
};

/**
 * Class JogROSInterface - Instantiated in main(). Handles ROS subs & pubs and creates the worker threads.
 */
class JogROSInterface
{
public:
  JogROSInterface();

private:
  // ROS subscriber callbacks
  void deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void deltaJointCmdCB(const control_msgs::JointJogConstPtr& msg);
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

  bool readParameters(ros::NodeHandle& n);

  // Jogging calculation thread
  static bool startJogCalcThread(const JogArmParameters& parameters, JogArmShared& shared_variables,
                                 const robot_model_loader::RobotModelLoaderPtr model_loader_ptr);

  // Collision checking thread
  static bool startCollisionCheckThread(const JogArmParameters& parameters, JogArmShared& shared_variables,
                                        const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr);

  // Variables to share between threads
  struct JogArmShared shared_variables_;

  robot_model_loader::RobotModelLoaderPtr model_loader_ptr_;

  // Store the parameters that were read from ROS server
  struct JogArmParameters ros_parameters_;
};

/**
 * Class LowPassFilter - Filter the joint velocities to avoid jerky motion.
 * This is a second-order Butterworth low-pass filter.
 * See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
 */
class LowPassFilter
{
public:
  explicit LowPassFilter(double low_pass_filter_coeff);
  double filter(double new_measurement_);
  void reset(double data);

private:
  double previous_measurements_[3] = { 0., 0., 0. };
  double previous_filtered_measurements_[2] = { 0., 0. };
  // Larger filter_coeff-> more smoothing of jog commands, but more lag.
  // Rough plot, with cutoff frequency on the y-axis:
  // https://www.wolframalpha.com/input/?i=plot+arccot(c)
  double filter_coeff_ = 10.;
};

/**
 * Class JogCalcs - Perform the Jacobian calculations.
 */
class JogCalcs
{
public:
  JogCalcs(const JogArmParameters parameters, JogArmShared& shared_variables,
           const robot_model_loader::RobotModelLoaderPtr model_loader_ptr);

protected:
  ros::NodeHandle nh_;

  moveit::planning_interface::MoveGroupInterface move_group_;

  sensor_msgs::JointState incoming_jts_;

  bool cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables);

  bool jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables);

  // Parse the incoming joint msg for the joints of our MoveGroup
  bool updateJoints();

  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  // Reset the data stored in low-pass filters so the trajectory won't jump when jogging is resumed.
  void resetVelocityFilters();

  // Avoid a singularity or other issue. Is handled differently for position vs. velocity control.
  void halt(trajectory_msgs::JointTrajectory& jt_traj);

  void publishWarning(bool active) const;

  bool checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory_<std::allocator<void>>& new_jt_traj);

  // Possibly calculate a velocity scaling factor, due to proximity of
  // singularity and direction of motion
  double decelerateForSingularity(Eigen::MatrixXd jacobian, const Eigen::VectorXd commanded_velocity);

  // Apply velocity scaling for proximity of collisions and singularities
  bool applyVelocityScaling(JogArmShared& shared_variables, trajectory_msgs::JointTrajectory& new_jt_traj,
                            const Eigen::VectorXd& delta_theta, double singularity_scale);

  trajectory_msgs::JointTrajectory composeOutgoingMessage(sensor_msgs::JointState& joint_state,
                                                          const ros::Time& stamp) const;

  void lowPassFilterVelocities(const Eigen::VectorXd& joint_vel);

  void lowPassFilterPositions();

  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const;

  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;

  sensor_msgs::JointState jt_state_, original_jt_state_;
  trajectory_msgs::JointTrajectory new_traj_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<jog_arm::LowPassFilter> velocity_filters_;
  std::vector<jog_arm::LowPassFilter> position_filters_;

  ros::Publisher warning_pub_;

  JogArmParameters parameters_;

  const int gazebo_redundant_message_count_ = 30;
};

class collisionCheckThread
{
public:
  collisionCheckThread(const JogArmParameters parameters, JogArmShared& shared_variables,
                       const robot_model_loader::RobotModelLoaderPtr model_loader_ptr);
};

}  // namespace jog_arm

static const std::string LOGNAME = "jog_arm_server";

#endif  // JOG_ARM_JOG_ROS_INTERFACE_H
