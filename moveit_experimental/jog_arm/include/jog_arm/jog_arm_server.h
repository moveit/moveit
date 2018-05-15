///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.h
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#ifndef JOG_ARM_SERVER_H
#define JOG_ARM_SERVER_H

#include <Eigen/Eigenvalues>
#include <geometry_msgs/Twist.h>
#include <jog_arm/get_ros_params.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace jog_arm
{
// For jogging calc thread
void* joggingPipeline(void* threadid);

// For collision checking thread
void* collisionCheck(void* threadid);

// Shared variables
geometry_msgs::TwistStamped g_command_deltas;
pthread_mutex_t g_command_deltas_mutex;

sensor_msgs::JointState g_joints;
pthread_mutex_t g_joints_mutex;

trajectory_msgs::JointTrajectory g_new_traj;
pthread_mutex_t g_new_traj_mutex;

bool g_imminent_collision(false);
pthread_mutex_t g_imminent_collision_mutex;

bool g_zero_trajectory_flag(false);
pthread_mutex_t g_zero_trajectory_flagmutex;

// ROS subscriber callbacks
void deltaCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

// ROS params to be read
int readParams(ros::NodeHandle& n);
std::string g_move_group_name, g_joint_topic, g_command_in_topic, g_command_frame, g_command_out_topic,
    g_planning_frame, g_warning_topic;
double g_linear_scale, g_rot_scale, g_singularity_threshold, g_hard_stop_sing_thresh, g_low_pass_filter_coeff,
    g_publish_period, g_incoming_cmd_timeout;
bool g_simulation, g_collision_check;

/**
 * Class LowPassFilter - Filter the joint velocities to avoid jerky motion.
 */
class LowPassFilter
{
public:
  LowPassFilter(double low_pass_filter_coeff);
  double filter(const double& new_msrmt);
  void reset(double data);
  double filter_coeff_ = 10.;

private:
  double prev_msrmts_[3] = { 0., 0., 0. };
  double prev_filtered_msrmts_[2] = { 0., 0. };
};

LowPassFilter::LowPassFilter(double low_pass_filter_coeff)
{
  filter_coeff_ = low_pass_filter_coeff;
}

void LowPassFilter::reset(double data)
{
  prev_msrmts_[0] = data;
  prev_msrmts_[1] = data;
  prev_msrmts_[2] = data;

  prev_filtered_msrmts_[0] = data;
  prev_filtered_msrmts_[1] = data;
}

double LowPassFilter::filter(const double& new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt = (1 / (1 + filter_coeff_ * filter_coeff_ + 1.414 * filter_coeff_)) *
                              (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
                               (filter_coeff_ * filter_coeff_ - 1.414 * filter_coeff_ + 1) * prev_filtered_msrmts_[1] -
                               (-2 * filter_coeff_ * filter_coeff_ + 2) * prev_filtered_msrmts_[0]);

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

/**
 * Class JogCalcs - Perform the Jacobian calculations.
 */
class JogCalcs
{
public:
  JogCalcs(const std::string& move_group_name);

protected:
  ros::NodeHandle nh_;

  moveit::planning_interface::MoveGroupInterface arm_;

  geometry_msgs::TwistStamped cmd_deltas_;

  sensor_msgs::JointState incoming_jts_;

  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  void jogCalcs(const geometry_msgs::TwistStamped& cmd);

  // Parse the incoming joint msg for the joints of our MoveGroup
  void updateJoints();

  Vector6d scaleCommand(const geometry_msgs::TwistStamped& command) const;

  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  bool updateJointVels(sensor_msgs::JointState& output, const Eigen::VectorXd& joint_vels) const;

  double checkConditionNumber(const Eigen::MatrixXd& matrix) const;

  // Reset the data stored in low-pass filters so the trajectory won't jump when
  // jogging is resumed.
  void resetVelocityFilters();

  // Halt the robot
  void halt(trajectory_msgs::JointTrajectory& jt_traj);

  const robot_state::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr kinematic_state_;

  sensor_msgs::JointState jt_state_, orig_jts_;

  tf::TransformListener listener_;

  ros::Time prev_time_;

  double delta_t_;

  std::vector<jog_arm::LowPassFilter> velocity_filters_;
  std::vector<jog_arm::LowPassFilter> position_filters_;

  // Check whether incoming cmds are stale. Pause if so
  ros::Duration time_of_incoming_cmd_;

  ros::Publisher warning_pub_;
};

class CollisionCheck
{
public:
  CollisionCheck(const std::string& move_group_name);

private:
  ros::NodeHandle nh_;

  ros::Publisher warning_pub_;
};

}  // namespace jog_arm

#endif  // JOG_ARM_SERVER_H