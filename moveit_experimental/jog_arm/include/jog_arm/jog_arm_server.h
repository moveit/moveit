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

#ifndef JOG_ARM_SERVER_H
#define JOG_ARM_SERVER_H

#include <Eigen/Eigenvalues>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pthread.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace jog_arm
{
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

// ROS params to be read
struct jog_arm_parameters
{
  std::string move_group_name, joint_topic, command_in_topic, command_frame, command_out_topic, planning_frame,
      warning_topic;
  double linear_scale, rotational_scale, singularity_threshold, hard_stop_singularity_threshold, low_pass_filter_coeff,
      publish_period, incoming_command_timeout;
  bool gazebo, collision_check;
} g_parameters;

/**
 * Class jogROSInterface - Instantiated in main(). Handles ROS subs & pubs.
 */
class jogROSInterface
{
public:
  jogROSInterface()
  {
    ros::NodeHandle n;

    // Read ROS parameters, typically from YAML file
    readParameters(n);

    // Crunch the numbers in this thread
    pthread_t joggingThread;
    int rc = pthread_create(&joggingThread, NULL, joggingPipeline, this);

    // Check collisions in this thread
    pthread_t collisionThread;
    rc = pthread_create(&collisionThread, NULL, collisionCheck, this);

    // ROS subscriptions. Share the data with the worker threads
    ros::Subscriber cmd_sub = n.subscribe(jog_arm::g_parameters.command_in_topic, 1, &jogROSInterface::deltaCmdCB, this);
    ros::Subscriber joints_sub = n.subscribe(jog_arm::g_parameters.joint_topic, 1, &jogROSInterface::jointsCB, this);

    // Publish freshly-calculated joints to the robot
    ros::Publisher joint_trajectory_pub =
        n.advertise<trajectory_msgs::JointTrajectory>(jog_arm::g_parameters.command_out_topic, 1);

    ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::g_parameters.joint_topic);
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>(jog_arm::g_parameters.command_in_topic);

    // Wait for jog filters to stablize
    ros::Duration(10 * jog_arm::g_parameters.publish_period).sleep();

    ros::Rate main_rate(1. / jog_arm::g_parameters.publish_period);

    while (ros::ok())
    {
      ros::spinOnce();

      // Send the newest target joints
      pthread_mutex_lock(&jog_arm::g_new_traj_mutex);
      if (jog_arm::g_new_traj.joint_names.size() != 0)
      {
        // Check for stale cmds
        if (ros::Time::now() - jog_arm::g_new_traj.header.stamp <
            ros::Duration(jog_arm::g_parameters.incoming_command_timeout))
        {
          // Skip the jogging publication if all inputs are 0.
          pthread_mutex_lock(&jog_arm::g_zero_trajectory_flagmutex);
          if (!jog_arm::g_zero_trajectory_flag)
          {
            jog_arm::g_new_traj.header.stamp = ros::Time::now();
            joint_trajectory_pub.publish(jog_arm::g_new_traj);
          }
          pthread_mutex_unlock(&jog_arm::g_zero_trajectory_flagmutex);
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(2, "jog_arm_server", "Stale joint "
                                                              "trajectory msg. Try a larger "
                                                              "'incoming_command_timeout' parameter.");
          ROS_WARN_STREAM_THROTTLE_NAMED(2, "jog_arm_server", "Did input from the "
                                                              "controller get interrupted? Are "
                                                              "calculations taking too long?");
        }
      }
      pthread_mutex_unlock(&jog_arm::g_new_traj_mutex);

      main_rate.sleep();
    }

    (void)pthread_join(joggingThread, NULL);
    (void)pthread_join(collisionThread, NULL);
  }

private:
  // ROS subscriber callbacks
  void deltaCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

  int readParameters(ros::NodeHandle& n);

  // Jogging calculation thread
  static void* joggingPipeline(void* thread_id);

  // Collision checking thread
  static void* collisionCheck(void* thread_id);
};

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

  moveit::planning_interface::MoveGroupInterface move_group_;

  geometry_msgs::TwistStamped cmd_deltas_;

  sensor_msgs::JointState incoming_jts_;

  void jogCalcs(const geometry_msgs::TwistStamped& cmd);

  // Parse the incoming joint msg for the joints of our MoveGroup
  void updateJoints();

  Eigen::VectorXd scaleCommand(const geometry_msgs::TwistStamped& command) const;

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