/*******************************************************************************
 *      Title     : spacenav_to_twist.cpp
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

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

#include <moveit_jog_arm/status_codes.h>
#include <moveit_jog_arm/make_shared_from_pool.h>

namespace moveit_jog_arm
{
constexpr size_t ROS_QUEUE_SIZE = 10;
constexpr char JOY_SUB[] = "spacenav/joy";
constexpr char CARTESIAN_COMMAND_TOPIC[] = "/jog_arm/delta_jog_cmds";
constexpr char JOINT_COMMAND_TOPIC[] = "/jog_arm/joint_delta_jog_cmds";

class SpaceNavToTwist : public nodelet::Nodelet
{
private:
  void onInit() override
  {
    nh_ = getNodeHandle();

    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(CARTESIAN_COMMAND_TOPIC, ROS_QUEUE_SIZE);
    joint_jog_pub_ = nh_.advertise<control_msgs::JointJog>(JOINT_COMMAND_TOPIC, ROS_QUEUE_SIZE);
    joy_sub_ = nh_.subscribe(JOY_SUB, ROS_QUEUE_SIZE, &SpaceNavToTwist::joyCallback, this);
  };

  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging with the axes
    auto twist_stamped = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
    twist_stamped->header.stamp = ros::Time::now();
    twist_stamped->twist.linear.x = msg->axes[0];
    twist_stamped->twist.linear.y = msg->axes[1];
    twist_stamped->twist.linear.z = msg->axes[2];

    twist_stamped->twist.angular.x = msg->axes[3];
    twist_stamped->twist.angular.y = msg->axes[4];
    twist_stamped->twist.angular.z = msg->axes[5];

    // Joint jogging with the buttons
    auto joint_jog = moveit::util::make_shared_from_pool<control_msgs::JointJog>();
    // This example is for a UR5.
    joint_jog->joint_names.push_back("shoulder_pan_joint");

    // Button 0: positive on the wrist joint
    // Button 1: negative on the wrist joint
    joint_jog->velocities.push_back(msg->buttons[0] - msg->buttons[1]);
    joint_jog->header.stamp = ros::Time::now();

    twist_stamped_pub_.publish(twist_stamped);
    joint_jog_pub_.publish(joint_jog);
  }

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_stamped_pub_;
  ros::Publisher joint_jog_pub_;
};
}  // namespace moveit_jog_arm

PLUGINLIB_EXPORT_CLASS(moveit_jog_arm::SpaceNavToTwist, nodelet::Nodelet)
