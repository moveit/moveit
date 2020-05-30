/*******************************************************************************
 *      Title     : nodelet_example.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 05/30/2020
 *      Author    : Tyler Weaver
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Los Alamos National Security, LLC
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

#include <moveit_jog_arm/status_codes.h>
#include <moveit_jog_arm/make_shared_from_pool.h>

namespace moveit_jog_arm
{
constexpr size_t ROS_QUEUE_SIZE = 10;
constexpr char STATUS_TOPIC[] = "/jog_arm/status";
constexpr char CARTESIAN_COMMAND_TOPIC[] = "/jog_arm/delta_jog_cmds";
constexpr char JOINT_COMMAND_TOPIC[] = "/jog_arm/joint_delta_jog_cmds";
constexpr double SEND_COMMANDS_PERIOD = 10;
constexpr size_t NUM_MESSAGES = 50;

class JogArmNodeletExample : public nodelet::Nodelet
{
private:
  void onInit() override
  {
    nh_ = getMTNodeHandle();

    // Wait for jog arm to be up
    ros::topic::waitForMessage<std_msgs::Int8>(STATUS_TOPIC, nh_);

    // Subscribe to status
    status_sub_ = nh_.subscribe(STATUS_TOPIC, ROS_QUEUE_SIZE, &JogArmNodeletExample::statusCB, this);

    // Create publishers to send commands
    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(CARTESIAN_COMMAND_TOPIC, ROS_QUEUE_SIZE);
    joint_jog_pub_ = nh_.advertise<control_msgs::JointJog>(JOINT_COMMAND_TOPIC, ROS_QUEUE_SIZE);

    // Create periodic job to send some commands
    // This is where you would start whatever you are using to generate the commands
    timer_ = nh_.createTimer(ros::Duration(SEND_COMMANDS_PERIOD), &JogArmNodeletExample::sendCommands, this);
  }

  void sendCommands(const ros::TimerEvent& timer_event)
  {
    // Suppres unused-parameter warning
    (void)(timer_event);

    ros::Rate cmd_rate(100);
    size_t count = 0;

    // Send a few Cartesian velocity commands
    while (ros::ok() && count < NUM_MESSAGES)
    {
      // Make a Cartesian velocity message
      // Messages are sent to jogger as boost::shared_ptr to enable zero-copy message_passing.
      // Because this message is not coppied we should not modify it after we send it.
      auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "panda_link0";
      msg->twist.linear.y = 0.01;
      msg->twist.linear.z = -0.01;

      // Send the message
      twist_stamped_pub_.publish(msg);
      cmd_rate.sleep();
      ++count;
    }

    // Leave plenty of time for the jogger to halt its previous motion.
    // For a faster response, adjust the incoming_command_timeout yaml parameter
    ros::Duration(2).sleep();

    // Send a few joint commands
    count = 0;
    while (ros::ok() && count < NUM_MESSAGES)
    {
      // Make a joint command
      // Messages are sent to jogger as boost::shared_ptr to enable zero-copy message_passing.
      // Because this message is not coppied we should not modify it after we send it.
      auto msg = moveit::util::make_shared_from_pool<control_msgs::JointJog>();
      msg->header.stamp = ros::Time::now();
      msg->joint_names.push_back("panda_link5");
      msg->velocities.push_back(0.2);

      // Send the message
      joint_jog_pub_.publish(msg);
      cmd_rate.sleep();
      ++count;
    }
  }

  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_jog_arm::StatusCode latest_stats = static_cast<moveit_jog_arm::StatusCode>(msg->data);
    if (latest_stats != status_)
    {
      status_ = latest_stats;
      const auto& status_str = moveit_jog_arm::JOG_ARM_STATUS_CODE_MAP.at(status_);
      NODELET_INFO_STREAM("Jogger status: " << status_str);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Publisher twist_stamped_pub_;
  ros::Publisher joint_jog_pub_;
  ros::Publisher pause_pub_;
  ros::Timer timer_;

  moveit_jog_arm::StatusCode status_ = moveit_jog_arm::StatusCode::INVALID;
};

}  // namespace moveit_jog_arm

PLUGINLIB_EXPORT_CLASS(moveit_jog_arm::JogArmNodeletExample, nodelet::Nodelet)
