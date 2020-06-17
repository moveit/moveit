/*******************************************************************************
 *      Title     : cpp_interface_example.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 11/20/2019
 *      Author    : Andy Zelenak
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

#include <std_msgs/Int8.h>

#include <moveit_jog_arm/jog_arm.h>
#include <moveit_jog_arm/status_codes.h>
#include <moveit_jog_arm/make_shared_from_pool.h>

static const std::string LOGNAME = "cpp_interface_example";

// Class for monitoring status of jog_arm
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_jog_arm::StatusCode latest_status = static_cast<moveit_jog_arm::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_jog_arm::JOG_ARM_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Jogger status: " << status_str);
    }
  }
  moveit_jog_arm::StatusCode status_ = moveit_jog_arm::StatusCode::INVALID;
  ros::Subscriber sub_;
};

/**
 * Instantiate the C++ jogging interface.
 * Send some Cartesian commands, then some joint commands.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Run the jogging C++ interface in a new timer to ensure a constant outgoing message rate.
  moveit_jog_arm::JogArm jog_arm(nh, planning_scene_monitor);
  jog_arm.start();

  // Subscribe to jog_arm status (and log it when it changes)
  StatusMonitor status_monitor(nh, jog_arm.getParameters().status_topic);

  // Create publishers to send jog_arm commands
  auto twist_stamped_pub =
      nh.advertise<geometry_msgs::TwistStamped>(jog_arm.getParameters().cartesian_command_in_topic, 1);
  auto joint_jog_pub = nh.advertise<control_msgs::JointJog>(jog_arm.getParameters().joint_command_in_topic, 1);

  ros::Rate cmd_rate(100);
  uint num_commands = 0;

  // Send a few Cartesian velocity commands
  while (ros::ok() && num_commands < 200)
  {
    // Make a Cartesian velocity message
    // Messages are sent to jogger as boost::shared_ptr to enable zero-copy message_passing.
    // Because this message is not copied we should not modify it after we send it.
    auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.y = 0.01;
    msg->twist.linear.z = -0.01;

    // Send the message
    twist_stamped_pub.publish(msg);
    cmd_rate.sleep();
    ++num_commands;
  }

  // Leave plenty of time for the jogger to halt its previous motion.
  // For a faster response, adjust the incoming_command_timeout yaml parameter
  ros::Duration(2).sleep();

  // Send a few joint commands
  num_commands = 0;
  while (ros::ok() && num_commands < 200)
  {
    // Make a joint command
    // Messages are sent to jogger as boost::shared_ptr to enable zero-copy message_passing.
    // Because this message is not copied we should not modify it after we send it.
    auto msg = moveit::util::make_shared_from_pool<control_msgs::JointJog>();
    msg->header.stamp = ros::Time::now();
    msg->joint_names.push_back("elbow_joint");
    msg->velocities.push_back(0.2);

    // Send the message
    joint_jog_pub.publish(msg);
    cmd_rate.sleep();
    ++num_commands;
  }

  jog_arm.stop();
  return 0;
}
