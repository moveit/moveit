/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Tyler Weaver
   Desc:   Test for the low latency mode of servo
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Servo
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>

static const std::string LOGNAME = "servo_low_latency_test";

namespace moveit_servo
{
class ServoFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    // Wait for several key topics / parameters
    ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    while (!nh_.hasParam("/robot_description") && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Load the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);

    // Create moveit_servo
    servo_ = std::make_shared<Servo>(nh_, planning_scene_monitor_);
  }
  void TearDown() override
  {
  }

protected:
  ros::NodeHandle nh_{ "~" };
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit_servo::ServoPtr servo_;
};  // class ServoFixture

TEST_F(ServoFixture, SendTwistStampedTest)
{
  servo_->start();

  auto parameters = servo_->getParameters();

  // count trajectory messages sent by servo
  size_t received_count = 0;
  boost::function<void(const trajectory_msgs::JointTrajectoryConstPtr&)> traj_callback =
      [&received_count](const trajectory_msgs::JointTrajectoryConstPtr& /*msg*/) { ++received_count; };
  auto traj_sub = nh_.subscribe(parameters.command_out_topic, 1, traj_callback);

  // Create publisher to send servo commands
  auto twist_stamped_pub = nh_.advertise<geometry_msgs::TwistStamped>(parameters.cartesian_command_in_topic, 1);

  constexpr double test_duration = 1.0;
  const double publish_period = parameters.publish_period;
  const size_t num_commands = static_cast<size_t>(test_duration / publish_period);

  // Set the rate differently from the publish period from the parameters to show that
  // the number of outputs is set by the number of commands sent and not the rate they are sent.
  ros::Rate publish_rate(2. / publish_period);

  // Send a few Cartesian velocity commands
  for (size_t i = 0; i < num_commands && ros::ok(); ++i)
  {
    auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "panda_link0";
    msg->twist.angular.y = 1.0;

    // Send the message
    twist_stamped_pub.publish(msg);
    publish_rate.sleep();
  }

  EXPECT_GT(received_count, num_commands - 20);
  EXPECT_GT(received_count, (unsigned)0);
  EXPECT_LT(received_count, num_commands + 20);
  servo_->setPaused(true);
}

TEST_F(ServoFixture, SendJointServoTest)
{
  servo_->start();

  auto parameters = servo_->getParameters();

  // count trajectory messages sent by servo
  size_t received_count = 0;
  boost::function<void(const trajectory_msgs::JointTrajectoryConstPtr&)> traj_callback =
      [&received_count](const trajectory_msgs::JointTrajectoryConstPtr& /*msg*/) { ++received_count; };
  auto traj_sub = nh_.subscribe(parameters.command_out_topic, 1, traj_callback);

  // Create publisher to send servo commands
  auto joint_servo_pub = nh_.advertise<control_msgs::JointJog>(parameters.joint_command_in_topic, 1);

  constexpr double test_duration = 1.0;
  const double publish_period = parameters.publish_period;
  const size_t num_commands = static_cast<size_t>(test_duration / publish_period);

  // Set the rate differently from the publish period from the parameters to show that
  // the number of outputs is set by the number of commands sent and not the rate they are sent.
  ros::Rate publish_rate(2. / publish_period);

  // Send a few joint velocity commands
  for (size_t i = 0; i < num_commands && ros::ok(); ++i)
  {
    auto msg = moveit::util::make_shared_from_pool<control_msgs::JointJog>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "panda_link3";
    msg->velocities.push_back(0.1);

    // Send the message
    joint_servo_pub.publish(msg);
    publish_rate.sleep();
  }

  EXPECT_GT(received_count, num_commands - 20);
  EXPECT_GT(received_count, (unsigned)0);
  EXPECT_LT(received_count, num_commands + 20);
  servo_->setPaused(true);
}
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(8);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
