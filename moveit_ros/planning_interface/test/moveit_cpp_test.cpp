/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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

/* Author: Jafar Abdi
   Desc: Test the MoveItCpp and PlanningComponent interfaces
*/

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// Msgs
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace moveit
{
namespace planning_interface
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/moveit_cpp_test");
    moveit_cpp_ptr = std::make_shared<MoveItCpp>(nh_);
    planning_component_ptr = std::make_shared<PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    jmg_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    planning_scene_monitor = moveit_cpp_ptr->getPlanningSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        true /* load octomap monitor */);

    target_pose1.header.frame_id = "panda_link0";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;

    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.55;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.6;

    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.55;
    target_pose2.position.y = -0.05;
    target_pose2.position.z = 0.8;
  }

protected:
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_ptr;
  PlanningComponentPtr planning_component_ptr;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  moveit::core::RobotModelConstPtr robot_model_ptr;
  const moveit::core::JointModelGroup* jmg_ptr;
  const std::string PLANNING_GROUP = "panda_arm";
  geometry_msgs::PoseStamped target_pose1;
  geometry_msgs::Pose target_pose2;
  geometry_msgs::Pose start_pose;
};

// Test the current and the initial state of the Panda robot
TEST_F(MoveItCppTest, GetCurrentStateTest)
{
  ros::Duration(1.0).sleep();  // Otherwise joint_states will result in an invalid robot state
  auto robot_model = moveit_cpp_ptr->getRobotModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  EXPECT_TRUE(moveit_cpp_ptr->getCurrentState(robot_state, 0.0));
  // Make sure the Panda robot is in "ready" state which is loaded from fake_controller.yaml
  std::vector<double> joints_vals;
  robot_state->copyJointGroupPositions(PLANNING_GROUP, joints_vals);
  EXPECT_NEAR(joints_vals[0], 0.0, 0.001);     // panda_joint1
  EXPECT_NEAR(joints_vals[1], -0.785, 0.001);  // panda_joint2
  EXPECT_NEAR(joints_vals[2], 0.0, 0.001);     // panda_joint3
  EXPECT_NEAR(joints_vals[3], -2.356, 0.001);  // panda_joint4
  EXPECT_NEAR(joints_vals[4], 0.0, 0.001);     // panda_joint5
  EXPECT_NEAR(joints_vals[5], 1.571, 0.001);   // panda_joint6
  EXPECT_NEAR(joints_vals[6], 0.785, 0.001);   // panda_joint7
}

// Test the name of the planning group used by PlanningComponent for the Panda robot
TEST_F(MoveItCppTest, NameOfPlanningGroupTest)
{
  EXPECT_STREQ(planning_component_ptr->getPlanningGroupName().c_str(), "panda_arm");
}

// Test setting the start state of the plan to the current state of the robot
TEST_F(MoveItCppTest, TestSetStartStateToCurrentState)
{
  planning_component_ptr->setStartStateToCurrentState();
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));
  // TODO(JafarAbdi) adding testing to the soln state
}

// Test setting the goal using geometry_msgs::PoseStamped and a robot's link name
TEST_F(MoveItCppTest, TestSetGoalFromPoseStamped)
{
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));
}

// Test setting the plan start state using moveit::core::RobotState
TEST_F(MoveItCppTest, TestSetStartStateFromRobotState)
{
  auto start_state = *(moveit_cpp_ptr->getCurrentState());
  start_state.setFromIK(jmg_ptr, start_pose);

  planning_component_ptr->setStartState(start_state);
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));
}

// Test settting the goal of the plan using a moveit::core::RobotState
TEST_F(MoveItCppTest, TestSetGoalFromRobotState)
{
  auto target_state = *(moveit_cpp_ptr->getCurrentState());

  target_state.setFromIK(jmg_ptr, target_pose2);

  planning_component_ptr->setGoal(target_state);

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));
}

// Test settting the goal of the plan using a moveit::core::RobotState
TEST_F(MoveItCppTest, TestPlanWithOctomap)
{
  sensor_msgs::PointCloud2 point_cloud_msg;
  sensor_msgs::PointCloud2Modifier point_cloud_mod(point_cloud_msg);

  point_cloud_msg.header.frame_id = "world";
  point_cloud_msg.height = 1;
  point_cloud_msg.width = 8;
  point_cloud_msg.is_bigendian = false;
  point_cloud_msg.is_dense = true;

  point_cloud_mod.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                       sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

  for (int i = 0; i < 8; ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    *iter_x = 0.1 * ((i >> 0) & 1);
    *iter_y = 0.1 * ((i >> 1) & 1);
    *iter_z = 0.1 * ((i >> 2) & 1);
  }

  ros::Publisher point_cloud_pub;
  point_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/head_mount_kinect/depth_registered/points", 10);
  for (int i = 0; i < 10; i++)
  {
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_pub.publish(point_cloud_msg);
    ros::WallDuration(0.1).sleep();
  }

  auto target_state = *(moveit_cpp_ptr->getCurrentState());

  target_state.setFromIK(jmg_ptr, target_pose2);

  planning_component_ptr->setGoal(target_state);

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));

  // Clearing the octomap and then checking the collision distance has caused a segfault in the past (fixed in #2104)
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = "manipulator";
  collision_request.distance = true;

  planning_scene_monitor->clearOctomap();

  {
    planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)
        ->getCollisionEnv()
        ->checkRobotCollision(collision_request, collision_result, target_state);
  }
  // No collision with empty octomap
  ASSERT_TRUE(!collision_result.collision);

  target_state.setFromIK(jmg_ptr, target_pose1.pose);

  planning_component_ptr->setGoal(target_state);

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr->plan()));
}
}  // namespace planning_interface
}  // namespace moveit

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "moveit_cpp_test");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return result;
}
