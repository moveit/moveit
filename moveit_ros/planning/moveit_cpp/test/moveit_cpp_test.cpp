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
// Msgs
#include <geometry_msgs/PointStamped.h>

namespace moveit_cpp
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/moveit_cpp_test");
    moveit_cpp_ptr_ = std::make_shared<MoveItCpp>(nh_);
    planning_component_ptr_ = std::make_shared<PlanningComponent>(PLANNING_GROUP_, moveit_cpp_ptr_);
    robot_model_ptr_ = moveit_cpp_ptr_->getRobotModel();
    jmg_ptr_ = robot_model_ptr_->getJointModelGroup(PLANNING_GROUP_);

    target_pose1_.header.frame_id = "panda_link0";
    target_pose1_.pose.orientation.w = 1.0;
    target_pose1_.pose.position.x = 0.28;
    target_pose1_.pose.position.y = -0.2;
    target_pose1_.pose.position.z = 0.5;

    start_pose_.orientation.w = 1.0;
    start_pose_.position.x = 0.55;
    start_pose_.position.y = 0.0;
    start_pose_.position.z = 0.6;

    target_pose2_.orientation.w = 1.0;
    target_pose2_.position.x = 0.55;
    target_pose2_.position.y = -0.05;
    target_pose2_.position.z = 0.8;
  }

protected:
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_ptr_;
  PlanningComponentPtr planning_component_ptr_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  const moveit::core::JointModelGroup* jmg_ptr_;
  const std::string PLANNING_GROUP_ = "panda_arm";
  geometry_msgs::PoseStamped target_pose1_;
  geometry_msgs::Pose target_pose2_;
  geometry_msgs::Pose start_pose_;
};

// Test the current and the initial state of the Panda robot
TEST_F(MoveItCppTest, GetCurrentStateTest)
{
  ros::Duration(1.0).sleep();  // Otherwise joint_states will result in an invalid robot state
  auto robot_model = moveit_cpp_ptr_->getRobotModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  EXPECT_TRUE(moveit_cpp_ptr_->getCurrentState(robot_state, 0.0));
  // Make sure the Panda robot is in "ready" state which is loaded from fake_controller.yaml
  std::vector<double> joints_vals;
  robot_state->copyJointGroupPositions(PLANNING_GROUP_, joints_vals);
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
  EXPECT_STREQ(planning_component_ptr_->getPlanningGroupName().c_str(), "panda_arm");
}

// Test setting the start state of the plan to the current state of the robot
TEST_F(MoveItCppTest, TestSetStartStateToCurrentState)
{
  planning_component_ptr_->setStartStateToCurrentState();
  planning_component_ptr_->setGoal(target_pose1_, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr_->plan()));
  // TODO(JafarAbdi) adding testing to the soln state
}

// Test setting the goal using geometry_msgs::PoseStamped and a robot's link name
TEST_F(MoveItCppTest, TestSetGoalFromPoseStamped)
{
  planning_component_ptr_->setGoal(target_pose1_, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr_->plan()));
}

// Test setting the plan start state using moveit::core::RobotState
TEST_F(MoveItCppTest, TestSetStartStateFromRobotState)
{
  auto start_state = *(moveit_cpp_ptr_->getCurrentState());
  start_state.setFromIK(jmg_ptr_, start_pose_);

  planning_component_ptr_->setStartState(start_state);
  planning_component_ptr_->setGoal(target_pose1_, "panda_link8");

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr_->plan()));
}

// Test settting the goal of the plan using a moveit::core::RobotState
TEST_F(MoveItCppTest, TestSetGoalFromRobotState)
{
  auto target_state = *(moveit_cpp_ptr_->getCurrentState());

  target_state.setFromIK(jmg_ptr_, target_pose2_);

  planning_component_ptr_->setGoal(target_state);

  ASSERT_TRUE(static_cast<bool>(planning_component_ptr_->plan()));
}
}  // namespace moveit_cpp

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "moveit_cpp_test");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return result;
}
