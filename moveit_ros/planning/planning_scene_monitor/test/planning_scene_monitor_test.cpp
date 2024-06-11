/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, University of Hamburg
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Michael 'v4hn' Goerner
   Desc: Tests for PlanningSceneMonitor
*/

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

class PlanningSceneMonitorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    psm = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    psm->monitorDiffs(true);
    scene = psm->getPlanningScene();
  }

  void TearDown() override
  {
    scene.reset();
  }

protected:
  planning_scene_monitor::PlanningSceneMonitorPtr psm;
  planning_scene::PlanningScenePtr scene;
};

// various code expects the monitored scene to remain the same
TEST_F(PlanningSceneMonitorTest, TestPersistentScene)
{
  auto scene{ psm->getPlanningScene() };
  moveit_msgs::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  psm->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, psm->getPlanningScene());

  msg.is_diff = msg.robot_state.is_diff = false;
  psm->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, psm->getPlanningScene());
}

using UpdateType = planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType;

#define TRIGGERS_UPDATE(msg, expected_update_type)                                                                     \
  {                                                                                                                    \
    psm->clearUpdateCallbacks();                                                                                       \
    auto received_update_type{ UpdateType::UPDATE_NONE };                                                              \
    psm->addUpdateCallback([&](auto type) { received_update_type = type; });                                           \
    psm->newPlanningSceneMessage(msg);                                                                                 \
    EXPECT_EQ(received_update_type, expected_update_type);                                                             \
  }

TEST_F(PlanningSceneMonitorTest, UpdateTypes)
{
  moveit_msgs::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_NONE);

  msg.fixed_frame_transforms.emplace_back();
  msg.fixed_frame_transforms.back().header.frame_id = "base_link";
  msg.fixed_frame_transforms.back().child_frame_id = "object";
  msg.fixed_frame_transforms.back().transform.rotation.w = 1.0;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_TRANSFORMS);
  msg.fixed_frame_transforms.clear();

  moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), msg.robot_state, false);
  msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE);

  msg.robot_state.is_diff = false;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE | UpdateType::UPDATE_GEOMETRY);

  msg.robot_state = moveit_msgs::RobotState{};
  msg.robot_state.is_diff = true;
  moveit_msgs::CollisionObject co;
  co.header.frame_id = "base_link";
  co.id = "object";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.pose.orientation.w = 1.0;
  co.primitives.emplace_back();
  co.primitives.back().type = shape_msgs::SolidPrimitive::SPHERE;
  co.primitives.back().dimensions = { 1.0 };
  msg.world.collision_objects.emplace_back(co);
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_GEOMETRY);

  msg.world.collision_objects.clear();
  msg.is_diff = false;

  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_SCENE);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "planning_scene_monitor_test");

  // ros::AsyncSpinner spinner{ 1 };
  // spinner.start();

  return RUN_ALL_TESTS();
}
