/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Aaryan Murgunde
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
 *   * Neither the name of PickNik Robotics nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Aaryan Murgunde */

// STD
#include <vector>
#include <random>

// ROS
#include <ros/ros.h>

// Testing Framework
#include <gtest/gtest.h>

// moveit PLanning Scene Interface
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ClearSceneFixture : public ::testing::Test
{
protected:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  int object_counter = 0;

  moveit_msgs::CollisionObject randomCollisionObject()
  {
    std::random_device rd;   // Seed the engine with a true random value if available
    std::mt19937 gen(rd());  // Mersenne Twister 19937 generator
    std::uniform_real_distribution<float> dist(-4.0f, 4.0f);

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = std::to_string(object_counter);
    collision_object.header.frame_id = "panda_link0";
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = std::abs(dist(gen));
    collision_object.primitives[0].dimensions[1] = std::abs(dist(gen));
    collision_object.primitives[0].dimensions[2] = std::abs(dist(gen));
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = dist(gen);
    collision_object.primitive_poses[0].position.y = dist(gen);
    collision_object.primitive_poses[0].position.z = dist(gen);
    collision_object.primitive_poses[0].orientation.w = 1.0;
    collision_object.operation = collision_object.ADD;

    // Global random counter update
    object_counter++;

    return collision_object;
  }
  moveit_msgs::AttachedCollisionObject randomAttachedCollisionObject()
  {
    std::random_device rd;   // Seed the engine with a true random value if available
    std::mt19937 gen(rd());  // Mersenne Twister 19937 generator
    std::uniform_real_distribution<float> dist(-4.0f, 4.0f);

    moveit_msgs::AttachedCollisionObject attached_collision_object;
    attached_collision_object.link_name = "panda_link0";
    attached_collision_object.object = randomCollisionObject();
    // Global random counter update
    object_counter++;

    return attached_collision_object;
  }
};

// Test 1 : ENV with only collision objects
TEST_F(ClearSceneFixture, CollisionObjectClearTest)
{
  // Verify the scene is clear
  ASSERT_EQ(planning_scene_interface.getObjects().size(), 0ul);

  // Add and verify if the objects have been added
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.reserve(4);

  for (int i = 0; i < 4; i++)
  {
    collision_objects.push_back(randomCollisionObject());
  }

  planning_scene_interface.applyCollisionObjects(collision_objects);
  ASSERT_EQ(planning_scene_interface.getObjects().size(), std::size_t(4));

  // Use the function call to clear the planning scene
  planning_scene_interface.clear();

  // Verify the scene is cleared
  ASSERT_EQ(planning_scene_interface.getObjects().size(), 0ul);
}

// Test 2 : ENV with only Attached objects
TEST_F(ClearSceneFixture, AttachedObjectClearTest)
{
  // Verify the scene is clear
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), 0ul);

  // Add and verify if the objects have been added
  std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;
  attached_objects.reserve(4);

  for (int i = 0; i < 4; i++)
  {
    attached_objects.push_back(randomAttachedCollisionObject());
  }

  planning_scene_interface.applyAttachedCollisionObjects(attached_objects);
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), std::size_t(4));

  // Use the function call to clear the planning scene
  planning_scene_interface.clear();

  // Verify the scene is cleared
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), 0ul);
}

TEST_F(ClearSceneFixture, CollisionAndAttachedObjectClearTest)
{
  // Verify the scene is clear
  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()), 0ul);

  // Add and verify if the objects have been added
  std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;
  attached_objects.reserve(2);

  for (int i = 0; i < 2; i++)
  {
    attached_objects.push_back(randomAttachedCollisionObject());
  }

  planning_scene_interface.applyAttachedCollisionObjects(attached_objects);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.reserve(2);

  for (int i = 0; i < 2; i++)
  {
    collision_objects.push_back(randomCollisionObject());
  }

  planning_scene_interface.applyCollisionObjects(collision_objects);

  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()),
            std::size_t(4));

  // Use the function call to clear the planning scene
  planning_scene_interface.clear();

  // Verify the scene is cleared
  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()), 0ul);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_interface_clearScene");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int test_result = RUN_ALL_TESTS();

  return test_result;
}
