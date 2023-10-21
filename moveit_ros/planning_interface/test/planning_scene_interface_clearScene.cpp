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

// Vector
#include <vector>

// ROS
#include <ros/ros.h>

// Testing Framework
#include <gtest/gtest.h>

// Moveit PLanning Scene Interface
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ClearSceneFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    // Random 4 Objects
    collision_objects.resize(4);
    // OBJ 1
    collision_objects[0].id = "1";
    collision_objects[0].header.frame_id = "panda_link0";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 1;
    collision_objects[0].primitives[0].dimensions[2] = 1;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1;
    collision_objects[0].primitive_poses[0].position.y = -1;
    collision_objects[0].primitive_poses[0].position.z = 2.0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // OBJ 2
    collision_objects[1].id = "2";
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 1.8;
    collision_objects[1].primitives[0].dimensions[1] = 0.1;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -2.0;
    collision_objects[1].primitive_poses[0].position.y = 0.9;
    collision_objects[1].primitive_poses[0].position.z = 1.8;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[0].ADD;

    // OBJ 3
    collision_objects[2].id = "3";
    collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.1;
    collision_objects[2].primitives[0].dimensions[1] = 2.8;
    collision_objects[2].primitives[0].dimensions[2] = 0.01;
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 3.0;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[0].ADD;

    // OBJ 4
    collision_objects[3].id = "4";
    collision_objects[3].header.frame_id = "panda_link0";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 1.6;
    collision_objects[3].primitives[0].dimensions[1] = 0.5;
    collision_objects[3].primitives[0].dimensions[2] = 1;
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = -0.3;
    collision_objects[3].primitive_poses[0].position.y = 0.9;
    collision_objects[3].primitive_poses[0].position.z = 2.0;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    collision_objects[3].operation = collision_objects[0].ADD;

    // Copy same objects in Attached Collision Objects
    attached_objects.resize(4);
    for (int i = 0; i < 4; i++)
    {
      attached_objects[i].object = collision_objects[i];
      attached_objects[i].link_name = "panda_link0";
    }
  }

protected:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;
};

// Test 1 : ENV with only collision objects 
TEST_F(ClearSceneFixture, CollisionObjectClearTest)
{
  // Verify the scene is clear 
  ASSERT_EQ(planning_scene_interface.getObjects().size(), std::size_t(0));

  //Add and verify if the objects have been added 
  planning_scene_interface.applyCollisionObjects(collision_objects);
  ASSERT_EQ(planning_scene_interface.getObjects().size(), std::size_t(4));

  // Use the function call to clear the planning scene
  planning_scene_interface.clearScene();

  //Verify the scene is cleared
  ASSERT_EQ(planning_scene_interface.getObjects().size(), std::size_t(0));
}

// Test 2 : ENV with only Attached objects 
TEST_F(ClearSceneFixture, AttachedObjectClearTest)
{
  // Verify the scene is clear 
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), std::size_t(0));

  //Add and verify if the objects have been added 
  planning_scene_interface.applyAttachedCollisionObjects(attached_objects);
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), std::size_t(4));

  // Use the function call to clear the planning scene
  planning_scene_interface.clearScene();

  //Verify the scene is cleared
  ASSERT_EQ(planning_scene_interface.getAttachedObjects().size(), std::size_t(0));
}

TEST_F(ClearSceneFixture, CollisionAndAttachedObjectClearTest)
{
  // Verify the scene is clear 
  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()),
            std::size_t(0));

  //Add and verify if the objects have been added 
  planning_scene_interface.applyAttachedCollisionObject(attached_objects[0]);
  planning_scene_interface.applyAttachedCollisionObject(attached_objects[1]);

  planning_scene_interface.applyCollisionObject(collision_objects[2]);
  planning_scene_interface.applyCollisionObject(collision_objects[3]);

  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()),
            std::size_t(4));
  
  // Use the function call to clear the planning scene
  planning_scene_interface.clearScene();

  //Verify the scene is cleared
  ASSERT_EQ((planning_scene_interface.getAttachedObjects().size() + planning_scene_interface.getObjects().size()),
            std::size_t(0));
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