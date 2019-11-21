/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Felix von Drigalski, Jacob Aas, Tyler Weaver
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
*   * Neither the name of OMRON SINIC X or PickNik Robotics nor the
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

/* Author: Felix von Drigalski, Jacob Aas, Tyler Weaver */

/* This integration test is heavily based on the tutorial for using subframes
 * https://ros-planning.github.io/moveit_tutorials/doc/subframes/subframes_tutorial.html
 */

// C++
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>

// The Testing Framework and Utils
#include <gtest/gtest.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Function copied from tutorial
// a small helper function to create our planning requests and move the robot.
bool moveToCartPose(geometry_msgs::PoseStamped pose, moveit::planning_interface::MoveGroupInterface& group,
                    std::string end_effector_link)
{
  group.clearPoseTargets();
  group.setEndEffectorLink(end_effector_link);
  group.setStartStateToCurrentState();
  group.setPoseTarget(pose);

  // The rest of the planning is done as usual. Naturally, you can also use the ``go()`` command instead of
  // ``plan()`` and ``execute()``.
  ROS_INFO_STREAM("Planning motion to pose:");
  ROS_INFO_STREAM(pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  if (group.plan(myplan) && group.execute(myplan))
    return true;

  ROS_WARN("Failed to perform motion.");
  return false;
}

// Function copied from tutorial
// This helper function creates two objects and publishes them to the PlanningScene: a box and a cylinder.
// The box spawns in front of the gripper, the cylinder at the tip of the gripper, as if it had been grasped.
void spawnCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  double z_offset_box = .25;  // The z-axis points away from the gripper
  double z_offset_cylinder = .12;

  // First, we start defining the `CollisionObject <http://docs.ros.org/api/moveit_msgs/html/msg/CollisionObject.html>`_
  // as usual.
  moveit_msgs::CollisionObject box;
  box.id = "box";
  box.header.frame_id = "panda_hand";
  box.primitives.resize(1);
  box.primitive_poses.resize(1);
  box.primitives[0].type = box.primitives[0].BOX;
  box.primitives[0].dimensions.resize(3);
  box.primitives[0].dimensions[0] = 0.05;
  box.primitives[0].dimensions[1] = 0.1;
  box.primitives[0].dimensions[2] = 0.02;
  box.primitive_poses[0].position.z = z_offset_box;

  // Then, we define the subframes of the CollisionObject. The subframes are defined in the ``frame_id`` coordinate
  // system, just like the shapes that make up the object. Each subframe consists of a name and a pose.
  // In this tutorial, we set the orientation of the subframes so that the z-axis of the subframe
  // points away from the object.
  // This is not strictly necessary, but it is helpful to follow a convention, and it avoids confusion when
  // setting the orientation of the target pose later on.
  box.subframe_names.resize(5);
  box.subframe_poses.resize(5);

  box.subframe_names[0] = "bottom";
  box.subframe_poses[0].position.y = -.05;
  box.subframe_poses[0].position.z = 0.0 + z_offset_box;

  tf2::Quaternion orientation;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[0].orientation = tf2::toMsg(orientation);

  box.subframe_names[1] = "top";
  box.subframe_poses[1].position.y = .05;
  box.subframe_poses[1].position.z = 0.0 + z_offset_box;
  orientation.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[1].orientation = tf2::toMsg(orientation);

  box.subframe_names[2] = "corner_1";
  box.subframe_poses[2].position.x = -.025;
  box.subframe_poses[2].position.y = -.05;
  box.subframe_poses[2].position.z = -.01 + z_offset_box;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[2].orientation = tf2::toMsg(orientation);

  box.subframe_names[3] = "corner_2";
  box.subframe_poses[3].position.x = .025;
  box.subframe_poses[3].position.y = -.05;
  box.subframe_poses[3].position.z = -.01 + z_offset_box;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[3].orientation = tf2::toMsg(orientation);

  box.subframe_names[4] = "side";
  box.subframe_poses[4].position.x = .0;
  box.subframe_poses[4].position.y = .0;
  box.subframe_poses[4].position.z = -.01 + z_offset_box;
  orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
  box.subframe_poses[4].orientation = tf2::toMsg(orientation);

  // Next, define the cylinder
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "cylinder";
  cylinder.header.frame_id = "panda_hand";
  cylinder.primitives.resize(1);
  cylinder.primitive_poses.resize(1);
  cylinder.primitives[0].type = box.primitives[0].CYLINDER;
  cylinder.primitives[0].dimensions.resize(2);
  cylinder.primitives[0].dimensions[0] = 0.06;   // height (along x)
  cylinder.primitives[0].dimensions[1] = 0.005;  // radius
  cylinder.primitive_poses[0].position.x = 0.0;
  cylinder.primitive_poses[0].position.y = 0.0;
  cylinder.primitive_poses[0].position.z = 0.0 + z_offset_cylinder;
  orientation.setRPY(0, (90.0 / 180.0 * M_PI), 0);
  cylinder.primitive_poses[0].orientation = tf2::toMsg(orientation);

  cylinder.subframe_poses.resize(1);
  cylinder.subframe_names.resize(1);
  cylinder.subframe_names[0] = "tip";
  cylinder.subframe_poses[0].position.x = 0.03;
  cylinder.subframe_poses[0].position.y = 0.0;
  cylinder.subframe_poses[0].position.z = 0.0 + z_offset_cylinder;
  orientation.setRPY(0, (90.0 / 180.0 * M_PI), 0);
  cylinder.subframe_poses[0].orientation = tf2::toMsg(orientation);

  // Lastly, the objects are published to the PlanningScene. In this tutorial, we publish a box and a cylinder.
  box.operation = moveit_msgs::CollisionObject::ADD;
  cylinder.operation = moveit_msgs::CollisionObject::ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects = { box, cylinder };
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Helper function to convert pose message to eigen Affine3d
void poseMsgToEigen(Eigen::Affine3d& left, const geometry_msgs::Pose& right)
{
  left = Eigen::Translation3d(right.position.x, right.position.y, right.position.z) *
         Eigen::Quaterniond(right.orientation.w, right.orientation.x, right.orientation.y, right.orientation.z);
}

// Helper function for finding the index of a value in a list of strings
bool findStringIndex(int* index, const std::vector<std::string>& list, const std::string& value)
{
  for (size_t i = 0; i < list.size(); ++i)
  {
    if (list[i] == value)
    {
      *index = i;
      return true;
    }
  }
  return false;
}

// Test method to test if the cylinder tip at near the box subrame_name meantioned
void testAtSubframe(const std::string& subframe_name,
                    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
                    moveit::planning_interface::MoveGroupInterface* group)
{
  geometry_msgs::Pose tip_in_hand_msg, hand_in_world_msg, tip_in_world_msg, box_subframe_msg;
  Eigen::Affine3d tip_in_hand, tip_in_world, hand_in_word, box_subframe_in_world;

  std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects =
      planning_scene_interface->getAttachedObjects();
  EXPECT_EQ(attached_objects["cylinder"].object.subframe_poses.size(), std::size_t(1));
  tip_in_hand_msg = attached_objects["cylinder"].object.subframe_poses[0];
  hand_in_world_msg = group->getCurrentPose("panda_hand").pose;
  tip_in_hand_msg.position.x += 0.01;
  poseMsgToEigen(tip_in_hand, tip_in_hand_msg);
  poseMsgToEigen(hand_in_word, hand_in_world_msg);
  tip_in_world = hand_in_word * tip_in_hand;

  std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface->getObjects();
  ASSERT_NE(objects.find("box"), objects.end());
  moveit_msgs::CollisionObject box = objects["box"];

  std::vector<std::string> box_subframe_names = box.subframe_names;
  int index = 0;
  ASSERT_TRUE(findStringIndex(&index, box_subframe_names, subframe_name));
  box_subframe_msg = box.subframe_poses[index];
  poseMsgToEigen(box_subframe_in_world, box_subframe_msg);

  ASSERT_LT(std::abs(box_subframe_in_world.translation()[0] - tip_in_world.translation()[0]), 0.1);
  ASSERT_LT(std::abs(box_subframe_in_world.translation()[1] - tip_in_world.translation()[1]), 0.1);
  ASSERT_LT(std::abs(box_subframe_in_world.translation()[1] - tip_in_world.translation()[1]), 0.1);
}

TEST(TestPlanUsingSubframes, SubframesTests)
{
  SCOPED_TRACE("TestPlanUsingSubframes");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::Pose tip_in_hand_msg, hand_in_world_msg, tip_in_world_msg;
  Eigen::Affine3d tip_in_hand, tip_in_world, hand_in_word;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(10.0);

  spawnCollisionObjects(planning_scene_interface);
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "cylinder";
  att_coll_object.link_name = "panda_hand";
  att_coll_object.object.operation = att_coll_object.object.ADD;
  ROS_INFO_STREAM("Attaching cylinder to robot.");
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  // Define a pose in the robot base.
  tf2::Quaternion orientation, orientation_1, target_orientation;
  geometry_msgs::PoseStamped fixed_pose, temp_pose_stamped;
  fixed_pose.header.frame_id = "panda_link0";
  fixed_pose.pose.position.y = -.4;
  fixed_pose.pose.position.z = .3;
  orientation.setRPY(0, (-20.0 / 180.0 * M_PI), 0);
  fixed_pose.pose.orientation = tf2::toMsg(orientation);

  // We multiply tf2 quaternions to chain together a few rotations. This is usually simpler and easier
  // to debug than figuring out a series of euler angles in your head.
  tf2::Quaternion flip_around_y;  // This is used to rotate the orientation of the target pose.
  flip_around_y.setRPY(0, (180.0 / 180.0 * M_PI), 0);

  ROS_INFO_STREAM("Moving to bottom of box with cylinder tip");
  temp_pose_stamped.header.frame_id = "box/bottom";

  // When constructing the target pose for the robot, we multiply the quaternions to get the
  // target orientation and convert the result to a ``geometry_msgs/orientation`` message.
  orientation_1.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  target_orientation = flip_around_y * orientation_1;
  temp_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  temp_pose_stamped.pose.position.z = 0.01;
  moveToCartPose(temp_pose_stamped, group, "cylinder/tip");
  testAtSubframe("bottom", &planning_scene_interface, &group);

  // The command "2" moves the cylinder tip to the top of the box (the right side in the top animation).
  ROS_INFO_STREAM("Moving to top of box with cylinder tip");
  temp_pose_stamped.header.frame_id = "box/top";
  orientation_1.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  target_orientation = flip_around_y * orientation_1;
  temp_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  temp_pose_stamped.pose.position.z = 0.01;
  moveToCartPose(temp_pose_stamped, group, "cylinder/tip");
  testAtSubframe("top", &planning_scene_interface, &group);

  ROS_INFO_STREAM("Moving to top of box with cylinder tip");
  temp_pose_stamped.header.frame_id = "box/corner_1";
  orientation_1.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  target_orientation = flip_around_y * orientation_1;
  temp_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  temp_pose_stamped.pose.position.z = 0.01;
  moveToCartPose(temp_pose_stamped, group, "cylinder/tip");
  testAtSubframe("corner_1", &planning_scene_interface, &group);

  temp_pose_stamped.header.frame_id = "box/corner_2";
  orientation_1.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  target_orientation = flip_around_y * orientation_1;
  temp_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  temp_pose_stamped.pose.position.z = 0.01;
  moveToCartPose(temp_pose_stamped, group, "cylinder/tip");
  testAtSubframe("corner_2", &planning_scene_interface, &group);

  temp_pose_stamped.header.frame_id = "box/side";
  orientation_1.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  target_orientation = flip_around_y * orientation_1;
  temp_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  temp_pose_stamped.pose.position.z = 0.01;
  moveToCartPose(temp_pose_stamped, group, "cylinder/tip");
  testAtSubframe("side", &planning_scene_interface, &group);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_test_plan_using_subframes");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
