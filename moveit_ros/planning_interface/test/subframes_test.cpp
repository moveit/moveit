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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr double EPSILON = 1e-3;
constexpr double Z_OFFSET = 0.01;

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
  const std::string log_name = "sapwn_collision_objects";
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

  ROS_INFO_STREAM_NAMED(log_name, "box: " << box);

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

  ROS_INFO_STREAM_NAMED(log_name, "cylinder: " << cylinder);

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

// Helper function to get the box subframe in world
bool getBoxSubframeInWorld(Eigen::Affine3d& box_subframe_in_world, const std::string& subframe_name,
                           moveit::planning_interface::PlanningSceneInterface* planning_scene_interface)
{
  const std::string log_name = "get_box_subframe_in_world";
  std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface->getObjects();
  if (objects.find("box") == objects.end())
  {
    ROS_ERROR_NAMED(log_name, "No box object");
    return false;
  }
  moveit_msgs::CollisionObject box = objects["box"];

  std::vector<std::string> box_subframe_names = box.subframe_names;
  int index = 0;
  if (!findStringIndex(&index, box_subframe_names, subframe_name))
  {
    ROS_ERROR_STREAM_NAMED(log_name, "Subframe " << subframe_name << " not found.");
    return false;
  }
  geometry_msgs::Pose box_subframe_msg = box.subframe_poses[index];
  poseMsgToEigen(box_subframe_in_world, box_subframe_msg);

  return true;
}

// Helper function to get the cylinder tip in world
bool getCylinderTipInWorld(Eigen::Affine3d& tip_in_world,
                           moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
                           moveit::planning_interface::MoveGroupInterface* group)
{
  const std::string log_name = "get_cylinder_tip_in_world";
  geometry_msgs::Pose tip_in_hand_msg, hand_in_world_msg;
  Eigen::Affine3d tip_in_hand, hand_in_world;

  std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects =
      planning_scene_interface->getAttachedObjects();
  if (attached_objects.find("cylinder") == attached_objects.end())
  {
    ROS_ERROR_NAMED(log_name, "No cylinder object");
    return false;
  }
  if (attached_objects["cylinder"].object.subframe_poses.size() != std::size_t(1))
  {
    ROS_ERROR_NAMED(log_name, "more than one subframe_pose on cylinder");
    return false;
  }
  tip_in_hand_msg = attached_objects["cylinder"].object.subframe_poses[0];
  hand_in_world_msg = group->getCurrentPose("panda_hand").pose;
  poseMsgToEigen(tip_in_hand, tip_in_hand_msg);
  poseMsgToEigen(hand_in_world, hand_in_world_msg);
  tip_in_world = hand_in_world * tip_in_hand;

  return true;
}

// Helper function for getting target location relative to a box subframe
bool getTargetInWorld(Eigen::Affine3d& target_in_world, const Eigen::Affine3d& box_subframe_in_world)
{
  // Calculate the target location relative to the subframe (z offset)
  geometry_msgs::Pose target_in_subframe_msg;
  target_in_subframe_msg.position.z = Z_OFFSET;
  Eigen::Affine3d target_in_subframe;
  poseMsgToEigen(target_in_subframe, target_in_subframe_msg);
  target_in_world = box_subframe_in_world * target_in_subframe;

  return true;
}

// Test method to check if the cylinder tip is close to the subframe "subframe_name" of the box object
void testIsCylinderTipAtTarget(const std::string& subframe_name,
                               moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
                               moveit::planning_interface::MoveGroupInterface* group,
                               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
{
  const std::string log_name = "test_is_cylinder_tip_at_target";
  ROS_ERROR_STREAM_NAMED(log_name, "Testing if cylinder/tip is at target location relative to box/" << subframe_name);

  // lock planning scene
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);

  std::vector< moveit_msgs::AttachedCollisionObject > attached_collision_objs;
  planning_scene->getAttachedCollisionObjectMsgs(attached_collision_objs);
  for (const auto object_msg : attached_collision_objs)
  {
    ROS_ERROR_STREAM_NAMED(log_name, "" << object_msg);
  }

  // get the tip location in world
  Eigen::Affine3d tip_in_world = planning_scene->getFrameTransform("cylinder");
  // Eigen::Affine3d tip_in_world;
  // ASSERT_TRUE(getCylinderTipInWorld(tip_in_world, planning_scene_interface, group));

  // get the box subframe location
  // Isometry3d box_subframe_in_world = planning_scene->getFrameTransform("box/" + subframe_name);
  Eigen::Affine3d box_subframe_in_world;
  ASSERT_TRUE(getBoxSubframeInWorld(box_subframe_in_world, subframe_name, planning_scene_interface));

  // Calculate the target location relative to the subframe
  Eigen::Affine3d target_in_world;
  ASSERT_TRUE(getTargetInWorld(target_in_world, box_subframe_in_world));

  // test that the cylinder tip and target are in the same position
  ASSERT_LT(std::abs(target_in_world.translation()[0] - tip_in_world.translation()[0]), EPSILON);
  ASSERT_LT(std::abs(target_in_world.translation()[1] - tip_in_world.translation()[1]), EPSILON);
  ASSERT_LT(std::abs(target_in_world.translation()[2] - tip_in_world.translation()[2]), EPSILON);
}

TEST(TestPlanUsingSubframes, SubframesTests)
{
  const std::string log_name = "test_plan_using_subframes";
  SCOPED_TRACE(log_name);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::Pose tip_in_hand_msg, hand_in_world_msg, tip_in_world_msg;
  Eigen::Affine3d tip_in_hand, tip_in_world, hand_in_world;

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  auto planning_scene_monitor 
    = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer);
  planning_scene_monitor->startWorldGeometryMonitor();
  std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
  service_name = ros::names::append("/move_group/", service_name);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(10.0);

  spawnCollisionObjects(planning_scene_interface);
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "cylinder";
  att_coll_object.link_name = "panda_hand";
  att_coll_object.object.operation = att_coll_object.object.ADD;
  ROS_INFO_STREAM_NAMED(log_name, "Attaching cylinder to robot.");
  ROS_INFO_STREAM_NAMED(log_name, "att_coll_object: " << att_coll_object);
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  tf2::Quaternion target_orientation;
  target_orientation.setRPY(-(90.0 / 180.0 * M_PI), (180.0 / 180.0 * M_PI), 0);
  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  target_pose_stamped.pose.position.z = Z_OFFSET;

  // When constructing the target pose for the robot, we multiply the quaternions to get the
  // target orientation and convert the result to a ``geometry_msgs/orientation`` message.
  ROS_INFO_STREAM_NAMED(log_name, "Moving to bottom of box with cylinder tip");
  target_pose_stamped.header.frame_id = "box/bottom";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState(service_name);
  testIsCylinderTipAtTarget("bottom", &planning_scene_interface, &group, planning_scene_monitor);

  ROS_INFO_STREAM_NAMED(log_name, "Moving to top of box with cylinder tip");
  target_pose_stamped.header.frame_id = "box/top";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState(service_name);
  testIsCylinderTipAtTarget("top", &planning_scene_interface, &group, planning_scene_monitor);

  ROS_INFO_STREAM_NAMED(log_name, "Moving to box corner 1 with cylinder tip");
  target_pose_stamped.header.frame_id = "box/corner_1";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState(service_name);
  testIsCylinderTipAtTarget("corner_1", &planning_scene_interface, &group, planning_scene_monitor);

  ROS_INFO_STREAM_NAMED(log_name, "Moving to box corner 2 with cylinder tip");
  target_pose_stamped.header.frame_id = "box/corner_2";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState(service_name);
  testIsCylinderTipAtTarget("corner_2", &planning_scene_interface, &group, planning_scene_monitor);

  ROS_INFO_STREAM_NAMED(log_name, "Moving to side of box with cylinder tip");
  target_pose_stamped.header.frame_id = "box/side";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState(service_name);
  testIsCylinderTipAtTarget("side", &planning_scene_interface, &group, planning_scene_monitor);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_test_plan_using_subframes");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return result;
}

