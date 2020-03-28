/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Tyler Weaver
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

/* Author: Tyler Weaver */

/* These integration tests are based on the tutorials for using move_group:
 * https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
 */

// C++
#include <string>
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
#include <eigen_conversions/eigen_msg.h>

constexpr double EPSILON = 1e-2;

static const std::string PLANNING_GROUP = "panda_arm";
constexpr double PLANNING_TIME_S = 10;

class MoveGroupTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/move_group_interface_cpp_test");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

    // set velocity and acceleration scaling factors (full speed)
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
  }

  // Helper function to move to a start position
  void MoveToStart(const geometry_msgs::Pose& pose)
  {
    // get the joint model group
    const robot_state::JointModelGroup* joint_model_group =
        move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState start_state(*move_group_->getCurrentState());
    start_state.setFromIK(joint_model_group, pose);
    move_group_->setStartState(start_state);

    // plan and move
    ASSERT_TRUE(move_group_->setJointValueTarget(start_state));
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ASSERT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ASSERT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

TEST_F(MoveGroupTestFixture, StartingConditionsTest)
{
  // test that setting the planning time works
  move_group_->setPlanningTime(PLANNING_TIME_S);
  EXPECT_EQ(move_group_->getPlanningTime(), PLANNING_TIME_S);

  // test that the world and panda robot is initialized the way we expect by default
  EXPECT_EQ(move_group_->getNodeHandle().getNamespace(), "/");
  EXPECT_EQ(move_group_->getEndEffectorLink(), "panda_link8");
  EXPECT_EQ(move_group_->getEndEffector(), "");
  EXPECT_EQ(move_group_->getPoseReferenceFrame(), "world");
  EXPECT_EQ(move_group_->getName(), "panda_arm");
  EXPECT_EQ(move_group_->getPlanningFrame(), "world");
  EXPECT_EQ(move_group_->getVariableCount(), std::size_t(7));
  EXPECT_EQ(move_group_->getDefaultPlannerId(), "");
  EXPECT_EQ(move_group_->getPlanningTime(), 10.0);
  EXPECT_EQ(move_group_->getGoalJointTolerance(), 0.0001);
  EXPECT_EQ(move_group_->getGoalPositionTolerance(), 0.0001);
  EXPECT_EQ(move_group_->getGoalOrientationTolerance(), 0.001);

  const std::vector<std::string> NAMED_TARGETS = { "ready", "extended" };
  std::vector<std::string> named_targets = move_group_->getNamedTargets();
  ASSERT_EQ(named_targets.size(), NAMED_TARGETS.size());
  for (size_t i = 0; i < named_targets.size(); ++i)
    EXPECT_EQ(named_targets[i], NAMED_TARGETS[i]) << "index: " << i;

  const std::vector<std::string> JOINT_MODEL_GROUP_NAMES = { "hand", "panda_arm", "panda_arm_hand" };
  std::vector<std::string> joint_model_group_names = move_group_->getJointModelGroupNames();
  ASSERT_EQ(joint_model_group_names.size(), JOINT_MODEL_GROUP_NAMES.size());
  for (size_t i = 0; i < joint_model_group_names.size(); ++i)
    EXPECT_EQ(joint_model_group_names[i], JOINT_MODEL_GROUP_NAMES[i]) << "index: " << i;

  const std::vector<std::string> JOINT_NAMES = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                                 "panda_joint5", "panda_joint6", "panda_joint7" };
  std::vector<std::string> joint_names = move_group_->getJointNames();
  ASSERT_EQ(joint_names.size(), JOINT_NAMES.size());
  for (size_t i = 0; i < joint_names.size(); ++i)
    EXPECT_EQ(joint_names[i], JOINT_NAMES[i]) << "index: " << i;

  const std::vector<std::string> LINK_NAMES = { "panda_link1", "panda_link2", "panda_link3", "panda_link4",
                                                "panda_link5", "panda_link6", "panda_link7", "panda_link8" };
  std::vector<std::string> link_names = move_group_->getLinkNames();
  ASSERT_EQ(link_names.size(), LINK_NAMES.size());
  for (size_t i = 0; i < link_names.size(); ++i)
    EXPECT_EQ(link_names[i], LINK_NAMES[i]) << "index: " << i;

  const std::vector<std::string> ACTIVE_JOINTS = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                                   "panda_joint5", "panda_joint6", "panda_joint7" };
  std::vector<std::string> active_joints = move_group_->getActiveJoints();
  ASSERT_EQ(active_joints.size(), ACTIVE_JOINTS.size());
  for (size_t i = 0; i < active_joints.size(); ++i)
    EXPECT_EQ(active_joints[i], ACTIVE_JOINTS[i]) << "index: " << i;

  const std::vector<std::string> JOINTS = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                            "panda_joint5", "panda_joint6", "panda_joint7", "panda_joint8" };
  std::vector<std::string> joints = move_group_->getJoints();
  ASSERT_EQ(joints.size(), JOINTS.size());
  for (size_t i = 0; i < joints.size(); ++i)
    EXPECT_EQ(joints[i], JOINTS[i]) << "index: " << i;
}

TEST_F(MoveGroupTestFixture, MoveToPoseTest)
{
  // Test setting target pose with eigen and with geometry_msgs
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;

  // convert to eigen
  Eigen::Isometry3d eigen_target_pose;
  tf::poseMsgToEigen(target_pose, eigen_target_pose);

  // set with eigen, get ros message representation
  move_group_->setPoseTarget(eigen_target_pose);
  geometry_msgs::PoseStamped set_target_pose = move_group_->getPoseTarget();
  Eigen::Isometry3d eigen_set_target_pose;
  tf::poseMsgToEigen(set_target_pose.pose, eigen_set_target_pose);

  // expect that they are identical
  {
    std::stringstream ss;
    ss << "eigen_target_pose: \n"
       << eigen_target_pose.matrix() << "\neigen_set_target_pose: \n"
       << eigen_set_target_pose.matrix();
    EXPECT_TRUE(eigen_set_target_pose.isApprox(eigen_target_pose, EPSILON)) << ss.str();
  }

  // plan and move
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  EXPECT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // get the pose after the movement
  geometry_msgs::PoseStamped after_move_pose = move_group_->getCurrentPose();
  Eigen::Isometry3d eigen_after_move_pose;
  tf::poseMsgToEigen(after_move_pose.pose, eigen_after_move_pose);

  // compare to planned pose
  {
    std::stringstream ss;
    ss << "eigen_target_pose: \n"
       << eigen_target_pose.matrix() << "\neigen_after_move_pose: \n"
       << eigen_after_move_pose.matrix();
    EXPECT_TRUE(eigen_after_move_pose.isApprox(eigen_target_pose, EPSILON)) << ss.str();
  }
}

TEST_F(MoveGroupTestFixture, JointSpaceGoalTest)
{
  // get the joint model group
  const robot_state::JointModelGroup* joint_model_group =
      move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // set velocity and acceleration scaling factors
  move_group_->setMaxVelocityScalingFactor(0.8);
  move_group_->setMaxAccelerationScalingFactor(0.8);

  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  MoveToStart(start_pose);

  // Next get the current set of joint values for the group.
  std::vector<double> plan_joint_positions;
  move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, plan_joint_positions);

  // Now, let's modify the joint positions.  (radians)
  ASSERT_EQ(plan_joint_positions.size(), std::size_t(7));
  plan_joint_positions = { 1.0, -1.0, 0.5, -1.0, 2.0, 0.5, -1.0 };
  move_group_->setJointValueTarget(plan_joint_positions);

  // plan and move
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  EXPECT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // test that we moved to the expected joint positions
  std::vector<double> result_joint_positions;
  move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, result_joint_positions);
  ASSERT_EQ(plan_joint_positions.size(), result_joint_positions.size());
  for (size_t i = 0; i < result_joint_positions.size(); ++i)
  {
    double joint_position_plan_result_delta = std::abs(plan_joint_positions[i] - result_joint_positions[i]);
    EXPECT_LT(joint_position_plan_result_delta, EPSILON)
        << "joint index: " << i << ", plan: " << plan_joint_positions[i] << ", result: " << result_joint_positions[i];
  }
}

TEST_F(MoveGroupTestFixture, PathConstraintTest)
{
  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  MoveToStart(start_pose);

  // create an orientation constraint
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_->setPathConstraints(test_constraints);

  // set the target state
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;
  EXPECT_TRUE(move_group_->setPoseTarget(target_pose));

  // set longer planning time
  move_group_->setPlanningTime(10.0);

  // plan and move
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  EXPECT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // clear path constraints
  move_group_->clearPathConstraints();

  // get the pose after the movement
  geometry_msgs::PoseStamped after_move_pose = move_group_->getCurrentPose();
  Eigen::Isometry3d eigen_after_move_pose;
  tf::poseMsgToEigen(after_move_pose.pose, eigen_after_move_pose);
  Eigen::Isometry3d eigen_target_pose;
  tf::poseMsgToEigen(target_pose, eigen_target_pose);

  // compare to planned pose
  {
    std::stringstream ss;
    ss << "eigen_target_pose: \n"
       << eigen_target_pose.matrix() << "\neigen_after_move_pose: \n"
       << eigen_after_move_pose.matrix();
    EXPECT_TRUE(eigen_after_move_pose.isApprox(eigen_target_pose, EPSILON)) << ss.str();
  }
}

TEST_F(MoveGroupTestFixture, CartPathTest)
{
  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  MoveToStart(start_pose);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::Pose target_waypoint = start_pose;
  target_waypoint.position.z -= 0.2;
  waypoints.push_back(target_waypoint);  // down

  target_waypoint.position.y -= 0.2;
  waypoints.push_back(target_waypoint);  // right

  target_waypoint.position.z += 0.2;
  target_waypoint.position.y += 0.2;
  target_waypoint.position.x -= 0.2;
  waypoints.push_back(target_waypoint);  // up and left

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Execute trajectory
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  EXPECT_EQ(move_group_->execute(cartesian_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // get the pose after the movement
  geometry_msgs::PoseStamped after_move_pose = move_group_->getCurrentPose();
  Eigen::Isometry3d eigen_after_move_pose;
  tf::poseMsgToEigen(after_move_pose.pose, eigen_after_move_pose);
  Eigen::Isometry3d eigen_target_pose;
  tf::poseMsgToEigen(target_waypoint, eigen_target_pose);

  // compare to planned pose
  {
    std::stringstream ss;
    ss << "eigen_target_pose: \n"
       << eigen_target_pose.matrix() << "\neigen_after_move_pose: \n"
       << eigen_after_move_pose.matrix();
    EXPECT_TRUE(eigen_after_move_pose.isApprox(eigen_target_pose, EPSILON)) << ss.str();
  }
}

TEST_F(MoveGroupTestFixture, CollisionObjectsTest)
{
  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  MoveToStart(start_pose);

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  planning_scene_interface_.addCollisionObjects(collision_objects);

  // plan trajectory avoiding object
  move_group_->setStartState(*move_group_->getCurrentState());
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.4;
  target_pose.position.z = 0.9;
  move_group_->setPoseTarget(target_pose);

  // plan and move
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  EXPECT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // get the pose after the movement
  geometry_msgs::PoseStamped after_move_pose = move_group_->getCurrentPose();
  Eigen::Isometry3d eigen_after_move_pose;
  tf::poseMsgToEigen(after_move_pose.pose, eigen_after_move_pose);
  Eigen::Isometry3d eigen_target_pose;
  tf::poseMsgToEigen(target_pose, eigen_target_pose);

  // compare to planned pose
  {
    std::stringstream ss;
    ss << "eigen_target_pose: \n"
       << eigen_target_pose.matrix() << "\neigen_after_move_pose: \n"
       << eigen_after_move_pose.matrix();
    EXPECT_TRUE(eigen_after_move_pose.isApprox(eigen_target_pose, EPSILON)) << ss.str();
  }

  // attach and detach collision object
  EXPECT_TRUE(move_group_->attachObject(collision_object.id));
  EXPECT_EQ(planning_scene_interface_.getAttachedObjects().size(), std::size_t(1));
  EXPECT_TRUE(move_group_->detachObject(collision_object.id));
  EXPECT_EQ(planning_scene_interface_.getAttachedObjects().size(), std::size_t(0));

  // remove object from world
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  EXPECT_EQ(planning_scene_interface_.getObjects().size(), std::size_t(1));
  planning_scene_interface_.removeCollisionObjects(object_ids);
  EXPECT_EQ(planning_scene_interface_.getObjects().size(), std::size_t(0));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_cpp_test");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
