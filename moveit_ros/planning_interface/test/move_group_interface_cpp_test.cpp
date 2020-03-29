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

    // get the starting pose
    start_pose_stamped_ = move_group_->getCurrentPose();

    // set velocity and acceleration scaling factors (full speed)
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
  }

  void RetrunToStartPose()
  {
    SCOPED_TRACE("RetrunToStartPose");
    PlanAndMoveToPose(start_pose_stamped_.pose);
  }

  void PlanAndMoveToPose(const geometry_msgs::Pose& pose)
  {
    SCOPED_TRACE("PlanAndMoveToPose");
    move_group_->setStartStateToCurrentState();
    ASSERT_TRUE(move_group_->setJointValueTarget(pose));
    PlanAndMove();
  }

  void PlanAndMove()
  {
    SCOPED_TRACE("PlanAndMove");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ASSERT_EQ(move_group_->plan(my_plan), moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ASSERT_EQ(move_group_->move(), moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  void TestEigenPose(const Eigen::Isometry3d& expected, const Eigen::Isometry3d& actual)
  {
    SCOPED_TRACE("TestEigenPose");
    std::stringstream ss;
    ss << "expected: \n" << expected.matrix() << "\nactual: \n" << actual.matrix();
    EXPECT_TRUE(actual.isApprox(expected, EPSILON)) << ss.str();
  }

  void TestPose(const Eigen::Isometry3d& expected_pose)
  {
    SCOPED_TRACE("TestPose(const Eigen::Isometry3d&)");
    // get the pose after the movement
    geometry_msgs::PoseStamped actual_pose_stamped = move_group_->getCurrentPose();
    Eigen::Isometry3d actual_pose;
    tf::poseMsgToEigen(actual_pose_stamped.pose, actual_pose);

    // compare to planned pose
    TestEigenPose(expected_pose, actual_pose);
  }

  void TestPose(const geometry_msgs::Pose& expected_pose_msg)
  {
    SCOPED_TRACE("TestPose(const geometry_msgs::Pose&)");
    Eigen::Isometry3d expected_pose;
    tf::poseMsgToEigen(expected_pose_msg, expected_pose);
    TestPose(expected_pose);
  }

  void TestJointPositions(const std::vector<double>& expected)
  {
    SCOPED_TRACE("TestJointPositions");
    const robot_state::JointModelGroup* joint_model_group =
        move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> actual;
    move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, actual);
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < actual.size(); ++i)
    {
      double delta = std::abs(expected[i] - actual[i]);
      EXPECT_LT(delta, EPSILON) << "joint index: " << i << ", plan: " << expected[i] << ", result: " << actual[i];
    }
  }

  void TestVectorOfStrings(const std::vector<std::string>& expected, const std::vector<std::string>& actual,
                           const std::string name)
  {
    SCOPED_TRACE("TestVectorOfStrings");
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < actual.size(); ++i)
      EXPECT_EQ(expected[i], actual[i]) << "(" << name << "[" << i << "])";
  }

protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  geometry_msgs::PoseStamped start_pose_stamped_;
};

TEST_F(MoveGroupTestFixture, StartingConditionsTest)
{
  SCOPED_TRACE("StartingConditionsTest");

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

  TestVectorOfStrings({ "ready", "extended" }, move_group_->getNamedTargets(), "named_targets");
  TestVectorOfStrings({ "hand", "panda_arm", "panda_arm_hand" }, move_group_->getJointModelGroupNames(),
                      "joint_model_group_names");
  TestVectorOfStrings({ "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                        "panda_joint7" },
                      move_group_->getJointNames(), "joint_names");
  TestVectorOfStrings({ "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6",
                        "panda_link7", "panda_link8" },
                      move_group_->getLinkNames(), "link_names");
  TestVectorOfStrings({ "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                        "panda_joint7" },
                      move_group_->getActiveJoints(), "active_joints");
  TestVectorOfStrings({ "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                        "panda_joint7", "panda_joint8" },
                      move_group_->getJoints(), "joints");
}

TEST_F(MoveGroupTestFixture, MoveToPoseTest)
{
  SCOPED_TRACE("MoveToPoseTest");

  // set current state to start state
  move_group_->setStartStateToCurrentState();

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
  TestEigenPose(eigen_target_pose, eigen_set_target_pose);

  // plan and move
  PlanAndMove();

  // get the pose after the movement
  TestPose(eigen_target_pose);

  // return to start pose for next test
  RetrunToStartPose();
}

TEST_F(MoveGroupTestFixture, JointSpaceGoalTest)
{
  SCOPED_TRACE("JointSpaceGoalTest");

  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  PlanAndMoveToPose(start_pose);

  // set start state for planning
  move_group_->setStartStateToCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> plan_joint_positions;
  move_group_->getCurrentState()->copyJointGroupPositions(
      move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP), plan_joint_positions);

  // Now, let's modify the joint positions.  (radians)
  ASSERT_EQ(plan_joint_positions.size(), std::size_t(7));
  plan_joint_positions = { 1.0, -0.5, 0.5, -1.0, 2.0, 1.0, -1.0 };
  move_group_->setJointValueTarget(plan_joint_positions);

  // plan and move
  PlanAndMove();

  // test that we moved to the expected joint positions
  TestJointPositions(plan_joint_positions);

  // return to start pose for next test
  RetrunToStartPose();
}

TEST_F(MoveGroupTestFixture, PathConstraintTest)
{
  SCOPED_TRACE("PathConstraintTest");

  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  PlanAndMoveToPose(start_pose);

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

  // move to a custom target pose
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;
  PlanAndMoveToPose(target_pose);

  // clear path constraints
  move_group_->clearPathConstraints();

  // get the pose after the movement
  TestPose(target_pose);

  // return to start pose for next test
  RetrunToStartPose();
}

TEST_F(MoveGroupTestFixture, CartPathTest)
{
  SCOPED_TRACE("CartPathTest");

  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  PlanAndMoveToPose(start_pose);

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
  TestPose(target_waypoint);

  // return to start pose for next test
  RetrunToStartPose();
}

TEST_F(MoveGroupTestFixture, CollisionObjectsTest)
{
  SCOPED_TRACE("CollisionObjectsTest");

  // set a custom start state
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.28;
  start_pose.position.y = -0.2;
  start_pose.position.z = 0.5;
  PlanAndMoveToPose(start_pose);

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
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.8;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  planning_scene_interface_.addCollisionObjects(collision_objects);

  // plan trajectory avoiding object
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.4;
  target_pose.position.z = 0.7;
  PlanAndMoveToPose(target_pose);

  // get the pose after the movement
  TestPose(target_pose);

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

  // return to start pose for next test
  RetrunToStartPose();
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
