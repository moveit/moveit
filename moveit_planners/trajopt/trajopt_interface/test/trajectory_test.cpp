/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Omid Heidari
   Desc:   Test the trajectory planned by trajopt
 */

// C++
#include <iostream>
#include <string>
#include <cmath>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <trajopt/problem_description.h>
#include <trajopt/kinematic_terms.h>
#include <trajopt/common.hpp>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

class TrajectoryTest : public ::testing::Test
{
public:
  TrajectoryTest()
  {
  }

protected:
  void SetUp() override
  {
    node_handle_ = ros::NodeHandle("~");
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    bool robot_model_ok_ = static_cast<bool>(robot_model_);
    if (!robot_model_ok_)
      ROS_ERROR_STREAM_NAMED("trajectory_test", "robot model is not loaded correctly");
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  std::vector<std::string> group_joint_names_;
  const std::string PLANNING_GROUP = "panda_arm";
  const double GOAL_TOLERANCE = 0.1;
  ros::NodeHandle node_handle_;
};  // class TrajectoryTest

TEST_F(TrajectoryTest, concatVectorValidation)
{
  std::vector<std::string> keys;
  node_handle_.getParamNames(keys);
  for (std::string key : keys)
  {
    std::cerr << key << std::endl;
  }

  std::vector<double> vec_a = { 1, 2, 3, 4, 5 };
  std::vector<double> vec_b = { 6, 7, 8, 9, 10 };
  std::vector<double> vec_c = trajopt::concatVector(vec_a, vec_b);
  EXPECT_EQ(vec_c.size(), vec_a.size() + vec_b.size());

  // Check if the output of concatVector is correct.
  // Loop over the output and the input vectors to see if they match
  std::size_t length_ab = vec_a.size() + vec_b.size();
  for (std::size_t index = 0; index < length_ab; ++index)
  {
    if (index < vec_a.size())
    {
      EXPECT_EQ(vec_c[index], vec_a[index]);
    }
    else
    {
      EXPECT_EQ(vec_c[index], vec_b[index - vec_a.size()]);
    }
  }
}

TEST_F(TrajectoryTest, goalTolerance)
{
  const std::string NODE_NAME = "trajectory_test";

  // Create a planing scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  // Create a RobotState to keep track of the current robot pose and planning group
  robot_state::RobotStatePtr robot_state(
      new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  robot_state->setToDefaultValues();
  robot_state->update();

  // Create JointModelGroup
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_));

  // Set the planner
  std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
  node_handle_.setParam("planning_plugin", planner_plugin_name);

  // Create pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model_, node_handle_, "planning_plugin", "request_adapters"));

  // Current state
  std::vector<double> current_joint_values = { 0, 0, 0, -1.5, 0, 0.6, 0.9 };
  robot_state->setJointGroupPositions(joint_model_group, current_joint_values);
  geometry_msgs::Pose pose_msg_current;
  const Eigen::Isometry3d& end_effector_transform_current =
      robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_current = tf2::toMsg(end_effector_transform_current);

  // Create response and request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Set start state
  // ========================================================================================
  std::vector<double> start_joint_values = { 0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075 };
<<<<<<< HEAD
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model_));
  start_state->setJointGroupPositions(joint_model_group, start_joint_values);
  start_state->update();
=======
  robot_state->setJointGroupPositions(joint_model_group, start_joint_values);
  robot_state->update();
>>>>>>> When using motion planning display in rviz, the reference trajectory is not set. Check this and warn the user if ref traj is empty

  req.start_state.joint_state.name = joint_names;
  req.start_state.joint_state.position = start_joint_values;
  req.goal_constraints.clear();
  req.group_name = PLANNING_GROUP;

  geometry_msgs::Pose pose_msg_start;
  const Eigen::Isometry3d& end_effector_transform_start = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_start = tf2::toMsg(end_effector_transform_start);

  // Set the goal state
  // ========================================================================================
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1 };
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  geometry_msgs::Pose pose_msg_goal;
  const Eigen::Isometry3d& end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_goal = tf2::toMsg(end_effector_transform_goal);

  // Reference Trajectory. The type should be defined in the yaml file.
  // ========================================================================================
  // type: JOINT_INTERPOLATED
  req.reference_trajectories.resize(1);
  req.reference_trajectories[0].joint_trajectory.resize(1);
  req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
  req.reference_trajectories[0].joint_trajectory[0].points.resize(1);
  req.reference_trajectories[0].joint_trajectory[0].points[0].positions = goal_joint_values;

  // Solve the problem
  // ========================================================================================
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }

  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
    return;
  }

  // Check the difference between the last step in the solution and the goal
  // ========================================================================================
  moveit_msgs::RobotTrajectory solution_trajectory;
  res.trajectory_->getRobotTrajectoryMsg(solution_trajectory);
  std::vector<double> joints_values_last_step = solution_trajectory.joint_trajectory.points.back().positions;

  for (std::size_t joint_index = 0; joint_index < joints_values_last_step.size(); ++joint_index)
  {
    double goal_error =
        abs(joints_values_last_step[joint_index] - req.goal_constraints[0].joint_constraints[joint_index].position);
    std::cerr << "goal_error: " << goal_error << std::endl;
    EXPECT_LT(goal_error, GOAL_TOLERANCE);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajectory_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
