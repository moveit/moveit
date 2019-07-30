/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Omid Heidari, Dave Coleman
   Desc:   Test the trajectory for various numbers of timesteps
 */

// C++
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <trajopt/common.hpp>

#include "trajopt_interface/problem_description.h"

namespace trackpose_cpp
{
class TrajectoryTest : public ::testing::Test
{
public:
  TrajectoryTest()
  {
  }

protected:
protected:
  robot_model::RobotModelPtr robot_model_;

};  // class TrajectoryTest

TEST_F(TrajectoryTest, GenerateInitialTrajectoryDimensions)
{
  planning_scene::PlanningSceneConstPtr planning_scene;  // not being used. no initialization needed.
  trajopt_interface::ProblemInfo pci(planning_scene, "panda_arm");
  pci.basic_info.n_steps = 30;
  std::vector<double> joint_values = { 0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075 };
  trajopt::TrajArray init_traj;

  generateInitialTrajectory(pci, joint_values, init_traj);

  EXPECT_EQ(init_traj.cols(), joint_values.size());
  EXPECT_EQ(init_traj.rows(), pci.basic_info.n_steps);
}

}  // namespace trackpose_cpp

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
