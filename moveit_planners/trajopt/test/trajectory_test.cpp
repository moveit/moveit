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

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

#include <trajopt/common.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <trajopt_interface/problem_description.h>
#include <trajopt_interface/kinematic_terms.h>

class TrajectoryTest : public ::testing::Test
{
public:
  TrajectoryTest()
  {
  }

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

TEST_F(TrajectoryTest, concatVectorValidation)
{
  std::vector<double> vec_a = {1, 2, 3, 4, 5};
  std::vector<double> vec_b = {6, 7, 8, 9, 10};
  std::vector<double> vec_c = trajopt_interface::concatVector(vec_a, vec_b);
  EXPECT_EQ(vec_c.size(), vec_a.size() + vec_b.size());

  size_t length_ab = vec_a.size() + vec_b.size();
  for(size_t index = 0; index < length_ab ; ++index)
    {
      if (index < vec_a.size())
        {
          EXPECT_EQ(vec_c[index],  vec_a[index]);
        }else
        {
          EXPECT_EQ(vec_c[index],  vec_b[index - vec_a.size()]);
        }

    }
}

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
