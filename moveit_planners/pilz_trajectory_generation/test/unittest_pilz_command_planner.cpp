/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include <iostream>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include "pilz_trajectory_generation/pilz_command_planner.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

class CommandPlannerTest : public testing::TestWithParam<std::string>
{
protected:

  void SetUp() override
  {
    createPlannerInstance();
  }

  /**
   * @brief initialize the planner plugin
   * The planner is loaded using the pluginlib. Checks that the planner was loaded properly.
   * Exceptions will cause test failure.
   *
   * This function should be called only once during initialization of the class.
   */
  void createPlannerInstance()
  {
    // Load planner name from parameter server
    if (!ph_.getParam("planning_plugin", planner_plugin_name_))
    {
      ROS_FATAL_STREAM("Could not find planner plugin name");
      FAIL();
    }

    // Load the plugin
    try
    {
      planner_plugin_loader_.reset(
            new pluginlib::ClassLoader<planning_interface::PlannerManager>(
              "moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
      FAIL();
    }

    // Create planner
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
      ASSERT_TRUE(planner_instance_->initialize(robot_model_, ph_.getNamespace())) << "Initialzing the planner instance failed.";
    }
    catch(pluginlib::PluginlibException& ex)
    {
      FAIL()  << "Could not create planner " << ex.what() << "\n";
    }
  }

  virtual void TearDown()
  {
    planner_plugin_loader_->unloadLibraryForClass(planner_plugin_name_);
  }

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};

  std::string planner_plugin_name_;

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;

  planning_interface::PlannerManagerPtr planner_instance_;

};

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P(InstantiationName, CommandPlannerTest, ::testing::Values(
                          PARAM_MODEL_NO_GRIPPER_NAME,
                          PARAM_MODEL_WITH_GRIPPER_NAME
                          ));

/**
 * @brief Test that PTP can be loaded
 * This needs to be extended with every new planning Algorithm
 */
TEST_P(CommandPlannerTest, ObtainLoadedPlanningAlgorithms)
{
  // Check for the algorithms
  std::vector<std::string> algs;

  planner_instance_->getPlanningAlgorithms(algs);
  ASSERT_EQ(3u, algs.size()) << "Found more or less planning algorithms as expected! Found:"
                            << ::testing::PrintToString(algs);


  // Collect the algorithms, check for uniqueness
  std::set<std::string> algs_set;
  for(const auto& alg : algs)
  {
    algs_set.insert(alg);
  }
  ASSERT_EQ(algs.size(), algs_set.size()) << "There are two or more algorithms with the same name!";
  ASSERT_TRUE(algs_set.find("LIN") != algs_set.end());
  ASSERT_TRUE(algs_set.find("PTP") != algs_set.end());
  ASSERT_TRUE(algs_set.find("CIRC") != algs_set.end());
}


/**
 * @brief Check that all announced planning algorithms can perform the service request if the planner_id is set.
 */
TEST_P(CommandPlannerTest, CheckValidAlgorithmsForServiceRequest)
{
  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for(auto alg : algs)
  {
    planning_interface::MotionPlanRequest req;
    req.planner_id = alg;

    EXPECT_TRUE(planner_instance_->canServiceRequest(req));
  }
}


/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is not supported
 */
TEST_P(CommandPlannerTest, CheckInvalidAlgorithmsForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "NON_EXISTEND_ALGORITHM_HASH_da39a3ee5e6b4b0d3255bfef95601890afd80709";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}

/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is empty
 */
TEST_P(CommandPlannerTest, CheckEmptyPlannerIdForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}


/**
 * @brief Check integrety against empty input
 */
TEST_P(CommandPlannerTest, CheckPlanningContextRequestNull)
{
  moveit_msgs::MotionPlanRequest req;
  moveit_msgs::MoveItErrorCodes error_code;
  EXPECT_EQ(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
}

/**
 * @brief Check that for the announced algorithmns getPlanningContext does not return nullptr
 */
TEST_P(CommandPlannerTest, CheckPlanningContextRequest)
{
  moveit_msgs::MotionPlanRequest req;
  moveit_msgs::MoveItErrorCodes error_code;

  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for(auto alg : algs)
  {
    req.planner_id = alg;

    EXPECT_NE(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
  }

}

/**
 * @brief Check the description can be obtained and is not empty
 */
TEST_P(CommandPlannerTest, CheckPlanningContextDescriptionNotEmptyAndStable)
{
  std::string desc = planner_instance_->getDescription();
  EXPECT_GT(desc.length(), 0u);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_pilz_command_planner");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
