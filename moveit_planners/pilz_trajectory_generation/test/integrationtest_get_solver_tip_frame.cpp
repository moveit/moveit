/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include <memory>
#include <stdexcept>
#include <vector>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

#include "pilz_trajectory_generation/tip_frame_getter.h"

static const std::string ROBOT_DESCRIPTION_PARAM {"robot_description"};

namespace pilz_trajectory_generation
{

class GetSolverTipFrameIntegrationTest : public testing::Test
{
protected:
  void SetUp() override;

protected:
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM).getModel() };
};

void GetSolverTipFrameIntegrationTest::SetUp()
{
  if(!robot_model_)
  {
    FAIL() << "Robot model could not be loaded.";
  }
}

/**
 * @brief Check if hasSolver() can be called successfully for the manipulator group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestHasSolverManipulator)
{
  EXPECT_TRUE(hasSolver(robot_model_->getJointModelGroup("manipulator"))) << "hasSolver returns false for manipulator";
}

/**
 * @brief Check if hasSolver() can be called successfully for the gripper group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestHasSolverGripperGroup)
{
  EXPECT_FALSE(hasSolver(robot_model_->getJointModelGroup("gripper"))) << "hasSolver returns true for gripper";
}

/**
 * @brief Check if getSolverTipFrame() can be called successfully for the manipulator group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestGetTipSolverFrameManipulator)
{
  getSolverTipFrame(robot_model_->getJointModelGroup("manipulator"));
}

/**
 * @brief Check if getSolverTipFrame() fails for gripper group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestGetTipSolverFrameGripper)
{
  EXPECT_THROW(getSolverTipFrame(robot_model_->getJointModelGroup("gripper")), NoSolverException);
}

}  // namespace pilz_trajectory_generation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_get_solver_tip_frame");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
