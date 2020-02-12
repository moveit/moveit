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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_extensions/joint_limits_interface_extension.h"

#include "pilz_trajectory_generation/joint_limits_aggregator.h"

using namespace pilz_extensions;

/**
 * @brief Unittest of the JointLimitsAggregator class
 */
class JointLimitsAggregator : public ::testing::Test
{
 protected:

  void SetUp() override
  {
    ros::NodeHandle node_handle("~");

    // Load robot module
    robot_model_loader::RobotModelLoader::Options opt("robot_description");
    model_loader_.reset(new robot_model_loader::RobotModelLoader(opt));
    robot_model_ = model_loader_->getModel();

    return;
  }

  /// The robot model loader
  robot_model_loader::RobotModelLoaderPtr model_loader_;

  /// The robot model
  robot_model::RobotModelConstPtr robot_model_;
};

/**
 * @brief Check for that the size of the map and the size of the given joint models is equal
 */
TEST_F(JointLimitsAggregator, ExpectedMapSize)
{
  ros::NodeHandle nh("~");

  pilz::JointLimitsContainer container
      = pilz::JointLimitsAggregator::getAggregatedLimits(nh, robot_model_->getActiveJointModels());

  EXPECT_EQ(robot_model_->getActiveJointModels().size(), container.getCount());
}

/**
 * @brief Check that the value in the parameter server correctly overrides the position(if within limits)
 */
TEST_F(JointLimitsAggregator, CorrectOverwriteByParamterPosition)
{
  ros::NodeHandle nh("~/valid_1");

  pilz::JointLimitsContainer container
        = pilz::JointLimitsAggregator::getAggregatedLimits(nh, robot_model_->getActiveJointModels());

  for(std::pair<std::string, JointLimit> lim : container)
  {
    // Check for the overwrite
    if(lim.first == "prbt_joint_1")
    {
      EXPECT_EQ(2, container.getLimit(lim.first).max_position);
      EXPECT_EQ(-2, container.getLimit(lim.first).min_position);
    }
    // Check that nothing else changed
    else
    {
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].min_position_,
                container.getLimit(lim.first).min_position);
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].max_position_,
                container.getLimit(lim.first).max_position);
    }
  }
}

/**
 * @brief Check that the value in the parameter server correctly overrides the velocity(if within limits)
 */
TEST_F(JointLimitsAggregator, CorrectOverwriteByParamterVelocity)
{
  ros::NodeHandle nh("~/valid_1");

  pilz::JointLimitsContainer container
        = pilz::JointLimitsAggregator::getAggregatedLimits(nh, robot_model_->getActiveJointModels());

  for(std::pair<std::string, JointLimit> lim : container)
  {
    // Check that velocity was only changed in joint "prbt_joint_3"
    if(lim.first == "prbt_joint_3")
    {
      EXPECT_EQ(1.1, container.getLimit(lim.first).max_velocity);
    }
    else
    {
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].max_velocity_,
                container.getLimit(lim.first).max_velocity);
    }
  }
}

/**
 * @brief Check that the acceleration and deceleration are set properly
 */
TEST_F(JointLimitsAggregator, CorrectSettingAccelerationAndDeceleration)
{
  ros::NodeHandle nh("~/valid_1");

    pilz::JointLimitsContainer container
        = pilz::JointLimitsAggregator::getAggregatedLimits(nh, robot_model_->getActiveJointModels());

  for(std::pair<std::string, JointLimit> lim : container)
  {
    if(lim.first == "prbt_joint_4")
    {
      EXPECT_EQ(5.5, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(-5.5, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
    else if(lim.first == "prbt_joint_5")
    {
      EXPECT_EQ(0, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(-6.6, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
    else
    {
      EXPECT_EQ(0, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(0, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
  }
}

/**
 * @brief Check that position limit violations are detected properly
 */
TEST_F(JointLimitsAggregator, LimitsViolationPosition)
{
  ros::NodeHandle nh_min("~/violate_position_min");

  EXPECT_THROW(pilz::JointLimitsAggregator::getAggregatedLimits(nh_min, robot_model_->getActiveJointModels()),
               pilz::AggregationBoundsViolationException);


  ros::NodeHandle nh_max("~/violate_position_max");

  EXPECT_THROW(pilz::JointLimitsAggregator::getAggregatedLimits(nh_max, robot_model_->getActiveJointModels()),
               pilz::AggregationBoundsViolationException);
}

/**
 * @brief Check that velocity limit violations are detected properly
 */
TEST_F(JointLimitsAggregator, LimitsViolationVelocity)
{
  ros::NodeHandle nh("~/violate_velocity");

  EXPECT_THROW(pilz::JointLimitsAggregator::getAggregatedLimits(nh, robot_model_->getActiveJointModels()),
               pilz::AggregationBoundsViolationException);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_joint_limits_aggregator");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
