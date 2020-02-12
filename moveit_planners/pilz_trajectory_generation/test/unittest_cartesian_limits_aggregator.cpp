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

#include <ros/ros.h>

#include "pilz_trajectory_generation/cartesian_limit.h"
#include "pilz_trajectory_generation/cartesian_limits_aggregator.h"

/**
 * @brief Unittest of the CartesianLimitsAggregator class
 */
class CartesianLimitsAggregator : public ::testing::Test
{
};

/**
 * @brief Check if only velocity is set
 */
TEST_F(CartesianLimitsAggregator, OnlyVelocity)
{
  ros::NodeHandle nh("~/vel_only");

  pilz::CartesianLimit limit = pilz::CartesianLimitsAggregator::getAggregatedLimits(nh);
  EXPECT_TRUE(limit.hasMaxTranslationalVelocity());
  EXPECT_EQ(limit.getMaxTranslationalVelocity(), 10);
  EXPECT_FALSE(limit.hasMaxTranslationalAcceleration());
  EXPECT_FALSE(limit.hasMaxTranslationalDeceleration());
  EXPECT_FALSE(limit.hasMaxRotationalVelocity());
}

/**
 * @brief Check if all values are set correctly
 */
TEST_F(CartesianLimitsAggregator, AllValues)
{
  ros::NodeHandle nh("~/all");

  pilz::CartesianLimit limit = pilz::CartesianLimitsAggregator::getAggregatedLimits(nh);
  EXPECT_TRUE(limit.hasMaxTranslationalVelocity());
  EXPECT_EQ(limit.getMaxTranslationalVelocity(), 1);

  EXPECT_TRUE(limit.hasMaxTranslationalAcceleration());
  EXPECT_EQ(limit.getMaxTranslationalAcceleration(), 2);

  EXPECT_TRUE(limit.hasMaxTranslationalDeceleration());
  EXPECT_EQ(limit.getMaxTranslationalDeceleration(), -3);

  EXPECT_TRUE(limit.hasMaxRotationalVelocity());
  EXPECT_EQ(limit.getMaxRotationalVelocity(), 4);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_cartesian_limits_aggregator");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
