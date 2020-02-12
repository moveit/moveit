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

#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_trajectory_generation/joint_limits_container.h"

class JointLimitsContainerTest : public ::testing::Test
{
 protected:

  void SetUp() override
  {
    pilz_extensions::JointLimit lim1;
    lim1.has_position_limits = true;
    lim1.min_position = -2;
    lim1.max_position = 2;
    lim1.has_acceleration_limits = true;
    lim1.max_acceleration = 3;            //<- Expected for common_limit_.max_acceleration

    pilz_extensions::JointLimit lim2;
    lim2.has_position_limits = true;
    lim2.min_position = -1;               //<- Expected for common_limit_.min_position
    lim2.max_position = 1;                //<- Expected for common_limit_.max_position
    lim2.has_deceleration_limits = true;
    lim2.max_deceleration = -5;           //<- Expected for common_limit_.max_deceleration

    pilz_extensions::JointLimit lim3;
    lim3.has_velocity_limits = true;
    lim3.max_velocity = 10;

    pilz_extensions::JointLimit lim4;
    lim4.has_position_limits = true;
    lim4.min_position = -1;
    lim4.max_position = 1;
    lim4.has_acceleration_limits = true;
    lim4.max_acceleration = 400;
    lim4.has_deceleration_limits = false;
    lim4.max_deceleration = -1;

    pilz_extensions::JointLimit lim5;
    lim5.has_position_limits = true;
    lim5.min_position = -1;
    lim5.max_position = 1;
    lim5.has_acceleration_limits = false;
    lim5.max_acceleration = 1;

    pilz_extensions::JointLimit lim6;
    lim6.has_velocity_limits = true;
    lim6.max_velocity = 2;                //<- Expected for common_limit_.max_velocity
    lim6.has_deceleration_limits = true;
    lim6.max_deceleration = -100;


    container_.addLimit("joint1", lim1);
    container_.addLimit("joint2", lim2);
    container_.addLimit("joint3", lim3);
    container_.addLimit("joint4", lim4);
    container_.addLimit("joint5", lim5);
    container_.addLimit("joint6", lim6);

    common_limit_ = container_.getCommonLimit();

  }

  pilz::JointLimitsContainer container_;
  pilz_extensions::JointLimit common_limit_;

};

/**
 * @brief Check postion
 */
TEST_F(JointLimitsContainerTest, CheckPositionUnification)
{
  EXPECT_EQ(-1, common_limit_.min_position);
  EXPECT_EQ(1, common_limit_.max_position);
}

/**
 * @brief Check velocity
 */
TEST_F(JointLimitsContainerTest, CheckVelocityUnification)
{
  EXPECT_EQ(2, common_limit_.max_velocity);
}


/**
 * @brief Check acceleration
 */
TEST_F(JointLimitsContainerTest, CheckAccelerationUnification)
{
  EXPECT_EQ(3, common_limit_.max_acceleration);
}

/**
 * @brief Check deceleration
 */
TEST_F(JointLimitsContainerTest, CheckDecelerationUnification)
{
  EXPECT_EQ(-5, common_limit_.max_deceleration);
}

/**
 * @brief Check AddLimit for positive and null deceleration
 */
TEST_F(JointLimitsContainerTest, CheckAddLimitDeceleration)
{

  pilz_extensions::JointLimit lim_invalid1;
  lim_invalid1.has_deceleration_limits = true;
  lim_invalid1.max_deceleration = 0;

  pilz_extensions::JointLimit lim_invalid2;
  lim_invalid2.has_deceleration_limits = true;
  lim_invalid2.max_deceleration = 1;

  pilz_extensions::JointLimit lim_valid;
  lim_valid.has_deceleration_limits = true;
  lim_valid.max_deceleration = -1;

  pilz::JointLimitsContainer container;

  EXPECT_EQ(false, container.addLimit("joint_invalid1", lim_invalid1));
  EXPECT_EQ(false, container.addLimit("joint_invalid2", lim_invalid2));
  EXPECT_EQ(true, container.addLimit("joint_valid", lim_valid));
}

/**
 * @brief Check AddLimit for already contained limit
 */
TEST_F(JointLimitsContainerTest, CheckAddLimitAlreadyContained)
{
  pilz_extensions::JointLimit lim_valid;
  lim_valid.has_deceleration_limits = true;
  lim_valid.max_deceleration = -1;

  pilz::JointLimitsContainer container;
  ASSERT_TRUE(container.addLimit("joint_valid", lim_valid));
  EXPECT_FALSE(container.addLimit("joint_valid", lim_valid));
}

/**
 * @brief An uninitialized container should not have any limits set.
 */
TEST_F(JointLimitsContainerTest, CheckEmptyContainer)
{
  pilz::JointLimitsContainer container;
  pilz_extensions::JointLimit limits = container.getCommonLimit();
  EXPECT_FALSE(limits.has_position_limits);
  EXPECT_FALSE(limits.has_velocity_limits);
  EXPECT_FALSE(limits.has_acceleration_limits);
}

/**
 * @brief empty position limits for first joint, second one should be returned
 */
TEST_F(JointLimitsContainerTest, FirstPositionEmpty)
{
  pilz_extensions::JointLimit lim1;

  pilz_extensions::JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;               //<- Expected for common_limit_.min_position
  lim2.max_position = 1;                //<- Expected for common_limit_.max_position

  pilz::JointLimitsContainer container;
  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  pilz_extensions::JointLimit limits = container.getCommonLimit();
  EXPECT_TRUE(limits.has_position_limits);
  EXPECT_EQ(1, limits.max_position);
  EXPECT_EQ(-1, limits.min_position);
}

/**
 * @brief Check position limits
 */
TEST_F(JointLimitsContainerTest, CheckVerifyPositionLimits)
{
  // positive check: inside limits
  std::vector<std::string> joint_names {"joint1", "joint2"};
  std::vector<double> joint_positions {0.5, 0.5};
  EXPECT_TRUE(container_.verifyPositionLimits(joint_names, joint_positions));

  // outside limit2
  joint_positions[1] = 7;
  EXPECT_FALSE(container_.verifyPositionLimits(joint_names, joint_positions));

  // invalid size
  std::vector<double> joint_positions1 {0.};
  EXPECT_THROW(container_.verifyPositionLimits(joint_names, joint_positions1),
               std::out_of_range);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
