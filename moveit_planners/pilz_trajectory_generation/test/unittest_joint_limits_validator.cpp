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

#include "pilz_trajectory_generation/joint_limits_validator.h"

class JointLimitsValidatorTest : public ::testing::Test
{
};

/**
 * @brief Check postion equality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionEquality)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in min_position detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMinPosition)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -2;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in max_position detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxPosition1)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 2;

  pilz_extensions::JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in max_position detection (using 3 limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxPosition2)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 2;

  pilz_extensions::JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;
  lim2.max_position = 1;

  pilz_extensions::JointLimit lim3;
  lim3.has_position_limits = true;
  lim3.min_position = -1;
  lim3.max_position = 1;


  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}


/**
 * @brief Check postion inequality in has_position_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasPositionLimits)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_position_limits = false;
  lim2.min_position = -1;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// VELOCITY
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


/**
 * @brief Check velocity equality
 */
TEST_F(JointLimitsValidatorTest, CheckVelocityEquality)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_velocity inequality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxVelocity1)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_velocity_limits = true;
  lim2.max_velocity = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_velocity inequality (using 3Limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxVelocity2)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_velocity_limits = true;
  lim2.max_velocity = 2;

  pilz_extensions::JointLimit lim3;
  lim3.has_velocity_limits = true;
  lim3.max_velocity = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in has_position_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasVelocityLimits)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_velocity_limits = true;

  pilz_extensions::JointLimit lim2;
  lim2.has_velocity_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// ACCELERATION
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------



/**
 * @brief Check acceleration equality
 */
TEST_F(JointLimitsValidatorTest, CheckAccelerationEquality)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_acceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxAcceleration1)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_acceleration_limits = true;
  lim2.max_acceleration = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_acceleration inequality (using 3 Limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxAcceleration2)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  pilz_extensions::JointLimit lim2;
  lim2.has_acceleration_limits = true;
  lim2.max_acceleration = 2;

  pilz_extensions::JointLimit lim3;
  lim3.has_acceleration_limits = true;
  lim3.max_acceleration = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check acceleration inequality in has_acceleration_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasAccelerationLimits)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_acceleration_limits = true;

  pilz_extensions::JointLimit lim2;
  lim2.has_acceleration_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// DECELERATION
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


/**
 * @brief Check deceleration equality
 */
TEST_F(JointLimitsValidatorTest, CheckDecelerationEquality)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_deceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityMaxDeceleration1)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  pilz_extensions::JointLimit lim2;
  lim2.has_deceleration_limits = true;
  lim2.max_deceleration = -2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_deceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityMaxDeceleration2)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  pilz_extensions::JointLimit lim2;
  lim2.has_deceleration_limits = true;
  lim2.max_deceleration = -2;

  pilz_extensions::JointLimit lim3;
  lim3.has_deceleration_limits = true;
  lim3.max_deceleration = -2;


  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check deceleration inequality in has_deceleration_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityHasDecelerationLimits)
{

  pilz::JointLimitsContainer container;

  pilz_extensions::JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  pilz_extensions::JointLimit lim2;
  lim2.has_deceleration_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  ASSERT_EQ(2u, container.getCount());

  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
