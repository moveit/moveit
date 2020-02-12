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

#include <gtest/gtest.h>

#include <pilz_trajectory_generation/trajectory_generator.h>

using namespace pilz;

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST(TrajectoryGeneratorTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<TrajectoryGeneratorInvalidLimitsException> tgil_ex {new TrajectoryGeneratorInvalidLimitsException("")};
    EXPECT_EQ(tgil_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<VelocityScalingIncorrect> vsi_ex {new VelocityScalingIncorrect("")};
    EXPECT_EQ(vsi_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<AccelerationScalingIncorrect> asi_ex {new AccelerationScalingIncorrect("")};
    EXPECT_EQ(asi_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<UnknownPlanningGroup> upg_ex {new UnknownPlanningGroup("")};
    EXPECT_EQ(upg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME);
  }

  {
    std::shared_ptr<NoJointNamesInStartState> njniss_ex {new NoJointNamesInStartState("")};
    EXPECT_EQ(njniss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<SizeMismatchInStartState> smiss_ex {new SizeMismatchInStartState("")};
    EXPECT_EQ(smiss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<JointsOfStartStateOutOfRange> jofssoor_ex {new JointsOfStartStateOutOfRange("")};
    EXPECT_EQ(jofssoor_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<NonZeroVelocityInStartState> nzviss_ex {new NonZeroVelocityInStartState("")};
    EXPECT_EQ(nzviss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<NotExactlyOneGoalConstraintGiven> neogcg_ex {new NotExactlyOneGoalConstraintGiven("")};
    EXPECT_EQ(neogcg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<OnlyOneGoalTypeAllowed> oogta_ex {new OnlyOneGoalTypeAllowed("")};
    EXPECT_EQ(oogta_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<StartStateGoalStateMismatch> ssgsm_ex {new StartStateGoalStateMismatch("")};
    EXPECT_EQ(ssgsm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<JointConstraintDoesNotBelongToGroup> jcdnbtg_ex {new JointConstraintDoesNotBelongToGroup("")};
    EXPECT_EQ(jcdnbtg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<JointsOfGoalOutOfRange> jogoor_ex {new JointsOfGoalOutOfRange("")};
    EXPECT_EQ(jogoor_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<PositionConstraintNameMissing> pcnm_ex {new PositionConstraintNameMissing("")};
    EXPECT_EQ(pcnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<OrientationConstraintNameMissing> ocnm_ex {new OrientationConstraintNameMissing("")};
    EXPECT_EQ(ocnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<PositionOrientationConstraintNameMismatch> pocnm_ex {new PositionOrientationConstraintNameMismatch("")};
    EXPECT_EQ(pocnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<NoIKSolverAvailable> nisa_ex {new NoIKSolverAvailable("")};
    EXPECT_EQ(nisa_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
  }

  {
    std::shared_ptr<NoPrimitivePoseGiven> nppg_ex {new NoPrimitivePoseGiven("")};
    EXPECT_EQ(nppg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
