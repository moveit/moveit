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
#include <gmock/gmock.h>

#include <moveit/robot_model/joint_model_group.h>

#include "pilz_trajectory_generation/tip_frame_getter.h"

namespace pilz_trajectory_generation
{

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ReturnRef;

class SolverMock
{
public:
  MOCK_CONST_METHOD0(getTipFrames, const std::vector<std::string>&());
};

class JointModelGroupMock
{
public:
  MOCK_CONST_METHOD0(getSolverInstance, SolverMock const*());
  MOCK_CONST_METHOD0(getName, const std::string&());
};

/**
 * @brief Test fixture for getSolverTipFrame tests.
 */
class GetSolverTipFrameTest : public testing::Test
{
protected:
  void SetUp() override;

protected:
  SolverMock solver_mock_;
  JointModelGroupMock jmg_mock_;
  const std::string group_name_ {"fake_group"};
};

void GetSolverTipFrameTest::SetUp()
{
  EXPECT_CALL(jmg_mock_, getName())
    .WillRepeatedly(ReturnRef(group_name_));
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<NoSolverException> nse_ex {new NoSolverException("")};
    EXPECT_EQ(nse_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<MoreThanOneTipFrameException> ex {new MoreThanOneTipFrameException("")};
    EXPECT_EQ(ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }
}

/**
 * @brief Checks that an exceptions is thrown in case a group has more than
 * one tip frame.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionMoreThanOneTipFrame)
{
  std::vector<std::string> tip_frames {"fake_tip_frame1", "fake_tip_frame2"};

  EXPECT_CALL(jmg_mock_, getSolverInstance())
    .Times(AtLeast(1))
    .WillRepeatedly(Return(&solver_mock_));

  EXPECT_CALL(solver_mock_, getTipFrames())
    .Times(AtLeast(1))
    .WillRepeatedly(ReturnRef(tip_frames));

  EXPECT_THROW(getSolverTipFrame(&jmg_mock_), MoreThanOneTipFrameException);
}

/**
 * @brief Checks that an exceptions is thrown in case a group does not
 * possess a solver.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionNoSolver)
{
  EXPECT_CALL(jmg_mock_, getSolverInstance())
    .WillOnce(Return(nullptr));

  EXPECT_THROW(getSolverTipFrame(&jmg_mock_), NoSolverException);
}

/**
 * @brief Checks that an exceptions is thrown in case a nullptr is
 * specified as JointModelGroup.
 */
TEST_F(GetSolverTipFrameTest, NullptrJointGroup)
{
  moveit::core::JointModelGroup* group = nullptr;
  EXPECT_THROW(hasSolver(group), std::invalid_argument);
}

}  // namespace pilz_trajectory_generation

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
