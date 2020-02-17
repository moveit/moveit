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
#include <gmock/gmock.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <stdexcept>

#include <ros/ros.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <pilz_testutils/async_test.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/sequence.h>
#include <pilz_industrial_motion_testutils/checks.h>

#include "pilz_msgs/MoveGroupSequenceAction.h"

static constexpr int WAIT_FOR_RESULT_TIME_OUT {5}; //seconds
static constexpr int TIME_BEFORE_CANCEL_GOAL {2}; //seconds
static constexpr int WAIT_FOR_ACTION_SERVER_TIME_OUT {10}; //seconds

const std::string SEQUENCE_ACTION_NAME("/sequence_move_group");

// Parameters from parameter server
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");

// events for callback tests
const std::string GOAL_SUCCEEDED_EVENT = "GOAL_SUCCEEDED";
const std::string SERVER_IDLE_EVENT = "SERVER_IDLE";

const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string GROUP_NAME("group_name");

using namespace pilz_industrial_motion_testutils;

class IntegrationTestSequenceAction : public testing::Test, public testing::AsyncTest
{
protected:
  void SetUp() override;

public:
  MOCK_METHOD0(active_callback, void());
  MOCK_METHOD1(feedback_callback, void(const pilz_msgs::MoveGroupSequenceFeedbackConstPtr& feedback));
  MOCK_METHOD2(done_callback, void(const actionlib::SimpleClientGoalState& state,
                                   const pilz_msgs::MoveGroupSequenceResultConstPtr& result));

protected:
  ros::NodeHandle ph_ {"~"};
  actionlib::SimpleActionClient<pilz_msgs::MoveGroupSequenceAction> ac_{ph_, SEQUENCE_ACTION_NAME, true};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  robot_model_loader::RobotModelLoader model_loader_;
  robot_model::RobotModelPtr robot_model_;
  double joint_position_tolerance_;

  std::string test_data_file_name_;
  std::string group_name_;
  TestdataLoaderUPtr data_loader_;

  //! The configuration at which the robot stays at the beginning of each test.
  JointConfiguration start_config;
};

void IntegrationTestSequenceAction::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));
  ph_.param<std::string>(GROUP_NAME, group_name_, "manipulator");

  robot_model_  = model_loader_.getModel();

  data_loader_.reset(new XmlTestdataLoader(test_data_file_name_, robot_model_));
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  // wait for action server
  ASSERT_TRUE(ac_.waitForServer(ros::Duration(WAIT_FOR_ACTION_SERVER_TIME_OUT))) << "Action server is not active.";

  // move to default position
  start_config = data_loader_->getJoints("ZeroPose", group_name_);
  robot_state::RobotState robot_state {start_config.toRobotState()};

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(start_config.getGroupName());
  move_group_->setPlannerId("PTP");
  move_group_->setGoalTolerance(joint_position_tolerance_);
  move_group_->setJointValueTarget(robot_state);
  move_group_->move();

  ASSERT_TRUE(isAtExpectedPosition(robot_state, *(move_group_->getCurrentState()), joint_position_tolerance_));
}

/**
 * @brief Test behavior of sequence action server when empty sequence is sent.
 *
 * Test Sequence:
 *    1. Send empty sequence.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Empty sequence is sent to the action server.
 *    2. Error code of the blend result is SUCCESS.
 */
TEST_F(IntegrationTestSequenceAction, TestSendingOfEmptySequence)
{
  pilz_msgs::MoveGroupSequenceGoal seq_goal;

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Execution of sequence failed.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that invalid (differing) group names are detected.
 *
 * Test Sequence:
 *    1. Invalidate first request (change group_name) and send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is failure.
 */
TEST_F(IntegrationTestSequenceAction, TestDifferingGroupNames)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  MotionCmd& cmd {seq.getCmd(1)};
  cmd.setPlanningGroup("WrongGroupName");

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME) << "Incorrect error code.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that negative blend radii are detected.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is not success and the planned trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, TestNegativeBlendRadius)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadius(0, -1.0);

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);

  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) << "Incorrect error code.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that overlapping blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with overlapping blend radii.
 *    2. Send goal for planning and execution.
 *    3. Evaluate the result.
 *
 * Expected Results:
 *    1. -
 *    2. Goal is sent to the action server.
 *    3. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, TestOverlappingBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadius(0, 10*seq.getBlendRadius(0));

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);

  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) << "Incorrect error code";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that too large blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with too large blend radii.
 *    2. Send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. -
 *    2. Goal is sent to the action server.
 *    3. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, TestTooLargeBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.erase(2, seq.size());
  seq.setBlendRadius(0, 10*seq.getBlendRadius(seq.size()-2));

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);

  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::FAILURE) << "Incorrect error code";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests what happens if sequence contains not executable (invalid)
 * command.
 *
 * Test Sequence:
 *    1. Create sequence containing at least one invalid command.
 *    2. Send goal for planning and execution.
 *    3. Evaluate the result.
 *
 * Expected Results:
 *    1. -
 *    2. Goal is sent to the action server.
 *    3. Error code indicates an error + planned trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, TestInvalidCmd)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  // Erase certain command to invalid command following the command in sequence.
  seq.erase(3, 4);

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_NE(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Incorrect error code.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that incorrect link_names are detected.
 *
 * Test Sequence:
 *    1. Create sequence and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceAction, TestInvalidLinkName)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setAllBlendRadiiToZero();

  // Invalidate link name
  CircInterimCart& circ {seq.getCmd<CircInterimCart>(1)};
  circ.getGoalConfiguration().setLinkName("InvalidLinkName");

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_NE(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Incorrect error code.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

//*******************************************************
//*** matcher for callback functions of action server ***
//*******************************************************
MATCHER_P(FeedbackStateEq, state, "") { return arg->state == state; }
MATCHER(IsResultSuccess, "") { return arg->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS; }
MATCHER(IsResultNotEmpty, "") { return !arg->planned_trajectory.empty() || !arg->trajectory_start.empty(); }

/**
 * @brief Tests that action server callbacks are called correctly.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is success. Active-, feedback- and done-callbacks are called.
 */
TEST_F(IntegrationTestSequenceAction, TestActionServerCallbacks)
{
  using ::testing::_;
  using ::testing::AllOf;
  using ::testing::AtLeast;
  using ::testing::InSequence;

  namespace ph = std::placeholders;

  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  // We do not need the complete sequence, just two commands.
  seq.erase(2, seq.size());

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  // set expectations (no guarantee, that done callback is called before idle feedback)
  EXPECT_CALL(*this, active_callback())
      .Times(1)
      .RetiresOnSaturation();

  EXPECT_CALL(*this, done_callback(_, AllOf(IsResultSuccess(), IsResultNotEmpty())))
      .Times(1)
      .WillOnce(ACTION_OPEN_BARRIER_VOID(GOAL_SUCCEEDED_EVENT))
      .RetiresOnSaturation();

  // the feedbacks are expected in order
  {
    InSequence dummy;

    EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("PLANNING")))
        .Times(AtLeast(1));
    EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("MONITOR")))
        .Times(AtLeast(1));
    EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("IDLE")))
        .Times(AtLeast(1))
        .WillOnce(ACTION_OPEN_BARRIER_VOID(SERVER_IDLE_EVENT))
        .RetiresOnSaturation();
  }

  // send goal using mocked callback methods
  ac_.sendGoal(seq_goal, std::bind(&IntegrationTestSequenceAction::done_callback, this, ph::_1, ph::_2),
               std::bind(&IntegrationTestSequenceAction::active_callback, this),
               std::bind(&IntegrationTestSequenceAction::feedback_callback, this, ph::_1));

  // wait for the ecpected events
  BARRIER({GOAL_SUCCEEDED_EVENT, SERVER_IDLE_EVENT});
}

/**
 * @brief Tests that goal can be cancelled.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Cancel goal before it finishes.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Goal is cancelled. Execution stops.
 */
TEST_F(IntegrationTestSequenceAction, TestCancellingOfGoal)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoal(seq_goal);
  // wait for 2 seconds
  ros::Duration(TIME_BEFORE_CANCEL_GOAL).sleep();

  ac_.cancelGoal();
  ac_.waitForResult(ros::Duration(WAIT_FOR_RESULT_TIME_OUT));

  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::PREEMPTED) << "Error code should be preempted.";
}

/**
 * @brief Tests the "only planning" flag.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is success.
 */
TEST_F(IntegrationTestSequenceAction, TestPlanOnlyFlag)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  // We do not need the complete sequence, just two commands.
  seq.erase(2, seq.size());

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.planning_options.plan_only = true;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Sequence execution failed.";
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";

  ASSERT_TRUE(isAtExpectedPosition(*(move_group_->getCurrentState()), start_config.toRobotState(), joint_position_tolerance_)) << "Robot did move although \"PlanOnly\" flag set.";
}


/**
 * @brief  Tests that robot state in planning_scene_diff is
 * ignored (Mainly for full coverage) in case "plan only" flag is set.
 *
 * Test Sequence:
 *    1. Send goal with "empty" planning scene for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is success.
 */
TEST_F(IntegrationTestSequenceAction, TestIgnoreRobotStateForPlanOnly)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  // We do not need the complete sequence, just two commands.
  seq.erase(2, seq.size());

  // create request
  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.planning_options.plan_only = true;
  seq_goal.request = seq.toRequest();

  seq_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Execution of sequence failed.";
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";

  ASSERT_TRUE(isAtExpectedPosition(*(move_group_->getCurrentState()), start_config.toRobotState(), joint_position_tolerance_)) << "Robot did move although \"PlanOnly\" flag set.";
}

/**
 * @brief Tests that negative blend radii are detected
 * (Mainly for full coverage) in case "plan only" flag is set.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is not success and the planned trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, TestNegativeBlendRadiusForPlanOnly)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadius(0, -1.0);

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();
  seq_goal.planning_options.plan_only = true;

  ac_.sendGoalAndWait(seq_goal);

  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) << "Incorrect error code.";
  EXPECT_TRUE(res->planned_trajectory.empty());
  EXPECT_TRUE(res->trajectory_start.empty());
}

/**
 * @brief Tests that robot state in planning_scene_diff is
 * ignored (Mainly for full coverage).
 *
 * Test Sequence:
 *    1. Send goal with "empty" planning scene for planning and execution.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Error code of the result is success.
 */
TEST_F(IntegrationTestSequenceAction, TestIgnoreRobotState)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  // We do not need the complete sequence, just two commands.
  seq.erase(2, seq.size());

  // create request
  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  seq_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Execution of sequence failed.";
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";
}

/**
 * @brief Tests the execution of a sequence with more than two commands.
 *
 * Test Sequence:
 *    1. Create large sequence requests and sent it to  action server.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Command succeeds, result trajectory is not empty.
 */
TEST_F(IntegrationTestSequenceAction, TestLargeRequest)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  // Create large request by making copies of the original sequence commands
  // and adding them to the end of the original sequence.
  size_t n {req.items.size()};
  for(size_t i = 0; i<n; ++i)
  {
    pilz_msgs::MotionSequenceItem item {req.items.at(i)};
    if (i == 0)
    {
      // Remove start state because only the first request
      // is allowed to have a start state in a sequence.
      item.req.start_state = moveit_msgs::RobotState();
    }
    req.items.push_back(item);
  }

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = req;

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Incorrect error code.";
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";
}

/**
 * @brief Tests the execution of a sequence command (without blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceAction, TestComplexSequenceWithoutBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  seq.setAllBlendRadiiToZero();

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";

}

/**
 * @brief Tests the execution of a sequence command (with blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceAction, TestComplexSequenceWithBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  pilz_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(res->planned_trajectory.empty()) << "Planned trajectory is empty";
  EXPECT_FALSE(res->trajectory_start.empty()) << "No start states returned";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_sequence_action_capability");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner {1};
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
