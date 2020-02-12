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
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <ros/ros.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/sequence.h>

#include "pilz_msgs/GetMotionSequence.h"
#include "pilz_msgs/MotionSequenceRequest.h"
#include "pilz_trajectory_generation/capability_names.h"

// Parameters from parameter server
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz_industrial_motion_testutils;

static std::string createJointName(const size_t& joint_number)
{
 return std::string("prbt_joint_") + std::to_string(joint_number + 1);
}

class IntegrationTestSequenceService : public ::testing::Test
{
protected:
  void SetUp() override;

protected:
  ros::NodeHandle ph_ {"~"};
  ros::ServiceClient client_;
  robot_model::RobotModelPtr robot_model_;

  std::string test_data_file_name_;
  TestdataLoaderUPtr data_loader_;
};

void IntegrationTestSequenceService::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  data_loader_.reset(new XmlTestdataLoader(test_data_file_name_, robot_model_));
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  ASSERT_TRUE(ros::service::waitForService(pilz_trajectory_generation::SEQUENCE_SERVICE_NAME, ros::Duration(10))) << "Service not available.";
  ros::NodeHandle nh; // connect to service in global namespace, not in ph_
  client_ = nh.serviceClient<pilz_msgs::GetMotionSequence>(pilz_trajectory_generation::SEQUENCE_SERVICE_NAME);
}

/**
 * @brief Test behavior of service when empty sequence is sent.
 *
 *  Test Sequence:
 *    1. Generate empty request and call sequence service.
 *    2. Evaluate the result.
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command is successful, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSendingOfEmptySequence)
{
  pilz_msgs::MotionSequenceRequest empty_list;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = empty_list;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.error_code.val) << "Planning failed.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests that invalid (differing) group names are detected.
 *
 * Test Sequence:
 *    1. Generate request, first request has invalid group_name +  Call sequence service.
 *    2. Invalidate first request (change group_name) and send goal for planning and execution.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestDifferingGroupNames)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  MotionCmd& cmd {seq.getCmd(1)};
  cmd.setPlanningGroup("WrongGroupName");

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME, srv.response.error_code.val) << "Planning should have failed but did not.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests that negative blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with negative blend_radius +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestNegativeBlendRadius)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadius(0, -1.0);

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, srv.response.error_code.val) << "Planning should have failed but did not.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests that overlapping blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with overlapping blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestOverlappingBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadius(0, 10*seq.getBlendRadius(0));

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, srv.response.error_code.val) << "Incorrect error code";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests that too large blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with too large blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestTooLargeBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.erase(2, seq.size());
  seq.setBlendRadius(0, 10*seq.getBlendRadius(seq.size()-2));

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::FAILURE, srv.response.error_code.val) << "Incorrect error code";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests behavior of service when sequence with invalid second
 * start state is sent.
 *
 *  Test Sequence:
 *    1. Generate request (second goal has invalid start state) +  Call sequence service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSecondTrajInvalidStartState)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  pilz_msgs::MotionSequenceRequest req_list {seq.toRequest()};

  // Set start state
  using std::placeholders::_1;
  JointConfiguration config {"MyGroupName", {-1., 2., -3., 4., -5., 0.},
                            std::bind(&createJointName, _1)};
  req_list.items[1].req.start_state.joint_state = config.toSensorMsg();

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests behavior of service when sequence with invalid first goal
 * is sent.
 *
 *  Test Sequence:
 *    1. Generate request with first goal out of workspace +  Call sequence service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestFirstGoalNotReachable)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  PtpJointCart& cmd {seq.getCmd<PtpJointCart>(0)};
  cmd.getGoalConfiguration().getPose().position.y = 27;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests that incorrect link_names are detected.
 *
 * Test Sequence:
 *    1. Create sequence and send it.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestInvalidLinkName)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setAllBlendRadiiToZero();

  // Invalidate link name
  CircInterimCart& circ {seq.getCmd<CircInterimCart>(1)};
  circ.getGoalConfiguration().setLinkName("InvalidLinkName");

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_NE(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_TRUE(srv.response.planned_trajectory.empty());
}

/**
 * @brief Tests the execution of a sequence with more than two commands.
 *
 * Test Sequence:
 *    1. Call service with serveral requests.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command succeeds, result trajectory is not empty.
 */
TEST_F(IntegrationTestSequenceService, TestLargeRequest)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  // Make copy of sequence commands and add them to the end of sequence.
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

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.planned_trajectory.size(), 1u);
  EXPECT_GT(srv.response.planned_trajectory.front().joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
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
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithoutBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  seq.setAllBlendRadiiToZero();

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.planned_trajectory.size(), 1u);
  EXPECT_GT(srv.response.planned_trajectory.front().joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
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
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.planned_trajectory.size(), 1u);
  EXPECT_GT(srv.response.planned_trajectory.front().joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_sequence_service_capability");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
