/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Andy Zelenak */

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

namespace
{
constexpr double DEFAULT_TIMESTEP = 0.1;  // sec
constexpr char JOINT_GROUP[] = "panda_arm";

class RuckigTests : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, JOINT_GROUP);
  }

  moveit::core::RobotModelPtr robot_model_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  trajectory_processing::RuckigSmoothing smoother_;
};

}  // namespace

TEST_F(RuckigTests, basic_trajectory)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Second waypoint has slightly-different joint positions
  std::vector<double> joint_positions;
  robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
  joint_positions.at(0) += 0.05;
  robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));
}

TEST_F(RuckigTests, basic_trajectory_with_custom_limits)
{
  // Check the version of computeTimeStamps that takes custom velocity/acceleration limits

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Second waypoint has slightly-different joint positions
  std::vector<double> joint_positions;
  robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
  joint_positions.at(0) += 0.05;
  robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Custom velocity & acceleration limits for some joints
  std::unordered_map<std::string, double> vel_limits{ { "panda_joint1", 1.3 } };
  std::unordered_map<std::string, double> accel_limits{ { "panda_joint2", 2.3 }, { "panda_joint3", 3.3 } };
  std::unordered_map<std::string, double> jerk_limits{ { "panda_joint5", 100.0 } };

  EXPECT_TRUE(smoother_.applySmoothing(*trajectory_, vel_limits, accel_limits, jerk_limits));
}

TEST_F(RuckigTests, trajectory_duration)
{
  // Compare against the OJET online trajectory generator: https://www.trajectorygenerator.com/ojet-online/
  const double ideal_duration = 0.317;

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // Special attention to Joint 0. It is the only joint to move in this test.
  // Zero velocities and accelerations at the endpoints
  robot_state.setVariablePosition("panda_joint1", 0.0);
  trajectory_->addPrefixWayPoint(robot_state, 0.0);

  robot_state.setToDefaultValues();
  robot_state.setVariablePosition("panda_joint1", 0.1);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));

  // No waypoint durations of zero except the first
  for (size_t waypoint_idx = 1; waypoint_idx < trajectory_->getWayPointCount() - 1; ++waypoint_idx)
  {
    EXPECT_NE(trajectory_->getWayPointDurationFromPrevious(waypoint_idx), 0);
  }

  // The trajectory duration should be within 10% of the analytical solution since the implementation here extends
  // the duration by 10% increments
  EXPECT_GT(trajectory_->getWayPointDurationFromStart(trajectory_->getWayPointCount() - 1), 0.9999 * ideal_duration);
  EXPECT_LT(trajectory_->getWayPointDurationFromStart(trajectory_->getWayPointCount() - 1), 1.11 * ideal_duration);
}

TEST_F(RuckigTests, single_waypoint)
{
  // With only one waypoint, Ruckig cannot smooth the trajectory.
  // It should simply pass the trajectory through unmodified and return true.

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Trajectory should not change
  auto first_waypoint_input = robot_state;

  // Only one waypoint is acceptable. True is returned.
  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));
  // And the waypoint did not change
  auto const new_first_waypoint = trajectory_->getFirstWayPointPtr();
  auto const& variable_names = new_first_waypoint->getVariableNames();
  for (std::string const& variable_name : variable_names)
  {
    EXPECT_EQ(first_waypoint_input.getVariablePosition(variable_name),
              new_first_waypoint->getVariablePosition(variable_name));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
