/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
 *  Copyright (c) 2020, Benjamin Scholz
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Benjamin Scholz, based off test_time_parameterization.cpp by Ken Anderson */

#include <gtest/gtest.h>
#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/cartesian_speed.h>
#include <moveit/utils/robot_model_test_utils.h>

// Static variables used in all tests
moveit::core::RobotModelConstPtr RMODEL = moveit::core::loadTestingRobotModel("pr2");
robot_trajectory::RobotTrajectory TRAJECTORY(RMODEL, "right_arm");

// Name of logger
const char* LOGGER_NAME = "trajectory_processing";

// Initialize one-joint, straight-line trajectory
bool initStraightTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const int num = 10;
  const double max = 2.0;
  unsigned i;

  const robot_model::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGGER_NAME, "Need to set the group");
    return false;
  }
  // leave initial velocity/acceleration unset
  const std::vector<int>& idx = group->getVariableIndexList();
  moveit::core::RobotState state(trajectory.getRobotModel());

  trajectory.clear();
  for (i = 0; i < num; i++)
  {
    state.setVariablePosition(idx[0], i * max / num);
    trajectory.addSuffixWayPoint(state, 0.0);
  }

  // leave final velocity/acceleration unset
  state.setVariablePosition(idx[0], max);
  trajectory.addSuffixWayPoint(state, 0.0);

  return true;
}

void printTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const robot_model::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  ROS_INFO_STREAM_NAMED(LOGGER_NAME,
                        "trajectory length is " << trajectory.getWayPointDurationFromStart(count - 1) << " seconds.");
  for (unsigned i = 0; i < count; i++)
  {
    robot_state::RobotStatePtr point = trajectory.getWayPointPtr(i);
    ROS_INFO_STREAM_NAMED(LOGGER_NAME, "Waypoint " << i << " time "
                                                               << trajectory.getWayPointDurationFromStart(i) << " pos "
                                                               << point->getVariablePosition(idx[0]) << " vel "
                                                               << point->getVariableVelocity(idx[0]) << " acc "
                                                               << point->getVariableAcceleration(idx[0]));

    if (i > 0)
    {
      robot_state::RobotStatePtr prev = trajectory.getWayPointPtr(i - 1);
      ROS_INFO_STREAM_NAMED(
          LOGGER_NAME,
          "jrk " << (point->getVariableAcceleration(idx[0]) - prev->getVariableAcceleration(idx[0])) /
                        (trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1)));
    }
  }
}

TEST(TestCartesianSpeed, TestCartesianEndEffectorSpeed)
{
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  EXPECT_EQ(initStraightTrajectory(TRAJECTORY), true);

  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  trajectory_processing::setMaxCartesianEndEffectorSpeed(TRAJECTORY, 0.01);
  printTrajectory(TRAJECTORY);
  size_t num_waypoints = TRAJECTORY.getWayPointCount();
  robot_state::RobotStatePtr kinematic_state = TRAJECTORY.getFirstWayPointPtr();
  Eigen::Isometry3d current_end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");
  Eigen::Isometry3d next_end_effector_state;
  double euclidean_distance = 0.0;

  for (size_t i = 0; i < num_waypoints - 1; i++)
  {
    kinematic_state = TRAJECTORY.getWayPointPtr(i + 1);
    next_end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");
    euclidean_distance += (next_end_effector_state.translation() -
            current_end_effector_state.translation()).norm();
    current_end_effector_state = next_end_effector_state;
  }
  double avg_speed = euclidean_distance / TRAJECTORY.getWayPointDurationFromStart(num_waypoints);
  ASSERT_NEAR(0.01, avg_speed, 1e-10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
