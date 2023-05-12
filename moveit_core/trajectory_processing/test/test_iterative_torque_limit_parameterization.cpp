/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Re:Build AppliedLogix, LLC
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

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/iterative_torque_limit_parameterization.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>

TEST(time_optimal_trajectory_generation, test_totg_with_torque_limits)
{
  // Request a trajectory. This will serve as the baseline.
  // Then decrease the joint torque limits and re-parameterize. The trajectory duration should be shorter.

  const std::string urdf = R"(<?xml version="1.0" ?>
      <robot name="one_robot">
      <link name="base_link"/>
      <joint name="joint_a" type="continuous">
        <axis xyz="0.7071 0.7071 0"/>
        <parent link="base_link"/>
        <child link="link_a"/>
      </joint>
      <link name="link_a"/>
      </robot>)";

  const std::string srdf = R"(<?xml version="1.0" ?>
      <robot name="one_robot">
        <virtual_joint name="base_joint" child_link="base_link" parent_frame="odom_combined" type="planar"/>
        <group name="single_dof_group">
          <joint name="joint_a"/>
        </group>
      "</robot>)";

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf);
  srdf::ModelSharedPtr srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initString(*urdf_model, srdf);
  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  ASSERT_TRUE((bool)robot_model) << "Failed to load robot model";

  auto group = robot_model->getJointModelGroup("single_dof_group");
  ASSERT_TRUE((bool)group) << "Failed to load joint model group ";
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ 0.5 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);

  const geometry_msgs::Vector3 gravity_vector = [] {
    geometry_msgs::Vector3 gravity;
    gravity.x = 0;
    gravity.y = 0;
    gravity.z = -9.81;
    return gravity;
  }();
  const std::vector<double> joint_torque_limits{ 250 };  // in N*m
  const double accel_limit_decrement_factor = 0.1;
  const std::unordered_map<std::string, double> velocity_limits = { { "joint_a", 3 } };
  const std::unordered_map<std::string, double> acceleration_limits = { { "joint_a", 3 } };

  trajectory_processing::IterativeTorqueLimitParameterization totg(0.1 /* path tolerance */, 0.01 /* resample dt */,
                                                                   0.001 /* min angle change */);

  // Assume no external forces on the robot.
  const std::vector<geometry_msgs::Wrench> external_link_wrenches = [&group] {
    geometry_msgs::Wrench zero_wrench;
    zero_wrench.force.x = 0;
    zero_wrench.force.y = 0;
    zero_wrench.force.z = 0;
    zero_wrench.torque.x = 0;
    zero_wrench.torque.y = 0;
    zero_wrench.torque.z = 0;
    // KDL (the dynamics solver) requires one wrench per link
    std::vector<geometry_msgs::Wrench> vector_of_zero_wrenches(group->getLinkModels().size(), zero_wrench);
    return vector_of_zero_wrenches;
  }();

  bool totg_success =
      totg.computeTimeStampsWithTorqueLimits(trajectory, gravity_vector, external_link_wrenches, joint_torque_limits,
                                             accel_limit_decrement_factor, velocity_limits, acceleration_limits,
                                             1.0 /* vel scaling */, 1.0 /* accel scaling */);
  ASSERT_TRUE(totg_success) << "Failed to compute timestamps";
  double first_duration = trajectory.getDuration();

  // Now decrease joint torque limits and re-time-parameterize. The trajectory duration should be longer.
  const std::vector<double> lower_torque_limits{ 1 };  // in N*m
  totg_success =
      totg.computeTimeStampsWithTorqueLimits(trajectory, gravity_vector, external_link_wrenches, lower_torque_limits,
                                             accel_limit_decrement_factor, velocity_limits, acceleration_limits,
                                             1.0 /* accel scaling */, 1.0 /* vel scaling */);
  ASSERT_TRUE(totg_success) << "Failed to compute timestamps";
  double second_duration = trajectory.getDuration();
  EXPECT_GT(second_duration, first_duration) << "The second time parameterization should result in a longer duration";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
