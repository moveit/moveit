///////////////////////////////////////////////////////////////////////////////
//      Title     : compliance_test.cpp
//      Project   : compliance_test
//      Created   : 4/2/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

// Demonstrate compliance on a stationary robot. The robot should act like a
// spring
// when pushed.

#include "jog_arm/compliance_test.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compliance_test");

  compliance_test::ComplianceClass test;

  return 0;
}

compliance_test::ComplianceClass::ComplianceClass() : spinner_(1), tf_listener_(tf_buffer_)
{
  spinner_.start();

  // To publish commands to robots
  vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>("left_arm/jog_arm_server/delta_jog_cmds", 1);

  // Listen to the jog_arm warning topic. Exit if the jogger stops
  jog_arm_warning_sub_ = n_.subscribe("jog_arm_server/halted", 1, &ComplianceClass::haltCB, this);

  // Listen to wrench data from a force/torque sensor
  ft_sub_ = n_.subscribe("left_ur5_wrench", 1, &ComplianceClass::ftCB, this);

  // Wait for first ft data to arrive
  ROS_INFO_NAMED("compliance_test", "Waiting for first force/torque data.");
  while (ros::ok() && ft_data_.header.frame_id == "")
    ros::Duration(0.1).sleep();
  ROS_INFO_NAMED("compliance_test", "Received initial FT data.");

  // Sleep to allow the publishers to be created and FT data to stabilize
  ros::Duration(2.).sleep();

  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::vector<double> stiffness(6, 50.);
  // Rotational components
  stiffness[3] = 200.;
  stiffness[4] = 200.;
  stiffness[5] = 200.;
  double filterCutoff = 10.;

  // Deadband for force/torque measurements
  std::vector<double> deadband(6, 10.);

  // Stop when any force exceeds X N, or torque exceeds X Nm
  std::vector<double> endConditionWrench(6, 60.0);

  // An object for compliant control
  compliant_control::CompliantControl comp(stiffness, deadband, endConditionWrench, filterCutoff, ft_data_, 100., 50.);

  // The 6 nominal velocity components.
  // For this demo, the robot should be stationary unless a force/torque is
  // applied
  std::vector<double> vel_nom(6, 0.0);

  // The 6 velocity feedback components
  std::vector<double> vel_out(6, 0.0);
  geometry_msgs::TwistStamped jog_cmd;
  // Make sure this command frame matches what the jog_arm node expects
  jog_cmd.header.frame_id = "left_ur5_ee_link";

  // Get initial status
  ft_data_ = transformToEEF(ft_data_, jog_cmd.header.frame_id);
  compliantEnum::exitCondition compliance_condition = comp.getVelocity(vel_nom, ft_data_, vel_out);

  // Loop at X Hz. Specific frequency is not critical
  ros::Rate rate(100.);

  while (ros::ok() && !jog_is_halted_ && (compliance_condition == compliantEnum::CONDITION_NOT_MET))
  {
    ft_data_ = transformToEEF(ft_data_, jog_cmd.header.frame_id);
    compliance_condition = comp.getVelocity(vel_nom, ft_data_, vel_out);

    // Send cmds to the robots
    jog_cmd.header.stamp = ros::Time::now();
    jog_cmd.twist.linear.x = vel_out[0];
    jog_cmd.twist.linear.y = vel_out[1];
    jog_cmd.twist.linear.z = vel_out[2];
    jog_cmd.twist.angular.x = vel_out[3];
    jog_cmd.twist.angular.y = vel_out[4];
    jog_cmd.twist.angular.z = vel_out[5];

    vel_pub_.publish(jog_cmd);

    rate.sleep();
  }

  if (jog_is_halted_)
    ROS_WARN_NAMED("compliance_test", "Jogging was halted. Singularity, jt "
                                      "limit, or collision?");
}

// CB for halt warnings from the jog_arm nodes
void compliance_test::ComplianceClass::haltCB(const std_msgs::Bool::ConstPtr& msg)
{
  jog_is_halted_ = msg->data;
}

// CB for force/torque data
void compliance_test::ComplianceClass::ftCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  ft_data_ = *msg;
  ft_data_.header.frame_id = "left_ur5_base";
}

// Transform a wrench to the EE frame
geometry_msgs::WrenchStamped compliance_test::ComplianceClass::transformToEEF(
    const geometry_msgs::WrenchStamped wrench_in, const std::string desired_ee_frame)
{
  geometry_msgs::TransformStamped prev_frame_to_new;

  prev_frame_to_new =
      tf_buffer_.lookupTransform(desired_ee_frame, wrench_in.header.frame_id, ros::Time(0), ros::Duration(1.0));

  // There is no method to transform a Wrench, so break it into vectors and
  // transform one at a time
  geometry_msgs::Vector3Stamped force_vector;
  force_vector.vector = wrench_in.wrench.force;
  force_vector.header.frame_id = wrench_in.header.frame_id;
  tf2::doTransform(force_vector, force_vector, prev_frame_to_new);

  geometry_msgs::Vector3Stamped torque_vector;
  torque_vector.vector = wrench_in.wrench.torque;
  torque_vector.header.frame_id = wrench_in.header.frame_id;
  tf2::doTransform(torque_vector, torque_vector, prev_frame_to_new);

  // Put these components back into a WrenchStamped
  geometry_msgs::WrenchStamped wrench_out;
  wrench_out.header.stamp = wrench_in.header.stamp;
  wrench_out.header.frame_id = desired_ee_frame;
  wrench_out.wrench.force = force_vector.vector;
  wrench_out.wrench.torque = torque_vector.vector;

  return wrench_out;
}