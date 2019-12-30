/*******************************************************************************
 *      Title     : cpp_interface_example.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 11/20/2019
 *      Author    : Andy Zelenak
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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

#include <moveit_jog_arm/jog_cpp_interface.h>

/**
 * Instantiate the C++ jogging interface.
 * Send some Cartesian commands, then some joint commands.
 * Then retrieve the current joint state from the jogger.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, moveit_jog_arm::LOGNAME);

  // Run the jogging C++ interface in a new thread to ensure a constant outgoing message rate.
  moveit_jog_arm::JogCppApi jog_interface;
  std::thread jogging_thread([&]() { jog_interface.startMainLoop(); });

  // Make a Cartesian velocity message
  geometry_msgs::TwistStamped velocity_msg;
  velocity_msg.header.frame_id = "base_link";
  velocity_msg.twist.linear.y = 0.01;
  velocity_msg.twist.linear.z = -0.01;

  ros::Rate cmd_rate(100);
  uint num_commands = 0;

  // Send a few Cartesian velocity commands
  while (ros::ok() && num_commands < 200)
  {
    ++num_commands;
    velocity_msg.header.stamp = ros::Time::now();
    jog_interface.provideTwistStampedCommand(velocity_msg);
    cmd_rate.sleep();
  }

  // Leave plenty of time for the jogger to halt its previous motion.
  // For a faster response, adjust the incoming_command_timeout yaml parameter
  ros::Duration(2).sleep();

  // Make a joint command
  control_msgs::JointJog base_joint_command;
  base_joint_command.joint_names.push_back("elbow_joint");
  base_joint_command.velocities.push_back(0.2);
  base_joint_command.header.stamp = ros::Time::now();

  // Send a few joint commands
  num_commands = 0;
  while (ros::ok() && num_commands < 200)
  {
    ++num_commands;
    base_joint_command.header.stamp = ros::Time::now();
    jog_interface.provideJointCommand(base_joint_command);
    cmd_rate.sleep();
  }

  // Retrieve the current joint state from the jogger
  sensor_msgs::JointState current_joint_state = jog_interface.getJointState();
  ROS_INFO_STREAM(current_joint_state);

  jog_interface.stopMainLoop();
  jogging_thread.join();
  return 0;
}
