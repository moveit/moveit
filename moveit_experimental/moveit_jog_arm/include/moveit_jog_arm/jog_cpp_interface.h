/*******************************************************************************
 *      Title     : jog_cpp_interface.h
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

#pragma once

#include <atomic>
#include "jog_interface_base.h"

namespace moveit_jog_arm
{
/**
* Class JogCppApi - This class should be instantiated in a new thread
* See cpp_interface_example.cpp
*/
class JogCppApi : JogInterfaceBase
{
public:
  JogCppApi();

  ~JogCppApi();

  void startMainLoop();

  void stopMainLoop();

  // Provide a Cartesian velocity command to the jogger.
  // The units are determined by settings in the yaml file.
  void provideTwistStampedCommand(const geometry_msgs::TwistStamped& velocity_command);

  // Send joint position(s) commands
  void provideJointCommand(const control_msgs::JointJog& joint_command);

  // Returns the most recent JointState that the jogger has received.
  // May eliminate the need to create your own joint_state subscriber.
  sensor_msgs::JointState getJointState();

  // Get planning link transform.
  // The transform from the MoveIt planning frame to robot_link_command_frame
  Eigen::Isometry3d getCommandFrameTransform();

private:
  ros::NodeHandle nh_;

  std::atomic<bool> stop_requested_;
};
}  // namespace moveit_jog_arm
