/*
     Title     : jog_ros_interface.h
     Project   : moveit_jog_arm
     Created   : 3/9/2017
     Author    : Brian O'Neil, Blake Anderson, Andy Zelenak

BSD 3-Clause License

Copyright (c) 2018, Los Alamos National Security, LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Server node for arm jogging with MoveIt.

#pragma once

#include "collision_check_thread.h"
#include <Eigen/Eigenvalues>
#include "jog_arm_data.h"
#include "jog_calcs.h"
#include "low_pass_filter.h"
#include <moveit/robot_state/robot_state.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

namespace moveit_jog_arm
{
/**
 * Class JogROSInterface - Instantiated in main(). Handles ROS subs & pubs and creates the worker threads.
 */
class JogROSInterface
{
public:
  JogROSInterface();
  ~JogROSInterface();

private:
  // ROS subscriber callbacks
  void deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void deltaJointCmdCB(const control_msgs::JointJogConstPtr& msg);
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

  bool readParameters(ros::NodeHandle& n);

  // Jogging calculation thread
  bool startJogCalcThread();

  // Collision checking thread
  bool startCollisionCheckThread();

  // Share data between threads
  JogArmShared shared_variables_;
  pthread_mutex_t shared_variables_mutex_;

  robot_model_loader::RobotModelLoaderPtr model_loader_ptr_;

  // Store the parameters that were read from ROS server
  JogArmParameters ros_parameters_;
};
}  // namespace moveit_jog_arm
