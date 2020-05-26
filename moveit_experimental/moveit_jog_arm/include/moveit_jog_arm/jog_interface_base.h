/*******************************************************************************
 *      Title     : jog_interface_base.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
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

#include "collision_check_thread.h"
#include <Eigen/Eigenvalues>
#include "jog_arm_data.h"
#include "jog_calcs.h"
#include "low_pass_filter.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ChangeDriftDimensions.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

namespace moveit_jog_arm
{
/**
 * Class JogInterfaceBase - Base class for C++ interface and ROS node.
 * Handles ROS subs & pubs and creates the worker threads.
 */
class JogInterfaceBase
{
public:
  JogInterfaceBase();

  /** \brief Update the joints of the robot */
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  bool changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                             moveit_msgs::ChangeDriftDimensions::Response& res);

  /** \brief Start the main calculation thread */
  bool startJogCalcThread();

  /** \brief Stop the main calculation thread */
  bool stopJogCalcThread();

  /** \brief Start collision checking */
  bool startCollisionCheckThread();

  /** \brief Stop collision checking */
  bool stopCollisionCheckThread();

protected:
  bool readParameters(ros::NodeHandle& n);

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Store the parameters that were read from ROS server
  JogArmParameters ros_parameters_;

  // Share data between threads
  JogArmShared shared_variables_;

  // Jog calcs
  std::unique_ptr<JogCalcs> jog_calcs_;
  std::unique_ptr<std::thread> jog_calc_thread_;

  // Collision checks
  std::unique_ptr<CollisionCheckThread> collision_checker_;
  std::unique_ptr<std::thread> collision_check_thread_;
};
}  // namespace moveit_jog_arm
