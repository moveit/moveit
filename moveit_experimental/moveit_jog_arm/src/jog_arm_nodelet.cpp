/*******************************************************************************
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

/*      Title     : jog_arm_nodelet.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 12/31/2018
 *      Author    : Tyler Weaver
 */

#include <memory>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <moveit_jog_arm/jog_arm.h>

namespace moveit_jog_arm
{
class JogArmNodelet : public nodelet::Nodelet
{
private:
  void onInit() override
  {
    NODELET_DEBUG("Initializing...");

    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();
    name_ = getName();

    // Load the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      NODELET_ERROR_STREAM("Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    // Start the planning scene monitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor();

    // Create and start JogArm
    jog_arm_ = std::make_shared<JogArm>(nh_, planning_scene_monitor_, private_nh_, name_);
    jog_arm_->start();
  }

  std::string name_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  JogArmPtr jog_arm_;
};

}  // namespace moveit_jog_arm

PLUGINLIB_EXPORT_CLASS(moveit_jog_arm::JogArmNodelet, nodelet::Nodelet)
