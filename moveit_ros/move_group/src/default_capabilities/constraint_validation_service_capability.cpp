/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, OMRON SINIC X Corporation
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
 *   * Neither the name of OMRON SINIC X Corp. nor the names of its
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

/* Author: Felix von Drigalski */

#include "constraint_validation_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupConstraintValidationService::MoveGroupConstraintValidationService()
  : MoveGroupCapability("ConstraintValidationService")
{
}

void move_group::MoveGroupConstraintValidationService::initialize()
{
  constraint_validity_service_ = root_node_handle_.advertiseService(
      CONSTRAINT_VALIDITY_SERVICE_NAME, &MoveGroupConstraintValidationService::computeService, this);
}

bool move_group::MoveGroupConstraintValidationService::computeService(moveit_msgs::GetConstraintValidity::Request& req,
                                                                      moveit_msgs::GetConstraintValidity::Response& res)
{
  planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
  robot_state::RobotState rs = ls->getCurrentState();

  ROS_INFO_NAMED("constraint_validation_service_capability", "Validating constraints inside capability");
  // Checks if the goal is defined either for existing link_names or the names of AttachedBody objects
  // that are attached to the robot. The latter are transformed to link_names so the planner can deal with them.
  res.constraints = req.constraints;
  bool valid = true;
  valid = valid && kinematic_constraints::validatePositionOrientationConstraints(rs, res.constraints);

  res.valid = valid;
  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupConstraintValidationService, move_group::MoveGroupCapability)
