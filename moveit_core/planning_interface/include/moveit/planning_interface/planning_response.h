/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>

namespace planning_interface
{
struct MotionPlanResponse
{
  robot_trajectory::RobotTrajectoryPtr trajectory;
  double planning_time;
  moveit::core::MoveItErrorCode error_code;
  moveit_msgs::RobotState start_state;
  std::string planner_id;

  [[deprecated("Use trajectory instead.")]] const robot_trajectory::RobotTrajectoryPtr& trajectory_;
  [[deprecated("Use planning_time instead.")]] const double& planning_time_;
  [[deprecated("Use error_code instead.")]] const moveit::core::MoveItErrorCode& error_code_;
  [[deprecated("Use start_state instead.")]] const moveit_msgs::RobotState& start_state_;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MotionPlanResponse()
    : planning_time(0.0)
    , trajectory_(trajectory)
    , planning_time_(planning_time)
    , error_code_(error_code)
    , start_state_(start_state)
  {
  }
#pragma GCC diagnostic pop

  MotionPlanResponse(const MotionPlanResponse& response) : MotionPlanResponse()
  {
    *this = response;
  }

  MotionPlanResponse& operator=(const MotionPlanResponse& response)
  {
    trajectory = response.trajectory;
    planning_time = response.planning_time;
    error_code = response.error_code;
    start_state = response.start_state;
    planner_id = response.planner_id;
    return *this;
  }

  void getMessage(moveit_msgs::MotionPlanResponse& msg) const;

  // Enable checking of query success or failure, for example if(response) ...
  explicit operator bool() const
  {
    return bool(error_code);
  }
};

struct MotionPlanDetailedResponse
{
  void getMessage(moveit_msgs::MotionPlanDetailedResponse& msg) const;

  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectory;
  std::vector<std::string> description;
  std::vector<double> processing_time;
  moveit::core::MoveItErrorCode error_code;
  moveit_msgs::RobotState start_state;
  std::string planner_id;

  // Enable checking of query success or failure, for example if(response) ...
  explicit operator bool() const
  {
    return bool(error_code);
  }
};

}  // namespace planning_interface
