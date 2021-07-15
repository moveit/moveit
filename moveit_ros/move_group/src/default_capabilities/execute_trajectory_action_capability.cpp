/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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

/* Author: Kentaro Wada */

#include "execute_trajectory_action_capability.h"

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupExecuteTrajectoryAction::MoveGroupExecuteTrajectoryAction() : MoveGroupCapability("ExecuteTrajectoryAction")
{
}

void MoveGroupExecuteTrajectoryAction::initialize()
{
  // start the move action server
  execute_action_server_ = std::make_unique<ExecuteTrajectoryActionServer>(
      root_node_handle_, EXECUTE_ACTION_NAME, [this](const auto& goal) { executePathCallback(goal); }, false);
  //execute_action_server_->registerPreemptCallback([this] { preemptExecuteTrajectoryCallback(); });
  execute_action_server_->start();
}

void MoveGroupExecuteTrajectoryAction::executePathCallback(ExecuteTrajectoryActionServer::GoalHandle goal)
{
  ROS_INFO_NAMED(name_, "Goal received (ExecuteTrajectoryActionServer)");
  ROS_DEBUG_STREAM_NAMED(name_, "Goal ID" << goal.getGoalID()); 
  moveit_msgs::ExecuteTrajectoryGoal goal_msg = *goal.getGoal();
  moveit_msgs::ExecuteTrajectoryResult action_res;
  
  goal.setAccepted("This goal has been accepted by the action server");
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    goal.setAborted(action_res, response);
    return;
  }
  
  // Define a lambda that sets the goal state when the trajectory is completed.
  // This is called when the trajectory finishes.
  bool execution_complete = false;
  auto goalId = goal.getGoalID();
  // Copy goalId here so that it is still available when this lambda is called later (otherwise it is deleted when the parent function ends)
  auto completedTrajectoryCallback = [&, goalId](const moveit_controller_manager::ExecutionStatus& status) {
    ROS_DEBUG_NAMED(name_, "Entered lambda");
    execution_complete = true;
    if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    }
    else
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    }
    ROS_DEBUG_STREAM_NAMED(name_, "Execution completed: " << status.asString());

    ROS_DEBUG_NAMED(name_, "getting ActionResultString");
    const std::string response = this->getActionResultString(action_res.error_code, false, false);

    ROS_DEBUG_STREAM_NAMED(name_, "List of goal_handles_: " << goal_handles_.size());
    
    {
      boost::mutex::scoped_lock lock(goal_handles_mutex_);
      for (auto it = goal_handles_.begin(); it != goal_handles_.end(); ++it)
      {
        if (it->getGoalID() == goalId)
        {
          ROS_DEBUG_STREAM_NAMED(name_, "goal (" << it->getGoal()->trajectory.joint_trajectory.joint_names[0] << ") set to " << response);
          if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
          {
            it->setSucceeded(action_res, response);
          }
          else
          {
            it->setAborted(action_res, response);
          }
          goal_handles_.erase(it);
          break;
        }
      }
      // ROS_DEBUG_STREAM_NAMED(name_, "goal set to " << response);
    }
    }; // end of lambda expression

    {
      boost::mutex::scoped_lock lock(goal_handles_mutex_);
      if (context_->trajectory_execution_manager_->pushAndExecuteSimultaneous(goal.getGoal()->trajectory, "", completedTrajectoryCallback))
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Pushed trajectory to queue.");
        goal_handles_.push_back(goal);
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Failed to pushed trajectory to queue.");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
        const std::string response = getActionResultString(action_res.error_code, false, false);
        goal.setAborted(action_res, response);
      }
    }
}

void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(MoveGroupState state)
{
  // moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
  // execute_feedback.state = stateToStr(state);
  // execute_action_server_->publishFeedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryAction, move_group::MoveGroupCapability)
