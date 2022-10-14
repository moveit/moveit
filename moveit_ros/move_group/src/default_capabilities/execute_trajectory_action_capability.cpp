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

MoveGroupExecuteTrajectoryAction::~MoveGroupExecuteTrajectoryAction()
{
  std::lock_guard<std::mutex> slock(active_goals_mutex_);
  // clear any remaining thread
  auto it = active_goals_.begin();
  while (it != active_goals_.end())
  {
    auto& goal_handle = it->first;
    auto& goal_thread = it->second;
    cancelGoal(goal_handle, "");
    if (goal_thread->joinable())
      goal_thread->join();
    it++;
  }
  active_goals_.clear();
}

void MoveGroupExecuteTrajectoryAction::initialize()
{
  // start the move action server
  execute_action_server_ = std::make_unique<ExecuteTrajectoryActionServer>(
      root_node_handle_, EXECUTE_ACTION_NAME, [this](const auto& goal) { goalCallback(goal); },
      [this](const auto& goal) { cancelCallback(goal); }, false);
  execute_action_server_->start();
}

void MoveGroupExecuteTrajectoryAction::clearInactiveGoals()
{
  // clear inactive goals
  std::lock_guard<std::mutex> slock(active_goals_mutex_);
  auto it = active_goals_.begin();
  while (it != active_goals_.end())
  {
    auto& goal_handle = it->first;
    auto& goal_thread = it->second;
    if (!isActive(goal_handle))
    {
      if (goal_thread->joinable())
        goal_thread->join();
      it = active_goals_.erase(it);
    }
    else
      it++;
  }
}

void MoveGroupExecuteTrajectoryAction::cancelGoal(ExecuteTrajectoryActionServer::GoalHandle& goal_handle,
                                                  const std::string response)
{
  // Check that the goal is still available
  if (!goal_handle.getGoal())
    return;

  unsigned int status = goal_handle.getGoalStatus().status;
  if (status == actionlib_msgs::GoalStatus::PENDING || status == actionlib_msgs::GoalStatus::ACTIVE ||
      status == actionlib_msgs::GoalStatus::PREEMPTING || status == actionlib_msgs::GoalStatus::RECALLING)
  {
    moveit_msgs::ExecuteTrajectoryResult action_res;
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    goal_handle.setCanceled(action_res, response);
  }
}

bool MoveGroupExecuteTrajectoryAction::isActive(ExecuteTrajectoryActionServer::GoalHandle& goal_handle)
{
  if (!goal_handle.getGoal())
    return false;
  unsigned int status = goal_handle.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
}

void MoveGroupExecuteTrajectoryAction::goalCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle)
{
  clearInactiveGoals();

  ROS_DEBUG_STREAM_NAMED(getName(), "Goal received (ExecuteTrajectoryActionServer)" << goal_handle.getGoalID());
  std::lock_guard<std::mutex> slock(active_goals_mutex_);

  if (context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
  {
    active_goals_.push_back(std::make_pair(
        goal_handle, std::make_unique<std::thread>([this, goal_handle]() { executePath(goal_handle); })));
  }
  else
  {
    // check if we need to send a preempted message for the goal that we're currently pursuing
    if (isActive(current_goal_))
    {
      // Stop current execution and then cancel current goal
      context_->trajectory_execution_manager_->stopExecution(true);
      const std::string response =
          "This goal was canceled because another goal was received by the MoveIt action server";
      cancelGoal(current_goal_, response);
    }

    current_goal_ = goal_handle;

    active_goals_.push_back(std::make_pair(
        goal_handle, std::make_unique<std::thread>([this, goal_handle]() { executePath(goal_handle); })));
  }
}

void MoveGroupExecuteTrajectoryAction::cancelCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle)
{
  ROS_DEBUG_STREAM_NAMED(getName(),
                         "Cancel goal requested (ExecuteTrajectoryActionServer): " << goal_handle.getGoalID());
  context_->trajectory_execution_manager_->stopExecution(true);  // TODO: fix
  const std::string response = "This goal was canceled by the user";
  cancelGoal(goal_handle, response);
}

void MoveGroupExecuteTrajectoryAction::executePath(ExecuteTrajectoryActionServer::GoalHandle goal_handle)
{
  if (goal_handle.getGoalStatus().status != actionlib_msgs::GoalStatus::PENDING)
  {
    ROS_INFO_NAMED(getName(), "Preempt requested before the goal is accepted");
    return;
  }

  goal_handle.setAccepted("This goal has been accepted by the action server");

  if (!context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
    context_->trajectory_execution_manager_->clear();

  auto executionCallback = [this, goal_handle](const moveit_controller_manager::ExecutionStatus& status) {
    sendGoalResponse(goal_handle, status);
  };

  if (context_->trajectory_execution_manager_->push(goal_handle.getGoal()->trajectory, "", executionCallback))
  {
    setExecuteTrajectoryState(MONITOR, goal_handle);
    if (!context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
      context_->trajectory_execution_manager_->execute(executionCallback, true);
  }
  else
  {
    ROS_ERROR_NAMED(getName(), "Failed to pushed trajectory");
    sendGoalResponse(goal_handle, moveit_controller_manager::ExecutionStatus::ABORTED);
  }
}

void MoveGroupExecuteTrajectoryAction::sendGoalResponse(
    ExecuteTrajectoryActionServer::GoalHandle goal_handle,
    const moveit_controller_manager::ExecutionStatus& execution_status)
{
  setExecuteTrajectoryState(IDLE, goal_handle);

  moveit_msgs::ExecuteTrajectoryResult action_res;

  if (execution_status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else if (execution_status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
  else if (execution_status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
  else  // ABORTED, FAILED, UNKNOWN
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;

  const std::string response = this->getActionResultString(action_res.error_code, false, false);

  if (isActive(goal_handle))
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      goal_handle.setSucceeded(action_res, response);
    else
      goal_handle.setAborted(action_res, response);
  }
}

void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(const MoveGroupState& state,
                                                                 ExecuteTrajectoryActionServer::GoalHandle& goal_handle)
{
  if (!isActive(goal_handle))
    return;
  moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
  execute_feedback.state = stateToStr(state);
  goal_handle.publishFeedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryAction, move_group::MoveGroupCapability)
