/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <memory>

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit_msgs/MoveGroupSequenceAction.h>

namespace trapezoidal_trajectory_generation
{
class CommandListManager;

/**
 * @brief Provide action to handle multiple trajectories and execute the result
 * in the form of a MoveGroup capability (plugin).
 */
class MoveGroupSequenceAction : public move_group::MoveGroupCapability
{
public:
  MoveGroupSequenceAction();

  virtual void initialize() override;

private:
  using ExecutableTrajs = std::vector<plan_execution::ExecutableTrajectory>;

  using StartStateMsgs = moveit_msgs::MoveGroupSequenceResult::_trajectory_start_type;
  using PlannedTrajMsgs = moveit_msgs::MoveGroupSequenceResult::_planned_trajectory_type;

private:
  void executeSequenceCallback(const moveit_msgs::MoveGroupSequenceGoalConstPtr& goal);
  void executeSequenceCallbackPlanAndExecute(const moveit_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                             moveit_msgs::MoveGroupSequenceResult& action_res);
  void executeMoveCallbackPlanOnly(const moveit_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                   moveit_msgs::MoveGroupSequenceResult& res);
  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(move_group::MoveGroupState state);
  bool planUsingSequenceManager(const moveit_msgs::MotionSequenceRequest& req,
                                plan_execution::ExecutableMotionPlan& plan);

private:
  static void convertToMsg(const ExecutableTrajs& trajs, StartStateMsgs& startStatesMsgs,
                           PlannedTrajMsgs& plannedTrajsMsgs);

private:
  std::unique_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupSequenceAction> > move_action_server_;
  moveit_msgs::MoveGroupSequenceFeedback move_feedback_;

  move_group::MoveGroupState move_state_{ move_group::IDLE };
  std::unique_ptr<trapezoidal_trajectory_generation::CommandListManager> command_list_manager_;
};
}  // namespace trapezoidal_trajectory_generation
