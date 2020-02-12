/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SEQUENCE_ACTION_CAPABILITY_H
#define SEQUENCE_ACTION_CAPABILITY_H

#include <memory>

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>

#include <pilz_msgs/MoveGroupSequenceAction.h>

namespace pilz_trajectory_generation
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

  using StartStateMsgs = pilz_msgs::MoveGroupSequenceResult::_trajectory_start_type;
  using PlannedTrajMsgs = pilz_msgs::MoveGroupSequenceResult::_planned_trajectory_type;

private:
  void executeSequenceCallback(const pilz_msgs::MoveGroupSequenceGoalConstPtr &goal);
  void executeSequenceCallbackPlanAndExecute(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                              pilz_msgs::MoveGroupSequenceResult& action_res);
  void executeMoveCallbackPlanOnly(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                    pilz_msgs::MoveGroupSequenceResult& res);
  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(move_group::MoveGroupState state);
  bool planUsingSequenceManager(const pilz_msgs::MotionSequenceRequest &req,
                                plan_execution::ExecutableMotionPlan& plan);

private:
  static void convertToMsg(const ExecutableTrajs& trajs,
                           StartStateMsgs& startStatesMsgs,
                           PlannedTrajMsgs& plannedTrajsMsgs);

private:
  std::unique_ptr<actionlib::SimpleActionServer<pilz_msgs::MoveGroupSequenceAction> > move_action_server_;
  pilz_msgs::MoveGroupSequenceFeedback move_feedback_;

  move_group::MoveGroupState move_state_ {move_group::IDLE};
  std::unique_ptr<pilz_trajectory_generation::CommandListManager> command_list_manager_;

};
}

#endif // SEQUENCE_ACTION_CAPABILITY_H
