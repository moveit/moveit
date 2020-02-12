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

// Modified by Pilz GmbH & Co. KG

#include "pilz_trajectory_generation/move_group_sequence_action.h"

#include <time.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "pilz_trajectory_generation/command_list_manager.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

MoveGroupSequenceAction::MoveGroupSequenceAction()
  : MoveGroupCapability("SequenceAction")
{
}

void MoveGroupSequenceAction::initialize()
{
  // start the move action server
  ROS_INFO_STREAM("initialize move group sequence action");
  move_action_server_.reset( new actionlib::SimpleActionServer<pilz_msgs::MoveGroupSequenceAction>(
                               root_node_handle_, "sequence_move_group",
                               boost::bind(&MoveGroupSequenceAction::executeSequenceCallback, this, _1), false) );
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupSequenceAction::preemptMoveCallback, this));
  move_action_server_->start();

  command_list_manager_.reset(new pilz_trajectory_generation::CommandListManager (
                            ros::NodeHandle("~"), context_->planning_scene_monitor_->getRobotModel()));

}

void MoveGroupSequenceAction::executeSequenceCallback(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal)
{
  setMoveState(move_group::PLANNING);

  // Handle empty requests
  if(goal->request.items.empty())
  {
    ROS_WARN("Received empty request. That's ok but maybe not what you intended.");
    setMoveState(move_group::IDLE);
    pilz_msgs::MoveGroupSequenceResult action_res;
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    move_action_server_->setSucceeded(action_res, "Received empty request.");
    return;
  }

  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  pilz_msgs::MoveGroupSequenceResult action_res;
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
    {
      ROS_WARN("Only plan will be calculated, although plan_only == false."); //LCOV_EXCL_LINE
    }
    executeMoveCallbackPlanOnly(goal, action_res);
  }
  else
  {
    executeSequenceCallbackPlanAndExecute(goal, action_res);
  }

  switch(action_res.error_code.val)
  {
  case moveit_msgs::MoveItErrorCodes::SUCCESS:
    move_action_server_->setSucceeded(action_res, "Success");
    break;
  case moveit_msgs::MoveItErrorCodes::PREEMPTED:
    move_action_server_->setPreempted(action_res, "Preempted");
    break;
  default:
    move_action_server_->setAborted(action_res, "See error code for more information");
    break;
  }

  setMoveState(move_group::IDLE);
}

void MoveGroupSequenceAction::executeSequenceCallbackPlanAndExecute(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                                                     pilz_msgs::MoveGroupSequenceResult& action_res)
{
  ROS_INFO("Combined planning and execution request received for MoveGroupSequenceAction.");

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::PlanningScene& planning_scene_diff =
      planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
        goal->planning_options.planning_scene_diff :
        clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupSequenceAction::startMoveExecutionCallback, this);

  opt.plan_callback_ =
      boost::bind(&MoveGroupSequenceAction::planUsingSequenceManager, this, boost::cref(goal->request), _1);

  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    ROS_WARN("Plan with sensing not yet implemented/tested. This option is ignored."); //LCOV_EXCL_LINE
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = plan.error_code_;
}

void MoveGroupSequenceAction::convertToMsg(const ExecutableTrajs& trajs,
                                           StartStateMsgs& startStatesMsgs,
                                           PlannedTrajMsgs& plannedTrajsMsgs)
{
  startStatesMsgs.resize(trajs.size());
  plannedTrajsMsgs.resize(trajs.size());
  for(size_t i = 0; i < trajs.size(); ++i)
  {
    robot_state::robotStateToRobotStateMsg(trajs.at(i).trajectory_->getFirstWayPoint(), startStatesMsgs.at(i));
    trajs.at(i).trajectory_->getRobotTrajectoryMsg(plannedTrajsMsgs.at(i));
  }
}

void MoveGroupSequenceAction::executeMoveCallbackPlanOnly(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                                           pilz_msgs::MoveGroupSequenceResult& res)
{
  ROS_INFO("Planning request received for MoveGroupSequenceAction action.");

  // lock the scene so that it does not modify the world representation while diff() is called
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  const planning_scene::PlanningSceneConstPtr& the_scene =
      (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
        static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
        lscene->diff(goal->planning_options.planning_scene_diff);

  ros::Time planning_start = ros::Time::now();
  RobotTrajCont traj_vec;
  try
  {
    traj_vec = command_list_manager_->solve(the_scene, context_->planning_pipeline_, goal->request);
  }
  catch(const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM("Planning pipeline threw an exception (error code: "
                     << ex.getErrorCode() << "): " << ex.what());
    res.error_code.val = ex.getErrorCode();
    return;
  }
  // LCOV_EXCL_START // Keep moveit up even if lower parts throw
  catch (const std::exception& ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return;
  }
  // LCOV_EXCL_STOP

  res.trajectory_start.resize(traj_vec.size());
  res.planned_trajectory.resize(traj_vec.size());
  for(RobotTrajCont::size_type i = 0; i < traj_vec.size(); ++i)
  {
    move_group::MoveGroupCapability::convertToMsg(traj_vec.at(i),
                                                  res.trajectory_start.at(i),
                                                  res.planned_trajectory.at(i));
  }
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time = (ros::Time::now() - planning_start).toSec();
}

bool MoveGroupSequenceAction::planUsingSequenceManager(const pilz_msgs::MotionSequenceRequest& req,
                                                       plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(move_group::PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  RobotTrajCont traj_vec;
  try { traj_vec = command_list_manager_->solve(plan.planning_scene_, context_->planning_pipeline_, req); }
  catch(const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM("Planning pipeline threw an exception (error code: "
                     << ex.getErrorCode() << "): " << ex.what());
    plan.error_code_.val = ex.getErrorCode();
    return false;
  }
  // LCOV_EXCL_START // Keep moveit up even if lower parts throw
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Planning pipeline threw an exception: " << ex.what());
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }
  // LCOV_EXCL_STOP

  if (!traj_vec.empty())
  {
    plan.plan_components_.resize(traj_vec.size());
    for (size_t i = 0; i<traj_vec.size(); ++i)
    {
      plan.plan_components_.at(i).trajectory_ = traj_vec.at(i);
      plan.plan_components_.at(i).description_ = "plan";
    }
  }
  plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

void MoveGroupSequenceAction::startMoveExecutionCallback()
{
  setMoveState(move_group::MONITOR);
}

void MoveGroupSequenceAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void MoveGroupSequenceAction::setMoveState(move_group::MoveGroupState state)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  move_action_server_->publishFeedback(move_feedback_);
}


} // namespace pilz_trajectory_generation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pilz_trajectory_generation::MoveGroupSequenceAction, move_group::MoveGroupCapability)

