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

#include "move_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupMoveAction::MoveGroupMoveAction() : MoveGroupCapability("MoveAction"), move_state_(IDLE)
{
}

void move_group::MoveGroupMoveAction::initialize()
{
  // start the move action server
  move_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(
      root_node_handle_, MOVE_ACTION, boost::bind(&MoveGroupMoveAction::executeMoveCallback, this, _1), false));
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupMoveAction::preemptMoveCallback, this));
  move_action_server_->start();
}

void move_group::MoveGroupMoveAction::executeMoveCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
{
  setMoveState(PLANNING);
  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit_msgs::MoveGroupResult action_res;
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only "
               "set to false. Only a motion plan will be computed anyway.");
    executeMoveCallback_PlanOnly(goal, action_res);
  }
  else
    executeMoveCallback_PlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
  std::string response =
      getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    move_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      move_action_server_->setPreempted(action_res, response);
    else
      move_action_server_->setAborted(action_res, response);
  }

  setMoveState(IDLE);
}

void move_group::MoveGroupMoveAction::executeMoveCallback_PlanAndExecute(const moveit_msgs::MoveGroupGoalConstPtr& goal,
                                                                         moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO("Combined planning and execution request received for MoveGroup action. Forwarding to planning and "
           "execution pipeline.");

  if (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const robot_state::RobotState& current_state = lscene->getCurrentState();

    // check to see if the desired constraints are already met
    for (std::size_t i = 0; i < goal->request.goal_constraints.size(); ++i)
      if (lscene->isStateConstrained(current_state,
                                     kinematic_constraints::mergeConstraints(goal->request.goal_constraints[i],
                                                                             goal->request.path_constraints)))
      {
        ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return;
      }
  }

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest& motion_plan_request =
      planning_scene::PlanningScene::isEmpty(goal->request.start_state) ? goal->request :
                                                                          clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene& planning_scene_diff =
      planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupMoveAction::startMoveExecutionCallback, this);

  opt.plan_callback_ =
      boost::bind(&MoveGroupMoveAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                                     _1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                                     goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupMoveAction::startMoveLookCallback, this));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupMoveAction::executeMoveCallback_PlanOnly(const moveit_msgs::MoveGroupGoalConstPtr& goal,
                                                                   moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO("Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);  // lock the scene so that it
                                                                                            // does not modify the world
                                                                                            // representation while
                                                                                            // diff() is called
  const planning_scene::PlanningSceneConstPtr& the_scene =
      (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
          static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
          lscene->diff(goal->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;
  try
  {
    context_->planning_pipeline_->generatePlan(the_scene, goal->request, res);
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch (...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

bool move_group::MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                                plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  bool solved = false;
  planning_interface::MotionPlanResponse res;
  try
  {
    solved = context_->planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch (...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void move_group::MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR);
}

void move_group::MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK);
}

void move_group::MoveGroupMoveAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void move_group::MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  move_action_server_->publishFeedback(move_feedback_);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)
