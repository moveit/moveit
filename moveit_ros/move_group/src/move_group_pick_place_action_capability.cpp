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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/move_group/names.h>
#include <moveit/move_group/move_group_pick_place_action_capability.h>

move_group::MoveGroupPickPlaceAction::MoveGroupPickPlaceAction(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                                                               const pick_place::PickPlacePtr &pick_place,
                                                               const plan_execution::PlanExecutionPtr &plan_execution,
                                                               const plan_execution::PlanWithSensingPtr &plan_with_sensing,
                                                               bool allow_trajectory_execution,
                                                               bool debug) : 
  MoveGroupCapability(psm, debug),
  pick_place_(pick_place),
  plan_execution_(plan_execution),
  plan_with_sensing_(plan_with_sensing),
  allow_trajectory_execution_(allow_trajectory_execution),
  pickup_state_(IDLE)
{
  // start the pickup action server
  pickup_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(root_node_handle_, PICKUP_ACTION,
                                                                                           boost::bind(&MoveGroupPickPlaceAction::executePickupCallback, this, _1), false));
  pickup_action_server_->registerPreemptCallback(boost::bind(&MoveGroupPickPlaceAction::preemptPickupCallback, this));
  pickup_action_server_->start();
  
  // start the place action server
  place_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(root_node_handle_, PLACE_ACTION,
                                                                                         boost::bind(&MoveGroupPickPlaceAction::executePlaceCallback, this, _1), false));
  place_action_server_->registerPreemptCallback(boost::bind(&MoveGroupPickPlaceAction::preemptPlaceCallback, this));
  place_action_server_->start();
}

void move_group::MoveGroupPickPlaceAction::startPickupExecutionCallback()
{
  setPickupState(MONITOR);
}

void move_group::MoveGroupPickPlaceAction::startPickupLookCallback()
{
  setPickupState(LOOK);
}

void move_group::MoveGroupPickPlaceAction::executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
{ 
  pick_place::PickPlanPtr plan; 
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    plan = pick_place_->planPick(ps, *goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Pick&place threw an exception: %s", ex.what()); 
  }
  catch(...)
  {
    ROS_ERROR("Pick&place threw an exception");
  }
  
  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {  
      action_res.error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      convertToMsg(result->trajectories_, action_res.trajectory_start, action_res.trajectory_stages);
      action_res.trajectory_descriptions = result->trajectory_descriptions_;
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

bool move_group::MoveGroupPickPlaceAction::planUsingPickPlace(const moveit_msgs::PickupGoal& goal, plan_execution::ExecutableMotionPlan &plan)
{
  setPickupState(PLANNING);
  
  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);
  
  pick_place::PickPlanPtr pick_plan;
  try
  {
    pick_plan = pick_place_->planPick(plan.planning_scene_, goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Pick&place threw an exception: %s", ex.what()); 
  }
  catch(...)
  {
    ROS_ERROR("Pick&place threw an exception");
  }
  
  if (pick_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = pick_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = pick_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      plan.planned_trajectory_ = result->trajectories_;
      plan.planned_trajectory_descriptions_ = result->trajectory_descriptions_; 
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  
  return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void move_group::MoveGroupPickPlaceAction::executePickupCallback_PlanAndExecute(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
{
  plan_execution::PlanExecution::Options opt;
  
  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.before_execution_callback_ = boost::bind(&MoveGroupPickPlaceAction::startPickupExecutionCallback, this);
  
  opt.plan_callback_ = boost::bind(&MoveGroupPickPlaceAction::planUsingPickPlace, this, boost::cref(*goal), _1);
  if (goal->planning_options.look_around && plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, plan_with_sensing_.get(), _1, opt.plan_callback_,
                                     goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
    plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupPickPlaceAction::startPickupLookCallback, this));
  }
  
  plan_execution::ExecutableMotionPlan plan;
  plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);  
  
  convertToMsg(plan.planned_trajectory_, action_res.trajectory_start, action_res.trajectory_stages);
  action_res.trajectory_descriptions = plan.planned_trajectory_descriptions_; 
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupPickPlaceAction::executePickupCallback(const moveit_msgs::PickupGoalConstPtr& goal)
{
  setPickupState(PLANNING);
  
  planning_scene_monitor_->updateFrameTransforms();
  
  moveit_msgs::PickupResult action_res;
  
  if (goal->planning_options.plan_only || !allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
    executePickupCallback_PlanOnly(goal, action_res);
  }
  else
    executePickupCallback_PlanAndExecute(goal, action_res);
  
  bool planned_trajectory_empty = action_res.trajectory_stages.empty();
  std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    pickup_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      pickup_action_server_->setPreempted(action_res, response);
    else 
      pickup_action_server_->setAborted(action_res, response);
  }
  
  setPickupState(IDLE);
}  

void move_group::MoveGroupPickPlaceAction::executePlaceCallback(const moveit_msgs::PlaceGoalConstPtr& goal)
{
  ROS_ERROR("Not implemented yet");
}

void move_group::MoveGroupPickPlaceAction::preemptPickupCallback()
{
}

void move_group::MoveGroupPickPlaceAction::preemptPlaceCallback()
{
}

void move_group::MoveGroupPickPlaceAction::setPickupState(MoveGroupState state)
{  
  pickup_state_ = state;
  pickup_feedback_.state = stateToStr(state);
  pickup_action_server_->publishFeedback(pickup_feedback_);
}
 
void move_group::MoveGroupPickPlaceAction::setPlaceState(MoveGroupState state)
{  
  place_state_ = state;
  place_feedback_.state = stateToStr(state);
  place_action_server_->publishFeedback(place_feedback_);
}
