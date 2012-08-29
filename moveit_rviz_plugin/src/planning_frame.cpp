/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#include "moveit_rviz_plugin/planning_frame.h"
#include "moveit_rviz_plugin/planning_display.h"
#include <rviz/display_context.h>
#include "ui_moveit_rviz_plugin_frame.h"
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>

moveit_rviz_plugin::PlanningFrame::PlanningFrame(PlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent) :
  QWidget(parent),
  planning_display_(pdisplay),
  context_(context),
  ui_(new Ui::MotionPlanningFrame())
{
  ui_->setupUi(this);
  connect( ui_->plan_button, SIGNAL( clicked() ), this, SLOT( planButtonClicked() ));
  ui_->tabWidget->setCurrentIndex(0);
}

moveit_rviz_plugin::PlanningFrame::~PlanningFrame(void)
{
}

void moveit_rviz_plugin::PlanningFrame::enable(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor)
{  
  if (planning_scene_monitor)
  {
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor));
    plan_execution_->getPlanningPipeline().displayComputedMotionPlans(false);
    plan_execution_->getPlanningPipeline().checkSolutionPaths(true); 
    plan_execution_->displayCostSources(true);
  }
  show();
}

void moveit_rviz_plugin::PlanningFrame::disable(void)
{
  plan_execution_.reset();
  hide();
}

void moveit_rviz_plugin::PlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = 1;
  mreq.allowed_planning_time = ros::Duration(5.0);
  planning_models::kinematicStateToRobotState(*planning_display_->getQueryStartState(), mreq.start_state);
  
  const planning_models::KinematicState::JointStateGroup *jsg = planning_display_->getQueryGoalState()->getJointStateGroup(mreq.group_name);
  if (jsg)
  {
    mreq.goal_constraints.resize(1);
    mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(jsg);
  }
}

void moveit_rviz_plugin::PlanningFrame::planButtonClicked(void)
{
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  plan_execution_->planOnly(mreq);
  const plan_execution::PlanExecution::Result &res = plan_execution_->getLastResult();
  if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    planning_models::KinematicStatePtr start_state(new planning_models::KinematicState(plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));
    planning_models::robotStateToKinematicState(*plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getTransforms(), res.trajectory_start_, *start_state);
    planning_display_->displayRobotTrajectory(start_state, res.planned_trajectory_states_);
  }
}

