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

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::planButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this));
}

void MotionPlanningFrame::executeButtonClicked(void)
{
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));
}

void MotionPlanningFrame::planAndExecuteButtonClicked(void)
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanAndExecuteButtonClicked, this));
}

void MotionPlanningFrame::randomStatesButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeRandomStatesButtonClicked, this));
}

void MotionPlanningFrame::setStartToCurrentButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSetStartToCurrentButtonClicked, this));
}

void MotionPlanningFrame::setGoalToCurrentButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSetGoalToCurrentButtonClicked, this));
}

void MotionPlanningFrame::allowReplanningToggled(bool checked)
{
  if (move_group_)
    move_group_->allowReplanning(checked);
}

void MotionPlanningFrame::allowLookingToggled(bool checked)
{
  if (move_group_)
    move_group_->allowLooking(checked);
}

void MotionPlanningFrame::pathConstraintsIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPathConstraints(ui_->path_constraints_combo_box->itemText(index).toStdString());
    else
      move_group_->clearPathConstraints();
  }
}

void MotionPlanningFrame::computePlanButtonClicked(void)
{
  if (!move_group_)
    return;
  configureForPlanning();
  current_plan_.reset(new move_group_interface::MoveGroup::Plan());
  if (move_group_->plan(*current_plan_))
    ui_->execute_button->setEnabled(true);
  else
    current_plan_.reset();
}

void MotionPlanningFrame::computeExecuteButtonClicked(void)
{
  if (move_group_ && current_plan_)
    move_group_->execute(*current_plan_);
}

void MotionPlanningFrame::computePlanAndExecuteButtonClicked(void)
{
  if (!move_group_)
    return;
  configureForPlanning();
  move_group_->move();
  ui_->plan_and_execute_button->setEnabled(true);
}

void MotionPlanningFrame::computeSetStartToCurrentButtonClicked(void)
{
  const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    const kinematic_state::KinematicState &s = ps->getCurrentState();
    planning_display_->setQueryStartState(kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(s)));
  }
}

void MotionPlanningFrame::computeSetGoalToCurrentButtonClicked(void)
{
  const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    const kinematic_state::KinematicState &s = ps->getCurrentState();
    planning_display_->setQueryGoalState(kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(s)));
  }
}

void MotionPlanningFrame::computeRandomStatesButtonClicked(void)
{
  std::string group_name = planning_display_->getCurrentPlanningGroup();

  if (planning_display_->getQueryStartState())
  {
    kinematic_state::KinematicStatePtr start(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
    kinematic_state::JointStateGroup *jsg = start->getJointStateGroup(group_name);
    if (jsg)
      jsg->setToRandomValues();
    planning_display_->setQueryStartState(start);
  }

  if (planning_display_->getQueryGoalState())
  {
    kinematic_state::KinematicStatePtr goal(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
    kinematic_state::JointStateGroup *jsg = goal->getJointStateGroup(group_name);
    if (jsg)
      jsg->setToRandomValues();
    planning_display_->setQueryGoalState(goal);
  }
}

void MotionPlanningFrame::populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc)
{
  std::string group = planning_display_->getCurrentPlanningGroup();
  ui_->planning_algorithm_combo_box->clear();

  // set the label for the planning library
  ui_->library_label->setText(QString::fromStdString(desc.name));
  ui_->library_label->setStyleSheet("QLabel { color : green; font: bold }");

  bool found_group = false;
  // the name of a planner is either "GROUP[planner_id]" or "planner_id"
  if (!group.empty())
    for (std::size_t i = 0 ; i < desc.planner_ids.size() ; ++i)
      if (desc.planner_ids[i] == group)
        found_group = true;
      else
        if (desc.planner_ids[i].substr(0, group.length()) == group)
        {
          std::string id = desc.planner_ids[i].substr(group.length());
          if (id.size() > 2)
          {
            id.resize(id.length() - 1);
            ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(id.substr(1)));
          }
        }
  if (ui_->planning_algorithm_combo_box->count() == 0 && !found_group)
    for (std::size_t i = 0 ; i < desc.planner_ids.size() ; ++i)
      ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(desc.planner_ids[i]));
  ui_->planning_algorithm_combo_box->insertItem(0, "<unspecified>");
  ui_->planning_algorithm_combo_box->setCurrentIndex(0);
}

void MotionPlanningFrame::populateConstraintsList(void)
{
  if (move_group_)
  {
    // add some artificial wait time (but in the background) for the constraints DB to connect
    double dt = (ros::WallTime::now() - move_group_construction_time_).toSec();
    if (dt < 0.2)
      ros::WallDuration(0.1).sleep();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this, move_group_->getKnownConstraints()));
  }
}

void MotionPlanningFrame::populateConstraintsList(const std::vector<std::string> &constr)
{
  ui_->path_constraints_combo_box->clear();
  ui_->path_constraints_combo_box->addItem("None");
  for (std::size_t i = 0 ; i < constr.size() ; ++i)
    ui_->path_constraints_combo_box->addItem(QString::fromStdString(constr[i]));
}

void MotionPlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = 1;
  mreq.allowed_planning_time = ros::Duration(ui_->planning_time->value());
  kinematic_state::kinematicStateToRobotState(*planning_display_->getQueryStartState(), mreq.start_state);
  mreq.workspace_parameters.min_corner.x = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.min_corner.y = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.min_corner.z = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  mreq.workspace_parameters.max_corner.x = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.max_corner.y = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.max_corner.z = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  const kinematic_state::JointStateGroup *jsg = planning_display_->getQueryGoalState()->getJointStateGroup(mreq.group_name);
  if (jsg)
  {
    mreq.goal_constraints.resize(1);
    mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(jsg);
  }
}

void MotionPlanningFrame::configureForPlanning(void)
{
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->setPlanningTime(ui_->planning_time->value());
  move_group_->setWorkspace(ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0,
                            ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0);
}

}
