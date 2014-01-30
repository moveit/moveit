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

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::planButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this), "compute plan");
}

void MotionPlanningFrame::executeButtonClicked()
{
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this), "execute");
}

void MotionPlanningFrame::planAndExecuteButtonClicked()
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanAndExecuteButtonClicked, this), "plan and execute");
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
    {
      std::string c = ui_->path_constraints_combo_box->itemText(index).toStdString();
      if (!move_group_->setPathConstraints(c))
        ROS_WARN_STREAM("Unable to set the path constraints: " << c);
    }
    else
      move_group_->clearPathConstraints();
  }
}

void MotionPlanningFrame::computePlanButtonClicked()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->result_label->setText("Planning...");

  configureForPlanning();
  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());
  if (move_group_->plan(*current_plan_))
  {
    ui_->execute_button->setEnabled(true);

    // Success
    ui_->result_label->setText(QString("Time: ").append(
        QString::number(current_plan_->planning_time_,'f',3)));
  }
  else
  {
    current_plan_.reset();

    // Failure
    ui_->result_label->setText("Failed");
  }
}

void MotionPlanningFrame::computeExecuteButtonClicked()
{
  if (move_group_ && current_plan_)
    move_group_->execute(*current_plan_);
}

void MotionPlanningFrame::computePlanAndExecuteButtonClicked()
{
  if (!move_group_)
    return;
  configureForPlanning();
  move_group_->move();
  ui_->plan_and_execute_button->setEnabled(true);
}

void MotionPlanningFrame::useStartStateButtonClicked()
{
  robot_state::RobotState start = *planning_display_->getQueryStartState();
  updateQueryStateHelper(start, ui_->start_state_selection->currentText().toStdString());
  planning_display_->setQueryStartState(start);
}

void MotionPlanningFrame::useGoalStateButtonClicked()
{
  robot_state::RobotState goal = *planning_display_->getQueryGoalState();
  updateQueryStateHelper(goal, ui_->goal_state_selection->currentText().toStdString());
  planning_display_->setQueryGoalState(goal);
}

void MotionPlanningFrame::updateQueryStateHelper(robot_state::RobotState &state, const std::string &v)
{
  if (v == "<random>")
  {
    configureWorkspace();
    if (const robot_model::JointModelGroup *jmg = state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
      state.setToRandomPositions(jmg);
  }
  else
    if (v == "<random valid>")
    {
      configureWorkspace();

      if (const robot_model::JointModelGroup *jmg =
        state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
      {
        // Loop until a collision free state is found
        static const int MAX_ATTEMPTS = 100;
        int attempt_count = 0; // prevent loop for going forever
        while (attempt_count < MAX_ATTEMPTS)
        {
          // Generate random state
          state.setToRandomPositions(jmg);

          state.update(); // prevent dirty transforms

          // Test for collision
          if (planning_display_->getPlanningSceneRO()->isStateValid(state, "", false))
            break;

          attempt_count ++;
        }
        // Explain if no valid rand state found
        if (attempt_count >= MAX_ATTEMPTS)
          ROS_WARN("Unable to find a random collision free configuration after %d attempts", MAX_ATTEMPTS);
      }
      else
      {
        ROS_WARN_STREAM("Unable to get joint model group " << planning_display_->getCurrentPlanningGroup());
      }
    }
    else
      if (v == "<current>")
      {
        const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
        if (ps)
          state = ps->getCurrentState();
      }
      else
        if (v == "<same as goal>")
        {
          state = *planning_display_->getQueryGoalState();
        }
        else
          if (v == "<same as start>")
          {
            state = *planning_display_->getQueryStartState();
          }
          else
          {
            // maybe it is a named state
            if (const robot_model::JointModelGroup *jmg = state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
              state.setToDefaultValues(jmg, v);
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
          if (desc.planner_ids[i].size() > group.length() && desc.planner_ids[i][group.length()] == '[')
          {
            std::string id = desc.planner_ids[i].substr(group.length());
            if (id.size() > 2)
            {
              id.resize(id.length() - 1);
              ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(id.substr(1)));
            }
          }
        }
  if (ui_->planning_algorithm_combo_box->count() == 0 && !found_group)
    for (std::size_t i = 0 ; i < desc.planner_ids.size() ; ++i)
      ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(desc.planner_ids[i]));
  ui_->planning_algorithm_combo_box->insertItem(0, "<unspecified>");
  ui_->planning_algorithm_combo_box->setCurrentIndex(0);
}

void MotionPlanningFrame::populateConstraintsList()
{
  if (move_group_)
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this, move_group_->getKnownConstraints()));
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
  mreq.allowed_planning_time = ui_->planning_time->value();
  robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), mreq.start_state);
  mreq.workspace_parameters.min_corner.x = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.min_corner.y = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.min_corner.z = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  mreq.workspace_parameters.max_corner.x = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.max_corner.y = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.max_corner.z = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  robot_state::RobotStateConstPtr s = planning_display_->getQueryGoalState();
  const robot_state::JointModelGroup *jmg = s->getJointModelGroup(mreq.group_name);
  if (jmg)
  {
    mreq.goal_constraints.resize(1);
    mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(*s, jmg);
  }
}

void MotionPlanningFrame::configureWorkspace()
{
  robot_model::VariableBounds bx, by, bz;
  bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;
  
  robot_model::JointModel::Bounds b(3);
  bx.min_position_ = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  bx.max_position_ = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  by.min_position_ = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  by.max_position_ = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  bz.min_position_ = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  bz.max_position_ = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  
  if (move_group_)
    move_group_->setWorkspace(bx.min_position_, by.min_position_, bz.min_position_,
                              bx.max_position_, by.max_position_, bz.max_position_);
  planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_display_->getPlanningSceneMonitor();
  // get non-const access to the kmodel and update planar & floating joints as indicated by the workspace settings
  if (psm && psm->getRobotModelLoader() && psm->getRobotModelLoader()->getModel())
  {
    const robot_model::RobotModelPtr &kmodel = psm->getRobotModelLoader()->getModel();
    const std::vector<robot_model::JointModel*> &jm = kmodel->getJointModels();
    for (std::size_t i = 0 ; i < jm.size() ; ++i)
      if (jm[i]->getType() == robot_model::JointModel::PLANAR)
      {
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[0], bx);
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[1], by);
      }
      else
        if (jm[i]->getType() == robot_model::JointModel::FLOATING)
        {
          jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[0], bx);
          jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[1], by);
          jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[2], bz);
        }
  }
}

void MotionPlanningFrame::configureForPlanning()
{
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->setPlanningTime(ui_->planning_time->value());
  configureWorkspace();
}

void MotionPlanningFrame::remotePlanCallback(const std_msgs::EmptyConstPtr& msg)
{
  planButtonClicked();
}

void MotionPlanningFrame::remoteExecuteCallback(const std_msgs::EmptyConstPtr& msg)
{
  executeButtonClicked();
}

void MotionPlanningFrame::remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& msg)
{
  if (move_group_ && planning_display_) {
    robot_state::RobotState state = *planning_display_->getQueryStartState();
    const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
    if (ps) {
      state = ps->getCurrentState();
      planning_display_->setQueryStartState(state);
    }
  }
}

void MotionPlanningFrame::remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg)
{
  if (move_group_ && planning_display_) {
    robot_state::RobotState state = *planning_display_->getQueryStartState();
    const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
    if (ps) {
      state = ps->getCurrentState();
      planning_display_->setQueryGoalState(state);
    }
  }
}

  
}
