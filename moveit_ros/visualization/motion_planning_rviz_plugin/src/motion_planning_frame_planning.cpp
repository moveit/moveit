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

#include <std_srvs/Empty.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::planButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this), "compute "
                                                                                                         "plan");
}

void MotionPlanningFrame::executeButtonClicked()
{
  ui_->execute_button->setEnabled(false);
  // execution is done in a separate thread, to not block other background jobs by blocking for synchronous execution
  planning_display_->spawnBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));
}

void MotionPlanningFrame::planAndExecuteButtonClicked()
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  // execution is done in a separate thread, to not block other background jobs by blocking for synchronous execution
  planning_display_->spawnBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanAndExecuteButtonClicked, this));
}

void MotionPlanningFrame::stopButtonClicked()
{
  ui_->stop_button->setEnabled(false);  // avoid clicking again
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeStopButtonClicked, this), "stop");
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

void MotionPlanningFrame::onClearOctomapClicked()
{
  std_srvs::Empty srv;
  clear_octomap_service_client_.call(srv);
}

void MotionPlanningFrame::computePlanButtonClicked()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->result_label->setText("Planning...");

  configureForPlanning();
  current_plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
  if (move_group_->plan(*current_plan_))
  {
    ui_->execute_button->setEnabled(true);

    // Success
    ui_->result_label->setText(QString("Time: ").append(QString::number(current_plan_->planning_time_, 'f', 3)));
  }
  else
  {
    current_plan_.reset();

    // Failure
    ui_->result_label->setText("Failed");
  }
  Q_EMIT planningFinished();
}

void MotionPlanningFrame::computeExecuteButtonClicked()
{
  if (move_group_ && current_plan_)
  {
    ui_->stop_button->setEnabled(true);  // enable stopping
    bool success = move_group_->execute(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    onFinishedExecution(success);
  }
}

void MotionPlanningFrame::computePlanAndExecuteButtonClicked()
{
  if (!move_group_)
    return;
  configureForPlanning();
  // move_group::move() on the server side, will always start from the current state
  // to suppress a warning, we pass an empty state (which encodes "start from current state")
  move_group_->setStartStateToCurrentState();
  ui_->stop_button->setEnabled(true);
  bool success = move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  onFinishedExecution(success);
  ui_->plan_and_execute_button->setEnabled(true);
}

void MotionPlanningFrame::computeStopButtonClicked()
{
  if (move_group_)
    move_group_->stop();
}

void MotionPlanningFrame::onFinishedExecution(bool success)
{
  // visualize result of execution
  if (success)
    ui_->result_label->setText("Executed");
  else
    ui_->result_label->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
  // disable stop button
  ui_->stop_button->setEnabled(false);

  // update query start state to current if neccessary
  if (ui_->start_state_selection->currentText() == "<current>")
    useStartStateButtonClicked();
}

void MotionPlanningFrame::useStartStateButtonClicked()
{
  // use background job: fetching the current state might take up to a second
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::useStartStateButtonExec, this),  //
                                      "update start state");
}

void MotionPlanningFrame::useStartStateButtonExec()
{
  robot_state::RobotState start = *planning_display_->getQueryStartState();
  updateQueryStateHelper(start, ui_->start_state_selection->currentText().toStdString());
  planning_display_->setQueryStartState(start);
}

void MotionPlanningFrame::useGoalStateButtonClicked()
{
  // use background job: fetching the current state might take up to a second
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::useGoalStateButtonExec, this),  //
                                      "update goal state");
}

void MotionPlanningFrame::useGoalStateButtonExec()
{
  robot_state::RobotState goal = *planning_display_->getQueryGoalState();
  updateQueryStateHelper(goal, ui_->goal_state_selection->currentText().toStdString());
  planning_display_->setQueryGoalState(goal);
}

void MotionPlanningFrame::updateQueryStateHelper(robot_state::RobotState& state, const std::string& v)
{
  if (v == "<random>")
  {
    configureWorkspace();
    if (const robot_model::JointModelGroup* jmg =
            state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
      state.setToRandomPositions(jmg);
    return;
  }

  if (v == "<random valid>")
  {
    configureWorkspace();

    if (const robot_model::JointModelGroup* jmg =
            state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
    {
      // Loop until a collision free state is found
      static const int MAX_ATTEMPTS = 100;
      int attempt_count = 0;  // prevent loop for going forever
      while (attempt_count < MAX_ATTEMPTS)
      {
        // Generate random state
        state.setToRandomPositions(jmg);

        state.update();  // prevent dirty transforms

        // Test for collision
        if (planning_display_->getPlanningSceneRO()->isStateValid(state, "", false))
          break;

        attempt_count++;
      }
      // Explain if no valid rand state found
      if (attempt_count >= MAX_ATTEMPTS)
        ROS_WARN("Unable to find a random collision free configuration after %d attempts", MAX_ATTEMPTS);
    }
    else
    {
      ROS_WARN_STREAM("Unable to get joint model group " << planning_display_->getCurrentPlanningGroup());
    }
    return;
  }

  if (v == "<current>")
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
      state = ps->getCurrentState();
    return;
  }

  if (v == "<same as goal>")
  {
    state = *planning_display_->getQueryGoalState();
    return;
  }

  if (v == "<same as start>")
  {
    state = *planning_display_->getQueryStartState();
    return;
  }

  // maybe it is a named state
  if (const robot_model::JointModelGroup* jmg = state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
    state.setToDefaultValues(jmg, v);
}

void MotionPlanningFrame::populatePlannersList(const moveit_msgs::PlannerInterfaceDescription& desc)
{
  std::string group = planning_display_->getCurrentPlanningGroup();
  ui_->planning_algorithm_combo_box->clear();

  // set the label for the planning library
  ui_->library_label->setText(QString::fromStdString(desc.name));
  ui_->library_label->setStyleSheet("QLabel { color : green; font: bold }");

  bool found_group = false;
  // the name of a planner is either "GROUP[planner_id]" or "planner_id"
  if (!group.empty())
    for (std::size_t i = 0; i < desc.planner_ids.size(); ++i)
      if (desc.planner_ids[i] == group)
        found_group = true;
      else if (desc.planner_ids[i].substr(0, group.length()) == group)
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
    for (std::size_t i = 0; i < desc.planner_ids.size(); ++i)
      ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(desc.planner_ids[i]));
  ui_->planning_algorithm_combo_box->insertItem(0, "<unspecified>");

  // retrieve default planner config from parameter server
  const std::string& default_planner_config = move_group_->getDefaultPlannerId(found_group ? group : std::string());
  int defaultIndex = ui_->planning_algorithm_combo_box->findText(QString::fromStdString(default_planner_config));
  if (defaultIndex < 0)
    defaultIndex = 0;  // 0 is <unspecified> fallback
  ui_->planning_algorithm_combo_box->setCurrentIndex(defaultIndex);
}

void MotionPlanningFrame::populateConstraintsList()
{
  if (move_group_)
    planning_display_->addMainLoopJob(
        boost::bind(&MotionPlanningFrame::populateConstraintsList, this, move_group_->getKnownConstraints()));
}

void MotionPlanningFrame::populateConstraintsList(const std::vector<std::string>& constr)
{
  ui_->path_constraints_combo_box->clear();
  ui_->path_constraints_combo_box->addItem("None");
  for (std::size_t i = 0; i < constr.size(); ++i)
    ui_->path_constraints_combo_box->addItem(QString::fromStdString(constr[i]));
}

void MotionPlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = ui_->planning_attempts->value();
  mreq.allowed_planning_time = ui_->planning_time->value();
  mreq.max_velocity_scaling_factor = ui_->velocity_scaling_factor->value();
  mreq.max_acceleration_scaling_factor = ui_->acceleration_scaling_factor->value();
  robot_state::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), mreq.start_state);
  mreq.workspace_parameters.min_corner.x = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.min_corner.y = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.min_corner.z = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  mreq.workspace_parameters.max_corner.x = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.max_corner.y = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.max_corner.z = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  robot_state::RobotStateConstPtr s = planning_display_->getQueryGoalState();
  const robot_state::JointModelGroup* jmg = s->getJointModelGroup(mreq.group_name);
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
    move_group_->setWorkspace(bx.min_position_, by.min_position_, bz.min_position_, bx.max_position_, by.max_position_,
                              bz.max_position_);
  planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_display_->getPlanningSceneMonitor();
  // get non-const access to the kmodel and update planar & floating joints as indicated by the workspace settings
  if (psm && psm->getRobotModelLoader() && psm->getRobotModelLoader()->getModel())
  {
    const robot_model::RobotModelPtr& kmodel = psm->getRobotModelLoader()->getModel();
    const std::vector<robot_model::JointModel*>& jm = kmodel->getJointModels();
    for (std::size_t i = 0; i < jm.size(); ++i)
      if (jm[i]->getType() == robot_model::JointModel::PLANAR)
      {
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[0], bx);
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[1], by);
      }
      else if (jm[i]->getType() == robot_model::JointModel::FLOATING)
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
  move_group_->setNumPlanningAttempts(ui_->planning_attempts->value());
  move_group_->setMaxVelocityScalingFactor(ui_->velocity_scaling_factor->value());
  move_group_->setMaxAccelerationScalingFactor(ui_->acceleration_scaling_factor->value());
  configureWorkspace();
  if (static_cast<bool>(planning_display_))
    planning_display_->dropVisualizedTrajectory();
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
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      robot_state::RobotState state = ps->getCurrentState();
      planning_display_->setQueryStartState(state);
    }
  }
}

void MotionPlanningFrame::remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg)
{
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      robot_state::RobotState state = ps->getCurrentState();
      planning_display_->setQueryGoalState(state);
    }
  }
}
}
