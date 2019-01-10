/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CITEC, Bielefeld University
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
 *   * Neither the name of CITEC / Bielefeld University nor the names of
 *     its contributors may be used to endorse or promote products derived
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

/* Author: Robert Haschke */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include "ui_motion_planning_rviz_plugin_frame_joints.h"

namespace moveit_rviz_plugin
{
JMGItemModel::JMGItemModel(const moveit::core::RobotState& robot_state, const std::string group_name, QObject* parent)
  : QAbstractTableModel(parent), robot_state_(robot_state), jmg_(nullptr)
{
  if (robot_state_.getRobotModel()->hasJointModelGroup(group_name))
    jmg_ = robot_state_.getRobotModel()->getJointModelGroup(group_name);
}

int JMGItemModel::rowCount(const QModelIndex& parent) const
{
  if (!jmg_)
    return robot_state_.getVariableCount();
  else
    return jmg_->getVariableCount();
}

int JMGItemModel::columnCount(const QModelIndex& parent) const
{
  return 2;
}

QVariant JMGItemModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
    return QVariant();

  int idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  int col = index.column();
  switch (role)
  {
    case Qt::DisplayRole:
      if (col == 0)
        return QString::fromStdString(robot_state_.getVariableNames()[idx]);
      else
        return robot_state_.getVariablePosition(idx);
  }
  return QVariant();
}

QVariant JMGItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    return section == 0 ? "Joint Name" : "Value";
  return QAbstractTableModel::headerData(section, orientation, role);
}

void JMGItemModel::stateChanged(const moveit::core::RobotState& state)
{
  if (robot_state_.getRobotModel() != state.getRobotModel())
    return;
  robot_state_.setVariablePositions(state.getVariablePositions());

  if (!jmg_)  // update whole value column
    dataChanged(index(0, 1), index(rowCount() - 1, 1));

  // for JMG only update affected variables
  int first = -1, last = -2;
  for (int idx : jmg_->getVariableIndexList())
  {
    if (++last == idx)
      continue;  // extend continuous range
    if (first >= 0)
      dataChanged(index(first, 1), index(last - 1, 1));
    first = last = idx;
  }
  if (first >= 0)
    dataChanged(index(first, 1), index(last - 1, 1));
}

MotionPlanningFrameJointsWidget::MotionPlanningFrameJointsWidget(QWidget* parent)
  : QWidget(parent), ui_(new Ui::MotionPlanningFrameJointsUI())
{
  ui_->setupUi(this);
}

MotionPlanningFrameJointsWidget::~MotionPlanningFrameJointsWidget()
{
  delete ui_;
}

void MotionPlanningFrameJointsWidget::changePlanningGroup(
    const std::string& group_name, const robot_interaction::InteractionHandlerPtr& start_state_handler,
    const robot_interaction::InteractionHandlerPtr& goal_state_handler)
{
  // release previous models (if any)
  ui_->joints_view_->setModel(nullptr);
  start_state_model_.reset();
  goal_state_model_.reset();

  // create new models
  start_state_handler_ = start_state_handler;
  goal_state_handler_ = goal_state_handler;
  start_state_model_.reset(new JMGItemModel(*start_state_handler_->getState(), group_name, this));
  goal_state_model_.reset(new JMGItemModel(*goal_state_handler_->getState(), group_name, this));
  ui_->joints_view_->setModel(goal_state_model_.get());
}

void MotionPlanningFrameJointsWidget::queryStartStateChanged()
{
  if (!start_state_model_ || !start_state_handler_)
    return;
  start_state_model_->stateChanged(*start_state_handler_->getState());
  ui_->joints_view_->setModel(start_state_model_.get());
  ui_->joints_view_label_->setText("Group joints of start state");
}

void MotionPlanningFrameJointsWidget::queryGoalStateChanged()
{
  if (!goal_state_model_ || !goal_state_handler_)
    return;
  goal_state_model_->stateChanged(*goal_state_handler_->getState());
  ui_->joints_view_->setModel(goal_state_model_.get());
  ui_->joints_view_label_->setText("Group joints of goal state");
}

}  // namespace
