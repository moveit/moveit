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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <QAbstractItemModel>
#include <QWidget>
#include <QStyledItemDelegate>
#include <memory>

namespace Ui
{
class MotionPlanningFrameJointsUI;
}
namespace robot_interaction
{
MOVEIT_CLASS_FORWARD(InteractionHandler)
}
namespace moveit_rviz_plugin
{
/** TableModel to display joint values of a referenced RobotState.
 *
 * Unfortunately we cannot store the RobotStatePtr (and thus ensure existence of the state during
 * the lifetime of this class instance), because RobotInteraction (which is the initial use case)
 * allocates internally a new RobotState if any other copy is held somewhere else.
 * Hence, we also store an (unsafe) raw pointer. Lifetime of this raw pointer needs to be ensured.
 */
class JMGItemModel : public QAbstractTableModel
{
  Q_OBJECT
  moveit::core::RobotState robot_state_;
  const moveit::core::JointModelGroup* jmg_;

public:
  JMGItemModel(const moveit::core::RobotState& robot_state, const std::string group_name, QObject* parent = nullptr);

  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index, int role) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

  const moveit::core::RobotState& getRobotState() const
  {
    return robot_state_;
  }
  /// retrieve the JointModel corresponding to the variable referenced by index
  const moveit::core::JointModel* getJointModel(const QModelIndex& index) const;
  /// retrieve the variable bounds referenced by variable index
  const moveit::core::VariableBounds* getVariableBounds(const QModelIndex& index) const;
  /// call this on any change of the RobotState
  void stateChanged(const moveit::core::RobotState& state);
};

class MotionPlanningFrameJointsWidget : public QWidget
{
  Q_OBJECT

public:
  MotionPlanningFrameJointsWidget(const MotionPlanningFrameJointsWidget&) = delete;
  MotionPlanningFrameJointsWidget(QWidget* parent = nullptr);
  ~MotionPlanningFrameJointsWidget();

  void changePlanningGroup(const std::string& group_name,
                           const robot_interaction::InteractionHandlerPtr& start_state_handler,
                           const robot_interaction::InteractionHandlerPtr& goal_state_handler);

public Q_SLOTS:
  void queryStartStateChanged();
  void queryGoalStateChanged();

private:
  Ui::MotionPlanningFrameJointsUI* ui_;
  robot_interaction::InteractionHandlerPtr start_state_handler_;
  robot_interaction::InteractionHandlerPtr goal_state_handler_;
  std::unique_ptr<JMGItemModel> start_state_model_;
  std::unique_ptr<JMGItemModel> goal_state_model_;
};

/// Delegate to show the joint value as with a progress bar indicator between min and max.
class ProgressBarDelegate : public QStyledItemDelegate
{
  Q_OBJECT

public:
  enum CustomRole
  {
    JointTypeRole = Qt::UserRole,
    PercentageRole
  };

  ProgressBarDelegate(QWidget* parent = 0) : QStyledItemDelegate(parent)
  {
  }

  void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};
}
