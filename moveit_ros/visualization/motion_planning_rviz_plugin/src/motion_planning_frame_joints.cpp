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
#include <QPainter>

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

Qt::ItemFlags JMGItemModel::flags(const QModelIndex& index) const
{
  if (!index.isValid())
    return 0;

  Qt::ItemFlags f = QAbstractTableModel::flags(index);
  if (index.column() == 1)
  {
    const moveit::core::JointModel* jm = getJointModel(index);
    if (!jm->isPassive() && !jm->getMimic())  // these are not editable
      f |= Qt::ItemIsEditable;
  }
  return f;
}

QVariant JMGItemModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
    return QVariant();

  int idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  switch (index.column())
  {
    case 0:  // joint name column
      switch (role)
      {
        case Qt::DisplayRole:
          return QString::fromStdString(robot_state_.getVariableNames()[idx]);
        case Qt::TextAlignmentRole:
          return Qt::AlignLeft;
      }
      break;
    case 1:  // joint value column
    {
      double value = robot_state_.getVariablePosition(idx);
      switch (role)
      {
        case Qt::DisplayRole:
          return value;
        case Qt::EditRole:
          if (const moveit::core::JointModel* jm = robot_state_.getRobotModel()->getJointOfVariable(idx))
            return jm->getType() == moveit::core::JointModel::REVOLUTE ? value * 180 / M_PI : value;
          break;
        case ProgressBarDelegate::JointTypeRole:
          if (const moveit::core::JointModel* jm = robot_state_.getRobotModel()->getJointOfVariable(idx))
            return jm->getType();
          break;
        case ProgressBarDelegate::PercentageRole:
          if (const moveit::core::VariableBounds* bounds = getVariableBounds(index))
            if (bounds->position_bounded_)
              return static_cast<int>(100. * (value - bounds->min_position_) /
                                      (bounds->max_position_ - bounds->min_position_));
          break;
        case Qt::TextAlignmentRole:
          return Qt::AlignRight;
      }
    }
  }
  return QVariant();
}

QVariant JMGItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    return section == 0 ? "Joint Name" : "Value";
  return QAbstractTableModel::headerData(section, orientation, role);
}

bool JMGItemModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (index.column() != 1 || role != Qt::EditRole)
    return false;

  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  const moveit::core::JointModel* jm = robot_state_.getRobotModel()->getJointOfVariable(var_idx);
  const moveit::core::VariableBounds* bounds = getVariableBounds(index);
  double v;
  if (bounds && value.userType() == QVariant::Int)  // int is interpreted as percentage between bounds
    v = bounds->min_position_ + value.toInt() / 100. * (bounds->max_position_ - bounds->min_position_);
  else if (value.userType() == QVariant::Double)
  {
    v = value.toDouble();
    // for revolute joints, we convert degrees to radians
    if (jm)
      if (jm->getType() == moveit::core::JointModel::REVOLUTE)
        v *= M_PI / 180;
  }
  else  // cannot handle this value
    return false;

  robot_state_.setVariablePosition(var_idx, v);
  jm->enforcePositionBounds(robot_state_.getVariablePositions() + jm->getFirstVariableIndex());
  dataChanged(index, index);
  return true;
}

const moveit::core::JointModel* JMGItemModel::getJointModel(const QModelIndex& index) const
{
  if (!index.isValid())
    return nullptr;
  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  return robot_state_.getRobotModel()->getJointOfVariable(var_idx);
}

const moveit::core::VariableBounds* JMGItemModel::getVariableBounds(const QModelIndex& index) const
{
  const moveit::core::JointModel* jm = getJointModel(index);
  if (!jm)
    return nullptr;
  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  return &jm->getVariableBounds()[var_idx - jm->getFirstVariableIndex()];
}

// copy positions from new_state and notify about these changes
void JMGItemModel::stateChanged(const moveit::core::RobotState& state)
{
  if (robot_state_.getRobotModel() != state.getRobotModel())
    return;
  robot_state_.setVariablePositions(state.getVariablePositions());

  dataChanged(index(0, 1), index(rowCount() - 1, 1));
}

MotionPlanningFrameJointsWidget::MotionPlanningFrameJointsWidget(MotionPlanningDisplay* display, QWidget* parent)
  : QWidget(parent), ui_(new Ui::MotionPlanningFrameJointsUI()), planning_display_(display)
{
  ui_->setupUi(this);
  ui_->joints_view_->setItemDelegateForColumn(1, new ProgressBarDelegate(this));
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

  // forward model updates to the PlanningDisplay
  connect(start_state_model_.get(), &JMGItemModel::dataChanged, this, [this]() {
    if (!ignore_state_changes_)
      planning_display_->setQueryStartState(start_state_model_->getRobotState());
  });
  connect(goal_state_model_.get(), &JMGItemModel::dataChanged, this, [this]() {
    if (!ignore_state_changes_)
      planning_display_->setQueryGoalState(goal_state_model_->getRobotState());
  });

  // show the goal state by default
  setActiveModel(goal_state_model_.get());
}

void MotionPlanningFrameJointsWidget::queryStartStateChanged()
{
  if (!start_state_model_ || !start_state_handler_)
    return;
  ignore_state_changes_ = true;
  start_state_model_->stateChanged(*start_state_handler_->getState());
  ignore_state_changes_ = false;
  setActiveModel(start_state_model_.get());
}

void MotionPlanningFrameJointsWidget::queryGoalStateChanged()
{
  if (!goal_state_model_ || !goal_state_handler_)
    return;
  ignore_state_changes_ = true;
  goal_state_model_->stateChanged(*goal_state_handler_->getState());
  ignore_state_changes_ = false;
  setActiveModel(goal_state_model_.get());
}

void MotionPlanningFrameJointsWidget::setActiveModel(JMGItemModel* model)
{
  ui_->joints_view_->setModel(model);
  ui_->joints_view_label_->setText(
      QString("Group joints of %1 state").arg(model == start_state_model_.get() ? "start" : "goal"));
}

void MotionPlanningFrameJointsWidget::triggerUpdate(JMGItemModel* model)
{
  if (model == start_state_model_.get())
    planning_display_->setQueryStartState(model->getRobotState());
  else
    planning_display_->setQueryGoalState(model->getRobotState());
}

void ProgressBarDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  // copied from QStyledItemDelegate::paint
  QStyle* style = option.widget ? option.widget->style() : QApplication::style();
  QStyleOptionViewItem style_option = option;
  initStyleOption(&style_option, index);

  if (index.column() == 1)
  {
    QVariant joint_type = index.data(JointTypeRole);
    double value = index.data().toDouble();
    bool is_revolute = joint_type.isValid() && joint_type.toInt() == moveit::core::JointModel::REVOLUTE;
    style_option.text = option.locale.toString(is_revolute ? value * 180 / M_PI : value, 'f', is_revolute ? 0 : 3);

    QVariant percentage = index.data(PercentageRole);
    if (percentage.isValid())
    {
      QStyleOptionProgressBar opt;
      opt.rect = option.rect;
      opt.minimum = 0;
      opt.maximum = 100;
      opt.progress = percentage.toInt();
      opt.text = style_option.text;
      opt.textAlignment = style_option.displayAlignment;
      opt.textVisible = true;
      style->drawControl(QStyle::CE_ProgressBar, &opt, painter);
      return;
    }
  }

  style->drawControl(QStyle::CE_ItemViewItem, &style_option, painter, option.widget);
}

QWidget* ProgressBarDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                                           const QModelIndex& index) const

{
  if (index.column() == 1)
  {
    QVariant percentage = index.data(PercentageRole);
    if (percentage.isValid())
    {
      auto* editor = new ProgressBarEditor(parent);
      connect(editor, &ProgressBarEditor::editingFinished, this, &ProgressBarDelegate::commitAndCloseEditor);
      connect(editor, &ProgressBarEditor::valueChanged, this, [=](int value) {
        JMGItemModel* model = dynamic_cast<JMGItemModel*>(const_cast<QAbstractItemModel*>(index.model()));
        model->setData(index, QVariant(value), Qt::EditRole);
      });
      return editor;
    }
  }
  return QStyledItemDelegate::createEditor(parent, option, index);
}

void ProgressBarDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
  if (ProgressBarEditor* pb_editor = qobject_cast<ProgressBarEditor*>(editor))
    pb_editor->setPercentage(index.data(PercentageRole).toInt());
  else
    QStyledItemDelegate::setEditorData(editor, index);
}

void ProgressBarDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
  if (ProgressBarEditor* pb_editor = qobject_cast<ProgressBarEditor*>(editor))
    model->setData(index, pb_editor->percentage());
  else
    QStyledItemDelegate::setModelData(editor, model, index);
}

void ProgressBarDelegate::commitAndCloseEditor()
{
  ProgressBarEditor* editor = qobject_cast<ProgressBarEditor*>(sender());
  commitData(editor);
  closeEditor(editor);
}

ProgressBarEditor::ProgressBarEditor(QWidget* parent) : QWidget(parent)
{
  setMouseTracking(true);
}

void ProgressBarEditor::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  QStyleOptionProgressBar opt;
  opt.rect = rect();
  opt.palette = this->palette();
  opt.minimum = 0;
  opt.maximum = 100;
  opt.progress = percentage_;
  opt.textVisible = false;
  style()->drawControl(QStyle::CE_ProgressBar, &opt, &painter);
}

void ProgressBarEditor::mouseMoveEvent(QMouseEvent* event)
{
  int p = 100 * event->x() / width();
  if (p < 0)
    p = 0;
  if (p > 100)
    p = 100;

  if (percentage_ != p)
  {
    percentage_ = p;
    valueChanged(p);
    update();
  }
  event->accept();
}

void ProgressBarEditor::mouseReleaseEvent(QMouseEvent* event)
{
  event->accept();
  editingFinished();
}
}  // namespace
