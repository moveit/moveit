/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Mario Prats, Ioan Sucan */

#include <moveit/warehouse/state_storage.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::populateRobotStatesList(void)
{
  ui_->list_states->clear();
  for (RobotStateMap::iterator it = robot_states_.begin(); it != robot_states_.end(); ++it)
  {
    QListWidgetItem* item = new QListWidgetItem(QString(it->first.c_str()));
    ui_->list_states->addItem(item);
  }
}

void MotionPlanningFrame::loadStateButtonClicked()
{
  if (robot_state_storage_)
  {
    bool ok;

    QString text =
        QInputDialog::getText(this, tr("Robot states to load"), tr("Pattern:"), QLineEdit::Normal, ".*", &ok);
    if (ok && !text.isEmpty())
    {
      loadStoredStates(text.toStdString());
    }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::loadStoredStates(const std::string& pattern)
{
  std::vector<std::string> names;
  try
  {
    robot_state_storage_->getKnownRobotStates(pattern, names);
  }
  catch (std::exception& ex)
  {
    QMessageBox::warning(this, "Cannot query the database",
                         QString("Wrongly formatted regular expression for robot states: ").append(ex.what()));
    return;
  }

  // Clear the current list
  clearStatesButtonClicked();

  for (std::size_t i = 0; i < names.size(); ++i)
  {
    moveit_warehouse::RobotStateWithMetadata rs;
    bool got_state = false;
    try
    {
      got_state = robot_state_storage_->getRobotState(rs, names[i]);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    if (!got_state)
      continue;

    // Overwrite if exists.
    if (robot_states_.find(names[i]) != robot_states_.end())
    {
      robot_states_.erase(names[i]);
    }

    // Store the current start state
    robot_states_.insert(RobotStatePair(names[i], *rs));
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::saveRobotStateButtonClicked(const robot_state::RobotState& state)
{
  bool ok = false;

  std::stringstream ss;
  ss << planning_display_->getRobotModel()->getName().c_str() << "_state_" << std::setfill('0') << std::setw(4)
     << robot_states_.size();

  QString text = QInputDialog::getText(this, tr("Choose a name"), tr("State name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (robot_states_.find(name) != robot_states_.end())
        QMessageBox::warning(
            this, "Name already exists",
            QString("The name '").append(name.c_str()).append("' already exists. Not creating state."));
      else
      {
        // Store the current start state
        moveit_msgs::RobotState msg;
        robot_state::robotStateToRobotStateMsg(state, msg);
        robot_states_.insert(RobotStatePair(name, msg));

        // Save to the database if connected
        if (robot_state_storage_)
        {
          try
          {
            robot_state_storage_->addRobotState(msg, name, planning_display_->getRobotModel()->getName());
          }
          catch (std::exception& ex)
          {
            ROS_ERROR("Cannot save robot state on the database: %s", ex.what());
          }
        }
        else
        {
          QMessageBox::warning(this, "Warning",
                               "Not connected to a database. The state will be created but not stored");
        }
      }
    }
    else
      QMessageBox::warning(this, "Start state not saved", "Cannot use an empty name for a new start state.");
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::saveStartStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryStartState());
}

void MotionPlanningFrame::saveGoalStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryGoalState());
}

void MotionPlanningFrame::saveCurrentStateButtonClicked()
{
  planning_display_->waitForCurrentRobotState();
  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (ps)
    saveRobotStateButtonClicked(ps->getCurrentState());
}

void MotionPlanningFrame::setAsStartStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    robot_state::RobotState robot_state(*planning_display_->getQueryStartState());
    robot_state::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryStartState(robot_state);
  }
}

void MotionPlanningFrame::setAsGoalStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    robot_state::RobotState robot_state(*planning_display_->getQueryGoalState());
    robot_state::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryGoalState(robot_state);
  }
}

void MotionPlanningFrame::removeStateButtonClicked()
{
  if (robot_state_storage_)
  {
    // Warn the user
    QMessageBox msgBox;
    msgBox.setText("All the selected states will be removed from the database");
    msgBox.setInformativeText("Do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret)
    {
      case QMessageBox::Yes:
      {
        QList<QListWidgetItem*> found_items = ui_->list_states->selectedItems();
        for (int i = 0; i < found_items.size(); ++i)
        {
          const std::string& name = found_items[i]->text().toStdString();
          try
          {
            robot_state_storage_->removeRobotState(name);
            robot_states_.erase(name);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR("%s", ex.what());
          }
        }
        break;
      }
    }
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::clearStatesButtonClicked()
{
  robot_states_.clear();
  populateRobotStatesList();
}

void MotionPlanningFrame::planAllStatesButtonClicked()
{
#if 0
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::planAllStatesCartesianExec, this),
                                      "plan all cartesian states");
#else
  planning_display_->spawnBackgroundJob(boost::bind(&MotionPlanningFrame::planAllStatesJointSpaceExec, this));
#endif
}

void MotionPlanningFrame::planAndExecuteAllStatesButtonClicked()
{
#if 0
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::planAndExecuteAllStatesCartesianExec, this),
                                      "plan and execute all cartesian states");
#else
  planning_display_->spawnBackgroundJob(boost::bind(&MotionPlanningFrame::planAndExecuteAllStatesJointSpaceExec, this));
#endif
}

void MotionPlanningFrame::planAllStatesCartesianTrajectory()
{
  std::vector<geometry_msgs::Pose> waypoints;
  for (RobotStateMap::iterator it = robot_states_.begin(); it != robot_states_.end(); ++it)
  {
    robot_state::RobotState robot_state(*planning_display_->getQueryGoalState());
    robot_state::robotStateMsgToRobotState(robot_states_[it->first], robot_state);
    planning_display_->setQueryGoalState(robot_state);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(robot_state.getGlobalLinkTransform(move_group_->getEndEffectorLink()), pose);
    ROS_INFO_STREAM("Using waypoint " << it->first << " : " << pose);
    waypoints.push_back(pose);
  }

  // setup default params
  double cart_step_size = 0.01;
  double cart_jump_thresh = 0.0;
  bool avoid_collisions = false;

  // compute trajectory
  moveit_msgs::RobotTrajectory trajectory;
  ROS_INFO_STREAM("computeCartesianPath");

  double fraction =
      move_group_->computeCartesianPath(waypoints, cart_step_size, cart_jump_thresh, trajectory, avoid_collisions);
  if (fraction > 0)
  {
    ROS_INFO("Planning suceeded with %f achieved", fraction * 100);

    // Success
    ui_->result_label->setText(QString::number(fraction * 100, 'f', 0).append(" % achieved"));

    moveit_msgs::RobotTrajectory scaled_trajectory = moveit_msgs::RobotTrajectory(trajectory);

    // Scaling (https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4)

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), scaled_trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool success =
        iptp.computeTimeStamps(rt, ui_->velocity_scaling_factor->value(), ui_->acceleration_scaling_factor->value());
    ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(scaled_trajectory);

    // Fill in move_group_
    current_plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
    current_plan_->trajectory_ = scaled_trajectory;
  }
  else
  {
    ROS_ERROR("Failed to compute CartesianPath");
  }
}

void MotionPlanningFrame::planAllStatesCartesianExec()
{
  if (!move_group_)
    return;

  ui_->plan_all_states_button->setEnabled(false);

  planAllStatesCartesianTrajectory();

  ui_->plan_all_states_button->setEnabled(true);
}

void MotionPlanningFrame::planAndExecuteAllStatesCartesianExec()
{
  if (!move_group_)
    return;

  ui_->plan_and_execute_all_states_button->setEnabled(false);

  planAllStatesCartesianTrajectory();
  computeExecuteButtonClicked();

  ui_->plan_and_execute_all_states_button->setEnabled(true);
}

void MotionPlanningFrame::planAllStatesJointSpaceExec()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->plan_all_states_button->setEnabled(false);

  std::vector<geometry_msgs::Pose> waypoints_;
  for (RobotStateMap::iterator it = robot_states_.begin(); it != robot_states_.end(); ++it)
  {
    robot_state::RobotState robot_state(*planning_display_->getQueryGoalState());
    robot_state::robotStateMsgToRobotState(robot_states_[it->first], robot_state);
    planning_display_->setQueryGoalState(robot_state);
    ROS_INFO_STREAM("Using waypoint " << it->first);
    ROS_INFO_STREAM(robot_state);

    computePlanButtonClicked();

    sleep(2);  // FIXME: wait fo display updates...

    // udpate next start state
    planning_display_->setQueryStartState(robot_state);
  }

  ui_->plan_all_states_button->setEnabled(true);
}

void MotionPlanningFrame::planAndExecuteAllStatesJointSpaceExec()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->plan_all_states_button->setEnabled(false);

  std::vector<geometry_msgs::Pose> waypoints_;
  for (RobotStateMap::iterator it = robot_states_.begin(); it != robot_states_.end(); ++it)
  {
    robot_state::RobotState robot_state(*planning_display_->getQueryGoalState());
    robot_state::robotStateMsgToRobotState(robot_states_[it->first], robot_state);
    planning_display_->setQueryGoalState(robot_state);
    ROS_INFO_STREAM("Using waypoint " << it->first);
    ROS_DEBUG_STREAM(robot_state);

    computePlanAndExecuteButtonClicked();

    sleep(2);  // FIXME: wait fo display updates...

    // udpate next start state
    planning_display_->setQueryStartState(robot_state);
  }

  ui_->plan_all_states_button->setEnabled(true);
}

}  // namespace
