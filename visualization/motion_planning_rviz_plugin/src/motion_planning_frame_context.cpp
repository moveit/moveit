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

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::databaseConnectButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClicked, this), "connect to database");
}

void MotionPlanningFrame::publishSceneButtonClicked()
{
  const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    moveit_msgs::PlanningScene msg;
    ps->getPlanningSceneMsg(msg);
    planning_scene_publisher_.publish(msg);
  }
}

void MotionPlanningFrame::planningAlgorithmIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPlannerId(ui_->planning_algorithm_combo_box->itemText(index).toStdString());
    else
      move_group_->setPlannerId("");
  }
}

void MotionPlanningFrame::resetDbButtonClicked()
{
  if (QMessageBox::warning(this, "Data about to be deleted", "The following dialog will allow you to drop a MoveIt Warehouse database. Are you sure you want to continue?",
                           QMessageBox::Yes | QMessageBox::No) == QMessageBox::No)
    return;

  QStringList dbs;
  dbs.append("Planning Scenes");
  dbs.append("Constraints");
  dbs.append("Robot States");

  bool ok = false;
  QString response = QInputDialog::getItem(this, tr("Select Database"), tr("Choose the database to reset:"),
                                           dbs, 2, false, &ok);
  if (!ok)
    return;

  if (QMessageBox::critical(this, "Data about to be deleted", QString("All data in database '").append(response).append("'. Are you sure you want to continue?"),
                            QMessageBox::Yes | QMessageBox::No) == QMessageBox::No)
    return;

  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeResetDbButtonClicked, this, response.toStdString()), "reset database");
}

void MotionPlanningFrame::computeDatabaseConnectButtonClicked()
{
  if (planning_scene_storage_)
  {
    planning_scene_storage_.reset();
    robot_state_storage_.reset();
    constraints_storage_.reset();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 1));
  }
  else
  {
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 2));
    try
    {
      planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(ui_->database_host->text().toStdString(),
                                                                               ui_->database_port->value(), 5.0));
      robot_state_storage_.reset(new moveit_warehouse::RobotStateStorage(ui_->database_host->text().toStdString(),
                                                                         ui_->database_port->value(), 5.0));
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(ui_->database_host->text().toStdString(),
                                                                          ui_->database_port->value(), 5.0));
    }
    catch(std::runtime_error &ex)
    {
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 3));
      ROS_ERROR("%s", ex.what());
      return;
    }
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 4));
  }
}

void MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper(int mode)
{
  if (mode == 1)
  {
    ui_->planning_scene_tree->setUpdatesEnabled(false);
    ui_->planning_scene_tree->clear();
    ui_->planning_scene_tree->setUpdatesEnabled(true);

    ui_->database_connect_button->setUpdatesEnabled(false);
    ui_->database_connect_button->setText(QString::fromStdString("Connect"));
    ui_->database_connect_button->setStyleSheet("QPushButton { color : green }");
    ui_->database_connect_button->setUpdatesEnabled(true);
    ui_->reset_db_button->hide();

    ui_->load_scene_button->setEnabled(false);
    ui_->load_query_button->setEnabled(false);
    ui_->save_query_button->setEnabled(false);
    ui_->save_scene_button->setEnabled(false);
    ui_->delete_query_button->setEnabled(false);
    ui_->delete_scene_button->setEnabled(false);
  }
  else
    if (mode == 2)
    {
      ui_->database_connect_button->setUpdatesEnabled(false);
      ui_->database_connect_button->setText(QString::fromStdString("Connecting ..."));
      ui_->database_connect_button->setUpdatesEnabled(true);
    }
    else
      if (mode == 3)
      {
        ui_->database_connect_button->setUpdatesEnabled(false);
        ui_->database_connect_button->setText(QString::fromStdString("Connect"));
        ui_->database_connect_button->setStyleSheet("QPushButton { color : green }");
        ui_->database_connect_button->setUpdatesEnabled(true);
        ui_->reset_db_button->hide();
      }
      else
        if (mode == 4)
        {
          ui_->database_connect_button->setUpdatesEnabled(false);
          ui_->database_connect_button->setText(QString::fromStdString("Disconnect"));
          ui_->database_connect_button->setStyleSheet("QPushButton { color : darkBlue }");
          ui_->database_connect_button->setUpdatesEnabled(true);
          ui_->save_scene_button->setEnabled(true);
          ui_->reset_db_button->show();
          populatePlanningSceneTreeView();
        }
}

void MotionPlanningFrame::computeResetDbButtonClicked(const std::string &db)
{
  if (db == "Constraints" && constraints_storage_)
    constraints_storage_->reset();
  else
    if (db == "Robot States" && robot_state_storage_)
      robot_state_storage_->reset();
    else
      if (db == "Planning Scenes")
        planning_scene_storage_->reset();
}
}
