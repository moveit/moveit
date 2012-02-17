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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: E. Gil Jones

#include <moveit_visualization_ros/planning_scene_file_menu.h>
#include <moveit_visualization_ros/qt_helper_functions.h>

#include <QAction>
#include <QFileDialog>

namespace moveit_visualization_ros 
{

PlanningSceneFileMenu::PlanningSceneFileMenu(QWidget* parent)
  : QMenu("Scene Files", parent)
{
  database_dialog_ = new PlanningSceneDatabaseDialog(this);

  QAction* connect_to_new_database = addAction("Connect to Planning Scene Database");
  save_current_scene_ = addAction("Save Current Planning Scene");
  save_current_scene_->setDisabled(true);
  load_planning_scene_ = addAction("Load Scene From Database");
  load_planning_scene_->setDisabled(true);
  //QAction* new_planning_scene = addAction("Create New Planning Scene...");
  
  connect(connect_to_new_database, SIGNAL(triggered()), SLOT(connectToNewDatabaseSignalled()));
  connect(save_current_scene_, SIGNAL(triggered()), SLOT(saveCurrentPlanningSceneSignalled()));
  connect(load_planning_scene_, SIGNAL(triggered()), SLOT(loadPlanningSceneSignalled()));
}

void PlanningSceneFileMenu::updatePlanningSceneSignalled(planning_scene::PlanningSceneConstPtr planning_scene)
{
  planning_scene_ = planning_scene;
}

void PlanningSceneFileMenu::connectToNewDatabaseSignalled() {
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::DirectoryOnly);
  dialog.setOption(QFileDialog::ShowDirsOnly, true);
  if(dialog.exec()) {
    QStringList dirnames = dialog.selectedFiles();
    warehouse_connector_.connectToDatabase(dirnames[0].toStdString(), storage_);
    save_current_scene_->setEnabled(true);
    load_planning_scene_->setEnabled(true);
  }
}

void PlanningSceneFileMenu::loadPlanningSceneSignalled() {
  database_dialog_->populateDatabaseDialog(storage_);
  database_dialog_->exec();
}

void PlanningSceneFileMenu::saveCurrentPlanningSceneSignalled() {
  moveit_msgs::PlanningScene scene;
  planning_scene_->getPlanningSceneMsg(scene);
  scene.robot_state.joint_state.header.stamp = ros::Time(ros::WallTime::now().toSec());
  storage_->addPlanningScene(scene);
}

}
