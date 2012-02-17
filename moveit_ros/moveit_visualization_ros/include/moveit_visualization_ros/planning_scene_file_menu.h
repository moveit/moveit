/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef _PLANNING_SCENE_FILE_MENU_H_
#define _PLANNING_SCENE_FILE_MENU_H_

#include <QString>
#include <QMenu>
#include <QAction>
#include <moveit_warehouse/warehouse_connector.h>
#include <planning_scene/planning_scene.h>
#include <moveit_visualization_ros/planning_scene_database_dialog.h>

namespace moveit_visualization_ros
{

class PlanningSceneFileMenu: public QMenu
{
  Q_OBJECT
  
  public:
  
  PlanningSceneFileMenu(QWidget* parent);
  
  PlanningSceneDatabaseDialog* getDatabaseDialog() {
    return database_dialog_;
  }
                                                                 
public Q_SLOTS:

  void connectToNewDatabaseSignalled();
  void saveCurrentPlanningSceneSignalled();
  void updatePlanningSceneSignalled(planning_scene::PlanningSceneConstPtr);
  void loadPlanningSceneSignalled();

Q_SIGNALS:

protected:
  
  moveit_warehouse::WarehouseConnector warehouse_connector_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> storage_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  PlanningSceneDatabaseDialog* database_dialog_;
  
  QAction* save_current_scene_;
  QAction* load_planning_scene_;

};

}

#endif
