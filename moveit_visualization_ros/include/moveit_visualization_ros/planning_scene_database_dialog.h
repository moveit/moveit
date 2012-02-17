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

#ifndef _PLANNING_SCENE_DATABASE_DIALOG_H_
#define _PLANNING_SCENE_DATABASE_DIALOG_H_

#include <moveit_warehouse/warehouse.h>
#include <planning_scene/planning_scene.h>

#include <QString>
#include <QMenu>
#include <QAction>
#include <QTableWidget>
#include <QDialog>
#include <QDateTime>

namespace moveit_visualization_ros
{

class PlanningSceneDatabaseDialog: public QDialog
{
  Q_OBJECT
  
  public:

  PlanningSceneDatabaseDialog(QWidget* parent);
                                             
  void populateDatabaseDialog(boost::shared_ptr<moveit_warehouse::PlanningSceneStorage>& storage);
                                                                                                
public Q_SLOTS:

  void loadSignalled();

Q_SIGNALS:

  void planningSceneLoaded(moveit_msgs::PlanningScenePtr);

protected:

  virtual void showEvent(QShowEvent* event);
  
  QTableWidget* planning_scene_table_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> storage_;
};

class PlanningSceneDateTableItem: public QObject, public QTableWidgetItem
{
  Q_OBJECT
  public:
  
  PlanningSceneDateTableItem(const QString& qs) : QTableWidgetItem(qs) {}
  
  bool operator< (const QTableWidgetItem& other) const {
    QDateTime other_time = QDateTime::fromString(other.text());
    QDateTime my_time = QDateTime::fromString(this->text());
    if(my_time < other_time) {
      return true;
    } else {
      return false;
    }
  }
};

}

#endif
