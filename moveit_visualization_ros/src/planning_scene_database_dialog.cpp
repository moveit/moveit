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

#include <moveit_visualization_ros/planning_scene_database_dialog.h>
#include <moveit_visualization_ros/qt_helper_functions.h>

#include <QVBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QMetaType>

namespace moveit_visualization_ros 
{

PlanningSceneDatabaseDialog::PlanningSceneDatabaseDialog(QWidget* parent) :
  QDialog(parent)
{ 
  qRegisterMetaType<moveit_msgs::PlanningScenePtr>("PlanningSceneMsgPtr");
  QVBoxLayout* layout = new QVBoxLayout(this);
  planning_scene_table_ = new QTableWidget(this);
  layout->addWidget(planning_scene_table_);
  
  QHBoxLayout* button_layout = new QHBoxLayout();
  QPushButton* load_button = new QPushButton(this);
  load_button->setText("Load...");
  QPushButton* remove_button = new QPushButton(this);
  remove_button->setText("Remove...");
  button_layout->addWidget(load_button);
  button_layout->addWidget(remove_button);

  layout->addLayout(button_layout);

  connect(load_button, SIGNAL(clicked()), this, SLOT(loadSignalled()));

  setLayout(layout);
}


void PlanningSceneDatabaseDialog::populateDatabaseDialog(boost::shared_ptr<moveit_warehouse::PlanningSceneStorage>& storage)
{
  storage_ = storage;
  std::vector<std::string> names;
  std::vector<ros::Time> times;
  storage_->getPlanningSceneNamesAndTimes(names, times);

  ROS_ASSERT(names.size() == times.size());

  planning_scene_table_->clear();
  planning_scene_table_->setRowCount(names.size());
  planning_scene_table_->setColumnCount(2);
  QStringList labels;
  labels.append("Name");
  labels.append("Timestamp");
  for(unsigned int i = 0; i < names.size(); i++) {
    QTableWidgetItem* name_item = new QTableWidgetItem(QString::fromStdString(names[i]));
    name_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(i, 0, name_item);

    QDateTime time;
    time.setTime_t((unsigned int)(times[i].toSec()));
    stringstream timestamp_stream;
    timestamp_stream << time.toString().toStdString();

    PlanningSceneDateTableItem* time_item = new PlanningSceneDateTableItem(QString::fromStdString(timestamp_stream.str()));
    time_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(i, 1, time_item);
  }

  planning_scene_table_->sortByColumn(1, Qt::DescendingOrder);
  planning_scene_table_->setHorizontalHeaderLabels(labels);
  planning_scene_table_->setMinimumWidth(500);
  planning_scene_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
}

void PlanningSceneDatabaseDialog::loadSignalled() {
  QList<QTableWidgetItem*> items = planning_scene_table_->selectedItems();
  
  if(items.size() > 0)
  {
    QTableWidgetItem* item = items[0];
    QTableWidgetItem* name_item = planning_scene_table_->item(item->row(),0);
    QTableWidgetItem* time_item = planning_scene_table_->item(item->row(),1);
    QDateTime qdt = QDateTime::fromString(time_item->text());
    ros::Time t(qdt.toTime_t()*1.0);
    moveit_warehouse::PlanningSceneWithMetadata pswm;
    bool ok = storage_->getPlanningScene(pswm, name_item->text().toStdString(), t);
    if(ok) {
      moveit_msgs::PlanningScenePtr p(new moveit_msgs::PlanningScene(*pswm));
      //p.reset(new moveit_msgs::PlanningScene(*pswm));
      planningSceneLoaded(p);
      done(1);
    }
  }
}

void PlanningSceneDatabaseDialog::showEvent(QShowEvent* event) {
  QDialog::showEvent(event);
  planning_scene_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
}

}
