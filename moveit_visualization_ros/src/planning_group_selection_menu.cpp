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

#include <moveit_visualization_ros/planning_group_selection_menu.h>
#include <moveit_visualization_ros/qt_helper_functions.h>

#include <QAction>

namespace moveit_visualization_ros 
{

PlanningGroupSelectionMenu::PlanningGroupSelectionMenu(QWidget* parent)
  : QMenu("Select Planning Group", parent)
{
}

void PlanningGroupSelectionMenu::init(const boost::shared_ptr<const srdf::Model> &srdf_model) {
  const std::vector<srdf::Model::Group>& groups = srdf_model->getGroups();

  action_group_ = new QActionGroup(this);

  for(unsigned int i = 0; i < groups.size(); i++) {
    QAction* na = addAction(groups[i].name_.c_str());
    na->setCheckable(true);
    action_group_->addAction(na);
  }
  action_group_->setExclusive(true);
  QObject::connect(action_group_, SIGNAL(triggered(QAction*)), this, SLOT(groupTriggered(QAction*)));
  action_group_->actions()[0]->setChecked(true);
  groupTriggered(action_group_->actions()[0]);
}

void PlanningGroupSelectionMenu::groupTriggered(QAction* action) {
  ROS_INFO_STREAM("Should be triggering " << action->text().toStdString());
  groupSelected(action->text());
}

}
