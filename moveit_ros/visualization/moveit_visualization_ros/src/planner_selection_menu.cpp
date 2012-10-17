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

#include <moveit_visualization_ros/planner_selection_menu.h>
#include <moveit_visualization_ros/qt_helper_functions.h>
#include <ros/console.h>
#include <QAction>

// Get list of planners
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <pluginlib/class_loader.h>
#include <planning_interface/planning_interface.h>

namespace moveit_visualization_ros
{

PlannerSelectionMenu::PlannerSelectionMenu(QWidget* parent)
  : QMenu("Select Planner", parent)
{
}

void PlannerSelectionMenu::init(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const std::string& group_name)
{
  // Settings for menu
  action_group_ = new QActionGroup(this);
  action_group_->setExclusive(true);
  QObject::connect(action_group_, SIGNAL(triggered(QAction*)), this, SLOT(plannerTriggered(QAction*)));

  // Load planning plugins
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader;
  std::map<std::string, boost::shared_ptr<planning_interface::Planner> > planner_interfaces;

  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface",
                                                                                        "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
  for (std::size_t i = 0 ; i < classes.size() ; ++i)
  {
    ROS_INFO("Attempting to load and configure %s", classes[i].c_str());
    try
    {
      boost::shared_ptr<planning_interface::Planner> p = planner_plugin_loader->createInstance(classes[i]);
      p->init(planning_scene->getKinematicModel());
      planner_interfaces[classes[i]] = p;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner '" << classes[i] << "': " << ex.what());
    }
  }

  if (planner_interfaces.empty())
    ROS_ERROR("No planning plugins have been loaded.");
  else
  {
    std::stringstream ss;
    for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it =
           planner_interfaces.begin() ; it != planner_interfaces.end(); ++it)
    {
      ss << it->first << " ";
    }
    ROS_INFO("Available planner instances: %s", ss.str().c_str());
  }

  for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces.begin() ;
       it != planner_interfaces.end(); ++it)
  {
    moveit_msgs::PlannerInterfaceDescription pi_desc;
    pi_desc.name = it->first;
    it->second->getPlanningAlgorithms(pi_desc.planner_ids);

    for( size_t i = 0; i < pi_desc.planner_ids.size(); ++i)
    {
      //std::cout << pi_desc.planner_ids[i] << std::endl;

      // Remember entire list
      available_planners.push_back(pi_desc.planner_ids[i]);
    }

  }

  QString qgroup_name = QString::fromStdString(group_name);
  newGroupSelected(qgroup_name);
}

void PlannerSelectionMenu::newGroupSelected(const QString& group_name)
{
  ROS_INFO_STREAM( "RELOADING PLANNERS FOR GROUP " << group_name.toStdString() );

  std::string group = group_name.toStdString();

  // Remove all of this menu's actions
  this->clear();

  size_t found_position;
  size_t count = 0;

  // Add all planners related to this group
  for(std::vector<std::string>::const_iterator group_it = available_planners.begin();
      group_it != available_planners.end(); ++group_it )
  {
    // Check if the planner name is in the front of the string
    found_position = group_it->find("[");
    if(found_position != std::string::npos)
    {
      // This planner has brackets. Now check if first part matches our group
      if(group_it->substr(0, found_position) == group)
      {
        // this planner is for currently selected group, so add it:
        QAction* na = addAction(QString::fromStdString(group_it->substr(found_position+1,
                                                                        group_it->size()-found_position-2)));
        na->setCheckable(true);
        action_group_->addAction(na);
        ++count;
      }
    }
  }

  // Check if no planner were added. Maybe this is not OMPL and they don't have []-based planners. Add all
  if(!count)
  {
    // Just add all
    for(std::vector<std::string>::const_iterator group_it = available_planners.begin();
        group_it != available_planners.end(); ++group_it )
    {
      QAction* na = addAction(QString::fromStdString(*group_it));
      na->setCheckable(true);
      action_group_->addAction(na);
    }
  }


  if(count)
  {
    // Check the first menu item
    action_group_->actions()[0]->setChecked(true);

    // Trigger the event for first menu item
    plannerTriggered(action_group_->actions()[0]);
  }
}

void PlannerSelectionMenu::plannerTriggered(QAction* action) {
  ROS_INFO_STREAM("Should be using planner " << action->text().toStdString());
  plannerSelected(action->text());
}


}
