/*********************************************************************
 *
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <object_recognition_msgs/ObjectRecognitionGoal.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::detectObjectsButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::detectObjects, this), "detect objects");
  pick_object_name_.clear();  
}

void MotionPlanningFrame::pickObjectButtonClicked()
{
  QList<QListWidgetItem *> sel = ui_->detected_objects_list->selectedItems();
  QList<QListWidgetItem *> sel_table = ui_->support_surfaces_list->selectedItems();
  
  std::string group_name = planning_display_->getCurrentPlanningGroup();  
  if(sel.empty())
  {
    ROS_INFO("No objects to pick");    
    return;    
  }
  pick_object_name_[group_name] = sel[0]->text().toStdString();

  if(!sel_table.empty())
    support_surface_name_ = sel_table[0]->text().toStdString();
  else
    support_surface_name_.clear();  

  ROS_INFO("Trying to pick up object %s from support surface %s", 
           pick_object_name_[group_name].c_str(),
           support_surface_name_.c_str());  
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::pickObject, this), "pick");
}

void MotionPlanningFrame::placeObjectButtonClicked()
{
  QList<QListWidgetItem *> sel_table = ui_->support_surfaces_list->selectedItems();
  std::string group_name = planning_display_->getCurrentPlanningGroup();  

  if(!sel_table.empty())
    support_surface_name_ = sel_table[0]->text().toStdString();
  else
    support_surface_name_.clear();  

  ui_->pick_button->setEnabled(false);
  ui_->place_button->setEnabled(false);

  std::vector<const robot_state::AttachedBody*> attached_bodies;  
  const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
  if(!ps)
  {
    ROS_ERROR("No planning scene");
    return;
  }
  ps->getCurrentState().getJointStateGroup(group_name)->getAttachedBodies(attached_bodies);

  if(attached_bodies.empty())
  {
    ROS_ERROR("No bodies to place");
    return;
  }

  geometry_msgs::Quaternion upright_orientation;
  upright_orientation.w = 1.0;  

  // Else place the first one
  place_poses_.clear();  
  place_poses_ = semantic_world_->generatePlacePoses(support_surface_name_, 
                                                    attached_bodies[0]->getShapes()[0],
                                                    upright_orientation,
                                                    0.1);
  planning_display_->visualizePlaceLocations(place_poses_);  
  place_object_name_ = attached_bodies[0]->getName();  
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::placeObject, this), "place");
}

void MotionPlanningFrame::pickObject()
{
  std::string group_name = planning_display_->getCurrentPlanningGroup();  
  ui_->pick_button->setEnabled(false);
  if(pick_object_name_.find(group_name) == pick_object_name_.end())
  {
    ROS_ERROR("No pick object set for this group");
    return;    
  }  
  if(!support_surface_name_.empty())
  {
    move_group_->setSupportSurfaceName(support_surface_name_);
  }
  if(move_group_->pick(pick_object_name_[group_name]))
  {
    ui_->place_button->setEnabled(true);
  }  
}

void MotionPlanningFrame::placeObject()
{ 
  move_group_->place(place_object_name_, place_poses_);
  return;  
}

void MotionPlanningFrame::selectedDetectedObjectChanged()
{
}

void MotionPlanningFrame::detectedObjectChanged( QListWidgetItem *item)
{  
}

void MotionPlanningFrame::triggerObjectDetection()
{
  std_msgs::Bool msg;
  msg.data = true;
  object_recognition_trigger_publisher_.publish(msg);

  /*  object_recognition_msgs::ObjectRecognitionGoal goal;
  object_recognition_client_->sendGoal(goal);
  if (!object_recognition_client_->waitForResult())
  {
    ROS_INFO_STREAM("Object recognition client returned early");
  }
  if (object_recognition_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //    return true;
  }
  else
  {
    ROS_WARN_STREAM("Fail: " << object_recognition_client_->getState().toString() << ": " << object_recognition_client_->getState().getText());
    //    return false;
    }*/
}

void MotionPlanningFrame::detectObjects()
{
  triggerObjectDetection(); // Sleep for a small time to allow recognition to happen
  ros::Duration(3.0).sleep();
  std::vector<std::string> objects, object_ids;  

  double min_x = ui_->roi_center_x->value() - ui_->roi_size_x->value()/2.0;
  double min_y = ui_->roi_center_y->value() - ui_->roi_size_y->value()/2.0;
  double min_z = ui_->roi_center_z->value() - ui_->roi_size_z->value()/2.0;

  double max_x = ui_->roi_center_x->value() + ui_->roi_size_x->value()/2.0;
  double max_y = ui_->roi_center_y->value() + ui_->roi_size_y->value()/2.0;
  double max_z = ui_->roi_center_z->value() + ui_->roi_size_z->value()/2.0;

  object_ids = planning_scene_interface_->getKnownObjectNamesInROI(min_x, min_y, min_z, max_x, max_y, max_z, true, objects);
  updateDetectedObjectsList(object_ids, objects);

  semantic_world_->addTablesToCollisionWorld();  
  std::vector<std::string> support_surfaces = semantic_world_->getTableNamesInROI(min_x, min_y, min_z, max_x, max_y, max_z);  
  updateSupportSurfacesList(support_surfaces);  
}

void MotionPlanningFrame::updateDetectedObjectsList(const std::vector<std::string> &object_ids, 
                                                    const std::vector<std::string> &objects)
{
  ui_->detected_objects_list->setUpdatesEnabled(false);
  bool oldState = ui_->detected_objects_list->blockSignals(true);
  ui_->detected_objects_list->clear();
  {
    for(std::size_t i = 0; i < object_ids.size(); ++i)
    {
      QListWidgetItem * item = new QListWidgetItem(QString::fromStdString(object_ids[i]),
                                                   ui_->detected_objects_list, (int)i);      
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);      
      ui_->detected_objects_list->addItem(item);
    }    
  }
  ui_->detected_objects_list->blockSignals(oldState);
  ui_->detected_objects_list->setUpdatesEnabled(true);
  if(!object_ids.empty())
    ui_->pick_button->setEnabled(true);  
}

void MotionPlanningFrame::updateSupportSurfacesList(const std::vector<std::string> &support_ids)
{
  ui_->support_surfaces_list->setUpdatesEnabled(false);
  bool oldState = ui_->support_surfaces_list->blockSignals(true);
  ui_->support_surfaces_list->clear();
  {
    for(std::size_t i = 0; i < support_ids.size(); ++i)
    {
      QListWidgetItem * item = new QListWidgetItem(QString::fromStdString(support_ids[i]),
                                                   ui_->support_surfaces_list, (int)i);      
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);      
      ui_->support_surfaces_list->addItem(item);
    }    
  }
  ui_->support_surfaces_list->blockSignals(oldState);
  ui_->support_surfaces_list->setUpdatesEnabled(true);
}

}
