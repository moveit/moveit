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

#ifndef _KINEMATICS_GROUP_VISUALIZATION_H_
#define _KINEMATICS_GROUP_VISUALIZATION_H_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

namespace moveit_visualization_ros
{

class KinematicsGroupVisualization {
public:
  
  KinematicsGroupVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene_monitor,
                               boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                               const std::string& group_name, 
                               const std::string& suffix_name, 
                               const std::string& kinematics_solver_name,
                               const std_msgs::ColorRGBA& good_color,
                               const std_msgs::ColorRGBA& bad_color,
                               ros::Publisher& marker_publisher); 

  ~KinematicsGroupVisualization() {
  }

  const planning_models::KinematicState& getState() const {
    return state_;
  }

  void updateEndEffectorState(const geometry_msgs::Pose& pose);

  void hideAllMarkers();

  void showAllMarkers();

  void setMarkerAlpha(double a);

  void disable6DOFControls();
  void enable6DOFControls();

  void addButtonClickCallback(const boost::function<void(void)>& button_click_callback);

  void addMenuEntry(const std::string& name, 
                    const boost::function<void(void)>& callback);

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

protected:

  void sendCurrentMarkers();
  void removeLastMarkers();

  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);  
  void processInteractiveMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void makeInteractiveControlMarker(const std::string& name,
                                    const std_msgs::ColorRGBA& color,
                                    bool add6dof); 

protected:

  std::string group_name_;
  std::string suffix_name_;
  std::string interactive_marker_name_;
  std::string regular_marker_name_;
  std::vector<std::string> end_effector_link_names_;

  bool last_solution_good_;
  bool last_solution_changed_;

  std_msgs::ColorRGBA good_color_;
  std_msgs::ColorRGBA bad_color_;

  geometry_msgs::Pose last_pose_;
  visualization_msgs::MarkerArray last_marker_array_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  planning_models::KinematicState state_;
  ros::Publisher marker_publisher_;
  
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
  boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> ik_solver_;
  
  boost::function<void(void)> button_click_callback_;
  bool dof_marker_enabled_;

  std::map<std::string, boost::function<void(void)> > default_callback_map_;

  interactive_markers::MenuHandler default_menu_handler_;
  std::map<interactive_markers::MenuHandler::EntryHandle, std::string> menu_handle_to_string_map_;

};

}

#endif
