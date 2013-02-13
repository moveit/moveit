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

#ifndef MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_PLUGIN_SCENE_DISPLAY_
#define MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_PLUGIN_SCENE_DISPLAY_

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Robot;
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class EnumProperty;
}

namespace moveit_rviz_plugin
{

class PlanningSceneDisplay : public rviz::Display
{
  Q_OBJECT

public:

  PlanningSceneDisplay();
  virtual ~PlanningSceneDisplay();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();
  
  void setLinkColor(const std::string &link_name, const QColor &color);
  void unsetLinkColor(const std::string& link_name);
  
  void queueRenderSceneGeometry();
  
  const robot_model::RobotModelConstPtr& getRobotModel();
  planning_scene_monitor::LockedPlanningSceneRO getPlanningSceneRO() const;
  planning_scene_monitor::LockedPlanningSceneRW getPlanningSceneRW();
  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor();
                                                                                                
private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void changedRobotDescription();
  void changedSceneName();
  void changedRootLinkName();
  void changedSceneEnabled();
  void changedSceneRobotEnabled();
  void changedRobotSceneAlpha();
  void changedSceneAlpha();
  void changedSceneColor();
  void changedAttachedBodyColor();
  void changedPlanningSceneTopic();
  void changedSceneDisplayTime();
  void changedOctreeRenderMode();
  void changedOctreeColorMode();
  
protected:

  void loadRobotModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  void renderPlanningScene();
  void setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor &color);
  void unsetLinkColor(rviz::Robot* robot, const std::string& link_name);
  void setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor &color);
  void unsetGroupColor(rviz::Robot* robot, const std::string& group_name);
  void unsetAllColors(rviz::Robot* robot);
  
  // overrides from Display  
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  virtual void onRobotModelLoaded();
  virtual void onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

  
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  
  Ogre::SceneNode* planning_scene_node_;            ///< displays planning scene with everything in it
  
  // render the planning scene
  RobotStateVisualizationPtr planning_scene_robot_;  
  PlanningSceneRenderPtr planning_scene_render_;
  
  bool planning_scene_needs_render_;
  float current_scene_time_;
  
  rviz::Property* scene_category_;
  rviz::Property* robot_category_;

  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* scene_name_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::BoolProperty* scene_enabled_property_;
  rviz::BoolProperty* scene_robot_enabled_property_;
  rviz::RosTopicProperty* planning_scene_topic_property_;
  rviz::FloatProperty* robot_alpha_property_;
  rviz::FloatProperty* scene_alpha_property_;
  rviz::ColorProperty* scene_color_property_;
  rviz::ColorProperty* attached_body_color_property_;
  rviz::FloatProperty* scene_display_time_property_;
  rviz::EnumProperty* octree_render_property_;
  rviz::EnumProperty* octree_coloring_property_;
};

} // namespace moveit_rviz_plugin

#endif
