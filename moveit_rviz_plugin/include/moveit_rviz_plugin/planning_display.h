/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Dave Coleman */

#ifndef MOVEIT_RVIZ_PLUGIN_PLANNING_DISPLAY_
#define MOVEIT_RVIZ_PLUGIN_PLANNING_DISPLAY_

#include <rviz/display.h>

#include "moveit_rviz_plugin/planning_frame.h"
#include "moveit_rviz_plugin/planning_scene_render.h"
#include "moveit_rviz_plugin/planning_markers.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <QDockWidget>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Robot;
class Shape;
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
}

namespace moveit_rviz_plugin
{

/**
 * \class PlanningDisplay
 * \brief
 */
class PlanningDisplay : public rviz::Display
{
  Q_OBJECT

public:

  struct TrajectoryMessageToDisplay
  {
    TrajectoryMessageToDisplay(const moveit_msgs::DisplayTrajectory::ConstPtr &message, const planning_scene::PlanningScenePtr &scene);
    TrajectoryMessageToDisplay(const planning_models::KinematicStatePtr &start_state, const std::vector<planning_models::KinematicStatePtr> &trajectory);
    
    planning_models::KinematicStatePtr start_state_;
    std::vector<planning_models::KinematicStatePtr> trajectory_;
  };

  /**
   * \brief Contructor
   */
  PlanningDisplay();

  /**
   * \brief Destructor
   */
  virtual ~PlanningDisplay();


  /**
   * \brief Overrides from Display
   */
  virtual void update(float wall_dt, float ros_dt);

  /**
   * \brief Called to reset plugin
   */
  virtual void reset();
  
  /**
   * \brief Robot Description parameter name
   */
  void setRobotDescription(const std::string &name);
  const std::string getRobotDescription(void);

  void setPlanningSceneTopic(const std::string &topic);
  const std::string getPlanningSceneTopic(void);

  /**
   * \brief Set of functions for highlighting parts of a robot
   */
  
  void setLinkColor( const std::string& link_name, float red, float green, float blue );
  void unsetLinkColor( const std::string& link_name );

  const planning_models::KinematicStatePtr& getQueryStartState(void) const
  {
    return query_start_state_;
  }

  const planning_models::KinematicStatePtr& getQueryGoalState(void) const
  {
    return query_goal_state_;
  }

  void setQueryStartState(const planning_models::KinematicStatePtr &start);
  void setQueryGoalState(const planning_models::KinematicStatePtr &goal);  
  
  void updateQueryStartState(void);
  void updateQueryGoalState(void);
  
  std::string getCurrentPlanningGroup(void) const;
  
  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor(void)
  {
    return scene_monitor_;
  }
  
  void displayRobotTrajectory(const planning_models::KinematicStatePtr &start_state,
                              const std::vector<planning_models::KinematicStatePtr> &trajectory);
                                                
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
  void changedSceneDisplayTime();
  void changedPlanningSceneTopic();
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedStateDisplayTime();
  void changedLoopDisplay();
  void changedTrajectoryTopic();
  void changedQueryStartState();
  void changedQueryGoalState();
  void changedPlanningGroup();
  
protected:
  // ******************************************************************************************
  // Protected
  // ******************************************************************************************
  
  /**
   * \brief Loads a URDF from our #description_param_
   */
  void loadRobotModel();

  /**
   * \brief ROS callback for an incoming kinematic path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

  /**
   * \brief Set the robot's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  void loadPlanningSceneMonitor();
  void renderPlanningScene();
  void setLinkColor(rviz::Robot* robot, const std::string& link_name, float red, float green, float blue );
  void unsetLinkColor(rviz::Robot* robot, const std::string& link_name );
  void setGroupColor(rviz::Robot* robot, const std::string& group_name, float red, float green, float blue );
  void unsetGroupColor(rviz::Robot* robot, const std::string& group_name );
  void unsetAllColors(rviz::Robot* robot);

  
  // overrides from Display  
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  rviz::Robot* query_robot_start_;                  ///< Handles drawing the robot at the start configuration
  rviz::Robot* query_robot_goal_;                   ///< Handles drawing the robot at the goal configuration
  rviz::Robot* display_path_robot_;                 ///< Handles actually drawing the robot along motion plans
  rviz::Robot* planning_scene_robot_;               ///< Handles actually drawing the robot from the planning scene

  Ogre::SceneNode* planning_scene_node_;            ///< displays planning scene

  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  boost::shared_ptr<TrajectoryMessageToDisplay> displaying_trajectory_message_;
  boost::shared_ptr<TrajectoryMessageToDisplay> trajectory_message_to_display_;

  ros::Subscriber trajectory_topic_sub_;

  // render the planning scene
  boost::scoped_ptr<PlanningSceneRender> planning_scene_render_;
  // interactive markers
  boost::scoped_ptr<PlanningMarkers> markers_;

  // the planning frame
  PlanningFrame *frame_;
  QDockWidget *frame_dock_;
  
  planning_models::KinematicStatePtr query_start_state_;
  planning_models::KinematicStatePtr query_goal_state_;
  bool update_display_start_state_;
  bool update_display_goal_state_;
  bool update_offset_transforms_;
  
  bool animating_path_;
  int current_state_;
  float current_state_time_;
  float current_scene_time_;
  bool planning_scene_needs_render_;
  
  // properties to show on side panel
  rviz::Property* scene_category_;
  rviz::Property* path_category_;
  rviz::Property* plan_category_;

  rviz::EditableEnumProperty* planning_group_property_;
  rviz::BoolProperty* query_start_state_property_;
  rviz::BoolProperty* query_goal_state_property_;

  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* scene_name_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::BoolProperty* scene_enabled_property_;
  rviz::BoolProperty* scene_robot_enabled_property_;
  rviz::FloatProperty* state_display_time_property_;
  rviz::FloatProperty* scene_display_time_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
  rviz::RosTopicProperty* planning_scene_topic_property_;
  rviz::FloatProperty* robot_path_alpha_property_;
  rviz::FloatProperty* robot_scene_alpha_property_;
  rviz::FloatProperty* scene_alpha_property_;
  rviz::BoolProperty* loop_display_property_;
  
  rviz::Display *int_marker_display_;
};

} // namespace moveit_rviz_plugin

#endif
