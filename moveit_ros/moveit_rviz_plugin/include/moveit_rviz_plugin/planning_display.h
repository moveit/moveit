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

#ifndef MOVEIT_RVIZ_PLUGIN_PLANNING_DISPLAY_H
#define MOVEIT_RVIZ_PLUGIN_PLANNING_DISPLAY_H

#include "rviz/helpers/color.h"
#include "rviz/display.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <OGRE/OgreMaterial.h>
#include <ros/ros.h>
#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Robot;
class RobotLink; //?
class Shape;
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
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

  /**
   * \brief Contructor
   */
  PlanningDisplay();

  /**
   * \brief Destructor
   */
  virtual ~PlanningDisplay();

  /**
   * \brief Called when plugin is ready to be loaded. Overrides from Display
   */
  virtual void onInitialize();

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

  /**
   * \brief Set the topic to listen on for the JointPath message
   * @param topic The ROS topic
   */
  void setTrajectoryTopic( const std::string& topic );
  const std::string getTrajectoryTopic();

  /**
   * \brief Set the amount of time each state should display for
   * @param time The length of time, in seconds
   */
  void setStateDisplayTime( float time );
  float getStateDisplayTime();

  /**
   * \brief
   */
  void setSceneDisplayTime( float time );
  float getSceneDisplayTime();

  /**
   * \brief Set whether the scene representation should be displayed
   * @param visible
   */
  void setSceneVisible( bool visible );
  bool getSceneVisible();

  /**
   * \brief Set whether the scene representation should be displayed
   * @param visible
   */
  void setSceneRobotVisible( bool visible );
  bool getSceneRobotVisible();

  /**
   * \brief Set whether the visual mesh representation should be displayed
   * @param visible
   */
  void setVisualVisible( bool visible );
  bool getVisualVisible();

  /**
   * \brief Set whether the collision representation should be displayed
   * @param visible
   */
  void setCollisionVisible( bool visible );
  bool getCollisionVisible();


  /**
   * \brief
   */
  void setPlanningSceneTopic(const std::string &topic);
  const std::string getPlanningSceneTopic(void);

  /**
   * \brief
   */
  void setRobotPathAlpha( float alpha );
  float getRobotPathAlpha();

  /**
   * \brief
   */
  void setLinkColor( const std::string& link_name, float red, float green, float blue );
  void unsetLinkColor( const std::string& link_name );

  /**
   * \brief
   */
  void setSceneRobotAlpha( float alpha );
  float getSceneRobotAlpha();

  /**
   * \brief
   */
  void setSceneAlpha( float alpha );
  float getSceneAlpha();

  /**
   * \brief
   */
  void setLoopDisplay(bool loop_display);
  bool getLoopDisplay();

  /**
   * \brief
   */
  void setSceneName(const std::string &name);
  const std::string getSceneName(void);

  /**
   * \brief
   */
  void setRootLinkName(const std::string &name);
  const std::string getRootLinkName(void);


  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();


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
  void changedVisualEnabled();
  void changedCollisionEnabled();
  void changedRobotPathAlpha();
  void changedStateDisplayTime();
  void changedLoopDisplay();
  void changedTrajectoryTopic();

protected:
  // ******************************************************************************************
  // Protected
  // ******************************************************************************************


  /**
   * \brief Subscribes to any ROS topics we need to subscribe to
   */
  void subscribe();
  /**
   * \brief Unsubscribes from all ROS topics we're currently subscribed to
   */
  void unsubscribe();

  /**
   * \brief Advertises any ROS topics
   */
  void advertise();

  /**
   * \brief Unadvertises all ROS topics that we have advertised
   */
  void unadvertise();

  /**
   * \brief Loads a URDF from our #description_param_
   */
  void load();

  /**
   * \brief ROS callback for an incoming kinematic path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

  /**
   * \brief Set the robot's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void renderPlanningScene();
  void renderShape(Ogre::SceneNode *node, const shapes::Shape *s, const Eigen::Affine3d &p, const rviz::Color &color, float alpha);
  void clearRenderedGeometry();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  struct ReceivedTrajectoryMessage;

  rviz::Robot* robot_;                              ///< Handles actually drawing the robot along motion plans
  rviz::Robot* scene_robot_;                        ///< Handles actually drawing the robot from the planning scene

  Ogre::SceneNode* scene_node_;            ///< displays planning scene

  ros::Subscriber sub_;

  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  moveit_msgs::DisplayTrajectory::ConstPtr incoming_trajectory_message_;
  boost::scoped_ptr<ReceivedTrajectoryMessage> displaying_trajectory_message_;
  std::vector<boost::shared_ptr<rviz::Shape> > scene_shapes_;
  std::vector<Ogre::ManualObject*> manual_objects_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  bool new_display_trajectory_;
  bool animating_path_;
  int current_state_;
  float current_state_time_;
  float current_scene_time_;
  ros::Time last_scene_render_;

  // properties to show on side panel
  rviz::Property* scene_category_;
  rviz::Property* path_category_;
  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* scene_name_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::BoolProperty* visual_enabled_property_;
  rviz::BoolProperty* collision_enabled_property_;
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

};

} // namespace moveit_rviz_plugin

#endif
