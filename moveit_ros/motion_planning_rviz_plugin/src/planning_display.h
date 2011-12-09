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

#ifndef MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_DISPLAY_H
#define MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_DISPLAY_H

#include "rviz/display.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/forwards.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_scene/planning_scene.h>

#include <ros/ros.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz
{
class Robot;
}

namespace motion_planning_rviz_plugin
{

/**
 * \class PlanningDisplay
 * \brief
 */
class PlanningDisplay : public rviz::Display
{
public:
  PlanningDisplay();
  virtual ~PlanningDisplay();

  virtual void onInitialize();
  virtual void reset();

  /**
   * \brief Set the robot description parameter
   * @param description_param The ROS parameter name which contains the robot xml description
   */
  void setRobotDescription( const std::string& description_param );
  const std::string& getRobotDescription() { return description_param_; }


  /**
   * \brief Set the topic to listen on for the JointPath message
   * @param topic The ROS topic
   */
  void setTrajectoryTopic( const std::string& topic );
  const std::string& getTrajectoryTopic() { return display_trajectory_topic_; }


  /**
   * \brief Set the amount of time each state should display for
   * @param time The length of time, in seconds
   */
  void setStateDisplayTime( float time );
  float getStateDisplayTime() { return state_display_time_; }


  /**
   * \brief Set whether the scene representation should be displayed
   * @param visible
   */
  void setSceneVisible( bool visible );
  bool getSceneVisible();

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


  void setAlpha( float alpha );
  float getAlpha() { return alpha_; }


  bool getLoopDisplay() { return loop_display_; }
  void setLoopDisplay(bool loop_display);


  virtual void update(float wall_dt, float ros_dt);

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged() {}
  virtual void createProperties();

protected:

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
   * \brief Uses libTF to set the robot's position, given the target frame and the planning frame
   */
  void calculateRobotPosition();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  struct ReceivedTrajectoryMessage;

  rviz::Robot* robot_;                              ///< Handles actually drawing the robot
  Ogre::SceneNode* scene_node_;            ///< displays planning scene

  ros::Subscriber sub_;

  // values filled in by properties on side panel
  std::string display_trajectory_topic_;
  std::string description_param_;             ///< ROS parameter that contains the robot xml description
  float alpha_;
  bool loop_display_;
  bool display_scene_;
  float state_display_time_;


  planning_scene::PlanningScenePtr planning_scene_;
  moveit_msgs::DisplayTrajectory::ConstPtr incoming_trajectory_message_;
  boost::scoped_ptr<ReceivedTrajectoryMessage> displaying_trajectory_message_;

  bool new_display_trajectory_;
  bool animating_path_;
  int current_state_;
  float current_state_time_;

  // properties to show on side panel
  rviz::BoolPropertyWPtr visual_enabled_property_;
  rviz::BoolPropertyWPtr collision_enabled_property_;
  rviz::BoolPropertyWPtr scene_enabled_property_;
  rviz::FloatPropertyWPtr state_display_time_property_;
  rviz::StringPropertyWPtr robot_description_property_;
  rviz::ROSTopicStringPropertyWPtr trajectory_topic_property_;
  rviz::FloatPropertyWPtr alpha_property_;
  rviz::BoolPropertyWPtr loop_display_property_;

};

} // namespace motion_planning_rviz_plugin

 #endif
