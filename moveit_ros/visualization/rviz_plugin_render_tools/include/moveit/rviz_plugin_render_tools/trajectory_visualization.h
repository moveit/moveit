/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

#ifndef MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TRAJECTORY_VISUALIZATION
#define MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TRAJECTORY_VISUALIZATION

#include <moveit/macros/class_forward.h>
#include <rviz/display.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

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
class ColorProperty;
class MovableText;
}

namespace moveit_rviz_plugin
{
MOVEIT_CLASS_FORWARD(TrajectoryVisualization);

class TrajectoryVisualization : public QObject
{
  Q_OBJECT

public:
  /**
   * \brief Playback a trajectory from a planned path
   * \param widget - either a rviz::Display or rviz::Property
   * \param display - the rviz::Display from the parent
   * \return true on success
   */
  TrajectoryVisualization(rviz::Property* widget, rviz::Display* display);

  virtual ~TrajectoryVisualization();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context, ros::NodeHandle update_nh);
  void onRobotModelLoaded(robot_model::RobotModelConstPtr robot_model);
  void onEnable();
  void onDisable();

public Q_SLOTS:
  void interruptCurrentDisplay();

private Q_SLOTS:

  /**
   * \brief Slot Event Functions
   */
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedLoopDisplay();
  void changedShowTrail();
  void changedTrajectoryTopic();
  void changedStateDisplayTime();
  void changedRobotColor();
  void enabledRobotColor();

protected:
  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
  float getStateDisplayTime();
  void clearTrajectoryTrail();

  // Handles actually drawing the robot along motion plans
  RobotStateVisualizationPtr display_path_robot_;

  // Handle colouring of robot
  void setRobotColor(rviz::Robot* robot, const QColor& color);
  void unsetRobotColor(rviz::Robot* robot);

  robot_trajectory::RobotTrajectoryPtr displaying_trajectory_message_;
  robot_trajectory::RobotTrajectoryPtr trajectory_message_to_display_;
  std::vector<rviz::Robot*> trajectory_trail_;
  ros::Subscriber trajectory_topic_sub_;
  bool animating_path_;
  int current_state_;
  float current_state_time_;
  boost::mutex update_trajectory_message_;

  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  // Pointers from parent display taht we save
  rviz::Display* display_;  // the parent display that this class populates
  rviz::Property* widget_;
  Ogre::SceneNode* scene_node_;
  rviz::DisplayContext* context_;
  ros::NodeHandle update_nh_;

  // Properties
  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::EditableEnumProperty* state_display_time_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
  rviz::FloatProperty* robot_path_alpha_property_;
  rviz::BoolProperty* loop_display_property_;
  rviz::BoolProperty* trail_display_property_;
  rviz::BoolProperty* interrupt_display_property_;
  rviz::ColorProperty* robot_color_property_;
  rviz::BoolProperty* enable_robot_color_property_;
};

}  // namespace moveit_rviz_plugin

#endif
