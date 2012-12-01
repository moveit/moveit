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

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_DISPLAY_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_DISPLAY_

#include <rviz/display.h>
#include <rviz/selection/selection_manager.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>

#ifndef Q_MOC_RUN
#include <moveit/motion_planning_rviz_plugin/background_processing.h>
#include <moveit/robot_interaction/robot_interaction.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <ros/ros.h>
#endif

#include <moveit_msgs/DisplayTrajectory.h>
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
class ColorProperty;
class MovableText;
}

namespace moveit_rviz_plugin
{

class MotionPlanningDisplay : public PlanningSceneDisplay
{
  Q_OBJECT
  
  public:
  
  struct TrajectoryMessageToDisplay
  {
    TrajectoryMessageToDisplay(const moveit_msgs::DisplayTrajectory::ConstPtr &message, const planning_scene::PlanningScenePtr &scene);
    TrajectoryMessageToDisplay(const kinematic_state::KinematicStatePtr &start_state, const std::vector<kinematic_state::KinematicStatePtr> &trajectory);
    
    kinematic_state::KinematicStatePtr start_state_;
    std::vector<kinematic_state::KinematicStatePtr> trajectory_;
    std::vector<double> time_from_start_;
  };
  
  /**
   * \brief Contructor
   */
  MotionPlanningDisplay();
  
  /**
   * \brief Destructor
   */
  virtual ~MotionPlanningDisplay();
  
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();
  
  // pass the execution of this function call to a separate thread that runs in the background
  void addBackgroundJob(const boost::function<void(void)> &job);
  
  // queue the execution of this function for the next time the main update() loop gets called
  void addMainLoopJob(const boost::function<void(void)> &job);
  
  const kinematic_state::KinematicStatePtr& getQueryStartState(void) const
  {
    return query_start_state_->getState();
  }
  
  const kinematic_state::KinematicStatePtr& getQueryGoalState(void) const
  {
    return query_goal_state_->getState();
  }
  
  const robot_interaction::RobotInteractionPtr& getRobotInteraction(void) const
  {
    return robot_interaction_;
  }

  void setQueryStartState(const kinematic_state::KinematicStatePtr &start);
  void setQueryGoalState(const kinematic_state::KinematicStatePtr &goal);  
  
  void updateQueryStartState(void);
  void updateQueryGoalState(void);
  
  std::string getCurrentPlanningGroup(void) const;
  
  void queueRenderSceneGeometry(void);
  
  void displayRobotTrajectory(const kinematic_state::KinematicStatePtr &start_state,
                              const std::vector<kinematic_state::KinematicStatePtr> &trajectory);
                                                                                                
private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedStateDisplayTime();
  void changedLoopDisplay();
  void changedShowTrail();
  void changedTrajectoryTopic();
  void changedQueryStartState();
  void changedQueryGoalState();
  void changedQueryStartColor();
  void changedQueryGoalColor();
  void changedQueryStartAlpha();
  void changedQueryGoalAlpha();
  void changedQueryCollidingLinkColor();
  void changedPlanningGroup();
  void changedShowWeightLimit();
  void changedShowManipulabilityIndex();
  void changedShowManipulability();
  void changedShowJointTorques();
  void changedMetricsSetPayload();
  void changedWorkspace();
  
protected:

  virtual void onRobotModelLoaded();
  virtual void onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

  void renderWorkspaceBox(void);
  void updateLinkColors(void);
  
  void displayTable(const std::map<std::string, double> &values,
                    const Ogre::ColourValue &color,
                    const Ogre::Vector3 &pos, const Ogre::Quaternion &orient);
  void displayMetrics(bool start);

  void executeMainLoopJobs(void);
  void clearTrajectoryTrail();  
  void publishInteractiveMarkers(void);
  void updateQueryStartState(robot_interaction::RobotInteraction::InteractionHandler *handler);
  void updateQueryGoalState(robot_interaction::RobotInteraction::InteractionHandler *handler);
  bool isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const;
  
  void computeMetrics(double payload);
  void computeMetrics(bool start, const std::string &group, double payload);
  void computeMetricsInternal(std::map<std::string, double> &metrics,
                              const robot_interaction::RobotInteraction::EndEffector &eef,
                              const kinematic_state::KinematicState &state, double payload);
  float getStateDisplayTime(void);
  
  // overrides from Display  
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  BackgroundProcessing background_process_;
  std::deque<boost::function<void(void)> > main_loop_jobs_;
  boost::mutex main_loop_jobs_lock_;
  
  KinematicStateVisualizationPtr query_robot_start_;                  ///< Handles drawing the robot at the start configuration
  KinematicStateVisualizationPtr query_robot_goal_;                   ///< Handles drawing the robot at the goal configuration
  KinematicStateVisualizationPtr display_path_robot_;                 ///< Handles actually drawing the robot along motion plans

  Ogre::SceneNode* text_display_scene_node_;        ///< displays texts
  bool text_display_for_start_;                     ///< indicates whether the text display is for the start state or not
  rviz::MovableText *text_to_display_;
    
  boost::shared_ptr<TrajectoryMessageToDisplay> displaying_trajectory_message_;
  boost::shared_ptr<TrajectoryMessageToDisplay> trajectory_message_to_display_;
  std::vector<rviz::Robot*> trajectory_trail_;
  ros::Subscriber trajectory_topic_sub_;
  bool animating_path_;
  int current_state_;
  float current_state_time_;

  // render the workspace box
  boost::scoped_ptr<rviz::Shape> workspace_box_;
  
  // the planning frame
  MotionPlanningFrame *frame_;
  QDockWidget *frame_dock_;
  
  // robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_start_state_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_goal_state_;
  std::vector<std::string> collision_links_start_;
  std::vector<std::string> collision_links_goal_;

  /// The metrics are pairs of name-value for each of the active end effectors, for both start & goal states.
  /// computed_metrics_[std::make_pair(IS_START_STATE, GROUP_NAME)] = a map of key-value pairs
  std::map<std::pair<bool, std::string>, std::map<std::string, double> > computed_metrics_;
  
  std::set<std::string> invalid_start_state_;
  std::set<std::string> invalid_goal_state_;
  
  //Metric calculations
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;  
  std::map<std::string, dynamics_solver::DynamicsSolverPtr> dynamics_solver_;
     
  // properties to show on side panel
  rviz::Property* path_category_;
  rviz::Property* plan_category_;
  rviz::Property* metrics_category_;
  
  rviz::EditableEnumProperty* planning_group_property_;
  rviz::BoolProperty* query_start_state_property_;
  rviz::BoolProperty* query_goal_state_property_;
  rviz::ColorProperty* query_start_color_property_;
  rviz::ColorProperty* query_goal_color_property_;  
  rviz::FloatProperty* query_start_alpha_property_;
  rviz::FloatProperty* query_goal_alpha_property_;
  rviz::ColorProperty* query_colliding_link_color_property_;

  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::EditableEnumProperty* state_display_time_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
  rviz::FloatProperty* robot_path_alpha_property_;
  rviz::BoolProperty* loop_display_property_;
  rviz::BoolProperty* trail_display_property_;
  rviz::BoolProperty* compute_weight_limit_property_;
  rviz::BoolProperty* show_manipulability_index_property_;
  rviz::BoolProperty* show_manipulability_property_;
  rviz::BoolProperty* show_joint_torques_property_;
  rviz::FloatProperty* metrics_set_payload_property_;
  rviz::BoolProperty* show_workspace_property_;

  rviz::Display *int_marker_display_;
};

} // namespace moveit_rviz_plugin

#endif
