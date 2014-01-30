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

/* Author: Ioan Sucan, Dave Coleman, Adam Leeper, Sachin Chitta */

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_DISPLAY_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_DISPLAY_

#include <rviz/display.h>
#include <rviz/selection/selection_manager.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <std_msgs/String.h>

#ifndef Q_MOC_RUN
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>

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

  MotionPlanningDisplay();

  virtual ~MotionPlanningDisplay();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  robot_state::RobotStateConstPtr getQueryStartState() const
  {
    return query_start_state_->getState();
  }

  robot_state::RobotStateConstPtr getQueryGoalState() const
  {
    return query_goal_state_->getState();
  }

  const robot_interaction::RobotInteractionPtr& getRobotInteraction() const
  {
    return robot_interaction_;
  }

  const robot_interaction::RobotInteraction::InteractionHandlerPtr& getQueryStartStateHandler() const
  {
    return query_start_state_;
  }

  const robot_interaction::RobotInteraction::InteractionHandlerPtr& getQueryGoalStateHandler() const
  {
    return query_goal_state_;
  }

  void setQueryStartState(const robot_state::RobotState &start);
  void setQueryGoalState(const robot_state::RobotState &goal);

  void updateQueryStartState();
  void updateQueryGoalState();

  void useApproximateIK(bool flag);

  // Pick Place
  void clearPlaceLocationsDisplay();
  void visualizePlaceLocations(const std::vector<geometry_msgs::PoseStamped> &place_poses);
  std::vector<boost::shared_ptr<rviz::Shape> > place_locations_display_;

  std::string getCurrentPlanningGroup() const;

  void changePlanningGroup(const std::string& group);

  void addStatusText(const std::string &text);
  void addStatusText(const std::vector<std::string> &text);
  void setStatusTextColor(const QColor &color);
  void resetStatusTextColor();

Q_SIGNALS:
  void timeToShowNewTrail();

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
  void changedQueryMarkerScale();
  void changedQueryStartColor();
  void changedQueryGoalColor();
  void changedQueryStartAlpha();
  void changedQueryGoalAlpha();
  void changedQueryCollidingLinkColor();
  void changedQueryJointViolationColor();
  void changedPlanningGroup();
  void changedShowWeightLimit();
  void changedShowManipulabilityIndex();
  void changedShowManipulability();
  void changedShowJointTorques();
  void changedMetricsSetPayload();
  void changedMetricsTextHeight();
  void changedWorkspace();
  void resetInteractiveMarkers();

protected:

  enum LinkDisplayStatus
  {
    COLLISION_LINK,
    OUTSIDE_BOUNDS_LINK
  };

  virtual void onRobotModelLoaded();
  virtual void onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  virtual void updateInternal(float wall_dt, float ros_dt);

  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

  void renderWorkspaceBox();
  void updateLinkColors();

  void displayTable(const std::map<std::string, double> &values,
                    const Ogre::ColourValue &color,
                    const Ogre::Vector3 &pos, const Ogre::Quaternion &orient);
  void displayMetrics(bool start);

  void executeMainLoopJobs();
  void clearTrajectoryTrail();
  void publishInteractiveMarkers(bool pose_update);

  void recomputeQueryStartStateMetrics();
  void recomputeQueryGoalStateMetrics();
  void drawQueryStartState();
  void drawQueryGoalState();
  void scheduleDrawQueryStartState(robot_interaction::RobotInteraction::InteractionHandler *handler, bool error_state_changed);
  void scheduleDrawQueryGoalState(robot_interaction::RobotInteraction::InteractionHandler *handler, bool error_state_changed);

  bool isIKSolutionCollisionFree(robot_state::RobotState *state, const robot_state::JointModelGroup *group, const double *ik_solution) const;

  void computeMetrics(bool start, const std::string &group, double payload);
  void computeMetricsInternal(std::map<std::string, double> &metrics,
                              const robot_interaction::RobotInteraction::EndEffector &eef,
                              const robot_state::RobotState &state, double payload);
  void updateStateExceptModified(robot_state::RobotState &dest, const robot_state::RobotState &src);
  float getStateDisplayTime();
  void updateBackgroundJobProgressBar();
  void backgroundJobUpdate(moveit::tools::BackgroundProcessing::JobEvent event, const std::string &jobname);

  void setQueryStateHelper(bool use_start_state, const std::string &v);
  void populateMenuHandler(boost::shared_ptr<interactive_markers::MenuHandler>& mh);

  void selectPlanningGroupCallback(const std_msgs::StringConstPtr& msg);

  // overrides from Display
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();


  RobotStateVisualizationPtr query_robot_start_;                  ///< Handles drawing the robot at the start configuration
  RobotStateVisualizationPtr query_robot_goal_;                   ///< Handles drawing the robot at the goal configuration
  RobotStateVisualizationPtr display_path_robot_;                 ///< Handles actually drawing the robot along motion plans

  Ogre::SceneNode* text_display_scene_node_;        ///< displays texts
  bool text_display_for_start_;                     ///< indicates whether the text display is for the start state or not
  rviz::MovableText *text_to_display_;

  robot_trajectory::RobotTrajectoryPtr displaying_trajectory_message_;
  robot_trajectory::RobotTrajectoryPtr trajectory_message_to_display_;
  std::vector<rviz::Robot*> trajectory_trail_;
  ros::Subscriber planning_group_sub_;
  ros::Subscriber trajectory_topic_sub_;
  ros::NodeHandle private_handle_, node_handle_;
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
  boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_start_;
  boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_goal_;
  std::map<std::string, LinkDisplayStatus> status_links_start_;
  std::map<std::string, LinkDisplayStatus> status_links_goal_;

  /// Hold the names of the groups for which the query states have been updated (and should not be altered when new info is received from the planning scene)
  std::set<std::string> modified_groups_;

  /// The metrics are pairs of name-value for each of the active end effectors, for both start & goal states.
  /// computed_metrics_[std::make_pair(IS_START_STATE, GROUP_NAME)] = a map of key-value pairs
  std::map<std::pair<bool, std::string>, std::map<std::string, double> > computed_metrics_;
  /// Some groups use position only ik, calls to the metrics have to be modified appropriately
  std::map<std::string, bool> position_only_ik_;

  //Metric calculations
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  std::map<std::string, dynamics_solver::DynamicsSolverPtr> dynamics_solver_;
  boost::mutex update_metrics_lock_;

  // properties to show on side panel
  rviz::Property* path_category_;
  rviz::Property* plan_category_;
  rviz::Property* metrics_category_;

  rviz::EditableEnumProperty* planning_group_property_;
  rviz::BoolProperty* query_start_state_property_;
  rviz::BoolProperty* query_goal_state_property_;
  rviz::FloatProperty* query_marker_scale_property_;
  rviz::ColorProperty* query_start_color_property_;
  rviz::ColorProperty* query_goal_color_property_;
  rviz::FloatProperty* query_start_alpha_property_;
  rviz::FloatProperty* query_goal_alpha_property_;
  rviz::ColorProperty* query_colliding_link_color_property_;
  rviz::ColorProperty* query_outside_joint_limits_link_color_property_;

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
  rviz::FloatProperty* metrics_text_height_property_;
  rviz::BoolProperty* show_workspace_property_;

  rviz::Display *int_marker_display_;
};

} // namespace moveit_rviz_plugin

#endif
