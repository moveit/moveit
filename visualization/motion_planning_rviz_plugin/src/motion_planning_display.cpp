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

#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/planning_scene_rviz_plugin/planning_link_updater.h>
#include <moveit/planning_scene_rviz_plugin/kinematic_state_visualization.h>
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_factory.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>

#include <moveit/kinematic_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

MotionPlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const moveit_msgs::DisplayTrajectory::ConstPtr &message,
                                                                              const planning_scene::PlanningScenePtr &scene)
{
  start_state_.reset(new kinematic_state::KinematicState(scene->getCurrentState()));
  kinematic_state::robotStateToKinematicState(*scene->getTransforms(), message->trajectory_start, *start_state_);
  trajectory_processing::convertToKinematicStates(trajectory_, message->trajectory_start, message->trajectory, *start_state_, scene->getTransforms());
  
  if (message->trajectory.joint_trajectory.points.size() > message->trajectory.multi_dof_joint_trajectory.points.size())
    for (std::size_t i = 0 ; i < message->trajectory.joint_trajectory.points.size() ; ++i)
      time_from_start_.push_back(message->trajectory.joint_trajectory.points[i].time_from_start.toSec());
  else
    for (std::size_t i = 0 ; i < message->trajectory.multi_dof_joint_trajectory.points.size() ; ++i)
      time_from_start_.push_back(message->trajectory.multi_dof_joint_trajectory.points[i].time_from_start.toSec());
  
  for (int i = (int)time_from_start_.size() - 1 ; i > 0 ; --i)
    time_from_start_[i] -= time_from_start_[i - 1];
}

MotionPlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const kinematic_state::KinematicStatePtr &start_state,
                                                                              const std::vector<kinematic_state::KinematicStatePtr> &trajectory) :
  start_state_(start_state), trajectory_(trajectory)
{
}

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
MotionPlanningDisplay::MotionPlanningDisplay() :
  PlanningSceneDisplay(),
  frame_(NULL),
  frame_dock_(NULL),
  animating_path_(false),
  int_marker_display_(NULL)
{  
  // Category Groups
  plan_category_  = new rviz::Property( "Planning Request",   QVariant(), "", this );
  metrics_category_ = new rviz::Property( "Planning Metrics", QVariant(), "", this );
  path_category_  = new rviz::Property( "Planned Path",   QVariant(), "", this );
  
  // Metrics category -----------------------------------------------------------------------------------------
  compute_weight_limit_property_ = new rviz::BoolProperty( "Show Weight Limit", false, "Shows the weight limit at a particular pose for an end-effector",
                                                           metrics_category_,
                                                           SLOT( changedShowWeightLimit() ), this );
  
  show_manipulability_index_property_ = new rviz::BoolProperty( "Show Manipulability Index", false, "Shows the manipulability index for an end-effector",
                                                                metrics_category_,
                                                                SLOT( changedShowManipulabilityIndex() ), this );
  
  show_manipulability_property_ = new rviz::BoolProperty( "Show Manipulability", false, "Shows the manipulability for an end-effector",
                                                          metrics_category_,
                                                          SLOT( changedShowManipulability() ), this );
  
  show_joint_torques_property_ = new rviz::BoolProperty( "Show Joint Torques", false, "Shows the joint torques for a given configuration and payload",
                                                         metrics_category_,
                                                         SLOT( changedShowJointTorques() ), this );
  
  metrics_set_payload_property_ =
    new rviz::FloatProperty( "Payload", 1.0f, "Specify the payload at the end effector (kg)",
                             metrics_category_,
                             SLOT( changedMetricsSetPayload() ), this );
  metrics_set_payload_property_->setMin( 0.0 );
  
  // Planning request category -----------------------------------------------------------------------------------------
  
  planning_group_property_ = new rviz::EditableEnumProperty("Planning Group", "", "The name of the group of links to plan for (from the ones defined in the SRDF)",
                                                            plan_category_,
                                                            SLOT( changedPlanningGroup() ), this );
  show_workspace_property_ = new rviz::BoolProperty( "Show Workspace", false, "Shows the axis-aligned bounding box for the workspace allowed for planning",
                                                     plan_category_,
                                                     SLOT( changedWorkspace() ), this );
  query_start_state_property_ = new rviz::BoolProperty( "Query Start State", true, "Shows the start state for the motion planning query",
                                                        plan_category_,
                                                        SLOT( changedQueryStartState() ), this );
  query_goal_state_property_ = new rviz::BoolProperty( "Query Goal State", true, "Shows the goal state for the motion planning query",
                                                       plan_category_,
                                                       SLOT( changedQueryGoalState() ), this );
  
  query_start_color_property_ = new rviz::ColorProperty("Start State Color", QColor(0, 255, 0), "The highlight color for the start state",
                                                        plan_category_,
                                                        SLOT( changedQueryStartColor() ), this);
  query_start_alpha_property_ =
    new rviz::FloatProperty( "Start State Alpha", 1.0f, "Specifies the alpha for the robot links",
                             plan_category_,
                             SLOT( changedQueryStartAlpha() ), this );
  query_start_alpha_property_->setMin( 0.0 );
  query_start_alpha_property_->setMax( 1.0 );
  
  
  query_goal_color_property_ = new rviz::ColorProperty( "Goal State Color", QColor(250, 128, 0), "The highlight color for the goal state",
                                                        plan_category_,
                                                        SLOT( changedQueryGoalColor() ), this );
  
  query_goal_alpha_property_ =
    new rviz::FloatProperty( "Goal State Alpha", 1.0f, "Specifies the alpha for the robot links",
                             plan_category_,
                             SLOT( changedQueryGoalAlpha() ), this );
  query_goal_alpha_property_->setMin( 0.0 );
  query_goal_alpha_property_->setMax( 1.0 );
  
  query_colliding_link_color_property_ = new rviz::ColorProperty( "Colliding Link Color", QColor(255, 0, 0), "The highlight color for colliding links",
                                                                  plan_category_,
                                                                  SLOT( changedQueryCollidingLinkColor() ), this );
  
  // Path category ----------------------------------------------------------------------------------------------------
  
  trajectory_topic_property_ =
    new rviz::RosTopicProperty( "Trajectory Topic", "",
                                ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),
                                "The topic on which the moveit_msgs::DisplayTrajectory messages are received",
                                path_category_,
                                SLOT( changedTrajectoryTopic() ), this );
  
  display_path_visual_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Visual", true, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                            path_category_,
                            SLOT( changedDisplayPathVisualEnabled() ), this );
  
  display_path_collision_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Collision", false, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
                            path_category_,
                            SLOT( changedDisplayPathCollisionEnabled() ), this );
  
  robot_path_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                             path_category_,
                             SLOT( changedRobotPathAlpha() ), this );
  robot_path_alpha_property_->setMin( 0.0 );
  robot_path_alpha_property_->setMax( 1.0 );
  
  state_display_time_property_ =  new rviz::EditableEnumProperty("State Display Time", "0.05 s", "The amount of wall-time to wait in between displaying states along a received trajectory path",
                                                                 path_category_,
                                                                 SLOT( changedStateDisplayTime() ), this );
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");
  
  loop_display_property_ =
    new rviz::BoolProperty( "Loop Animation", false, "Indicates whether the last received path is to be animated in a loop",
                            path_category_,
                            SLOT( changedLoopDisplay() ), this );
  
  trail_display_property_ =
    new rviz::BoolProperty( "Show Trail", false, "Show a path trail",
                            path_category_,
                            SLOT( changedShowTrail() ), this );
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
MotionPlanningDisplay::~MotionPlanningDisplay()
{
  clearTrajectoryTrail();
  delete text_to_display_;
  delete int_marker_display_;
  delete frame_dock_;
}

void MotionPlanningDisplay::onInitialize(void)
{  
  PlanningSceneDisplay::onInitialize();
  
  text_display_scene_node_ = planning_scene_node_->createChildSceneNode();
  
  display_path_robot_.reset(new KinematicStateVisualization(planning_scene_node_, context_, "Planned Path", path_category_));
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  display_path_robot_->setVisible(false); 
  
  query_robot_start_.reset(new KinematicStateVisualization(planning_scene_node_, context_, "Planning Request Start", NULL));
  query_robot_start_->setCollisionVisible(false);
  query_robot_start_->setVisualVisible(true);
  query_robot_start_->setVisible( query_start_state_property_->getBool() );
  
  query_robot_goal_.reset(new KinematicStateVisualization(planning_scene_node_, context_, "Planning Request Goal", NULL ));
  query_robot_goal_->setCollisionVisible(false);
  query_robot_goal_->setVisualVisible(true);
  query_robot_goal_->setVisible( query_goal_state_property_->getBool() );
  
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new MotionPlanningFrame(this, context_, window_context ? window_context->getParentWindow() : NULL);
  if (window_context)
    frame_dock_ = window_context->addPane("Motion Planning", frame_);  
  
  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction::RobotInteraction::INTERACTIVE_MARKER_TOPIC + "/update"));
  
  text_to_display_ = new rviz::MovableText("EMPTY");
  text_to_display_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_to_display_->setCharacterHeight(0.08);
  text_to_display_->showOnTop();
  text_to_display_->setVisible(false);
  text_display_for_start_ = false;
  text_display_scene_node_->attachObject(text_to_display_);
}

void MotionPlanningDisplay::reset(void)
{ 
  clearTrajectoryTrail();
  text_to_display_->setVisible(false);
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  
  display_path_robot_->clear();
  query_robot_start_->clear();
  query_robot_goal_->clear();
  
  PlanningSceneDisplay::reset();
  
  frame_->disable();
  frame_->enable();
  
  query_robot_start_->setVisible( query_start_state_property_->getBool() );
  query_robot_goal_->setVisible( query_goal_state_property_->getBool() );
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  display_path_robot_->setVisible(false); 
}

void MotionPlanningDisplay::addBackgroundJob(const boost::function<void(void)> &job)
{
  background_process_.addJob(job);
}

void MotionPlanningDisplay::addMainLoopJob(const boost::function<void(void)> &job)
{
  boost::mutex::scoped_lock slock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void MotionPlanningDisplay::changedShowWeightLimit(void)
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowManipulabilityIndex(void)
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowManipulability(void)
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowJointTorques(void)
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedMetricsSetPayload(void)
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::displayTable(const std::map<std::string, double> &values,
                                         const Ogre::ColourValue &color,
                                         const Ogre::Vector3 &pos,
                                         const Ogre::Quaternion &orient)
{
  // the line we want to render
  std::stringstream ss;
  for (std::map<std::string, double>::const_iterator it = values.begin() ; it != values.end() ; ++it)
    ss << boost::format("%-10s %-4.2f") % it->first % it->second << std::endl;
  
  if (ss.str().empty())
  {
    text_to_display_->setVisible(false);
    return;
  }
  
  text_to_display_->setCaption(ss.str());
  text_to_display_->setColor(color);
  text_display_scene_node_->setPosition(pos);
  text_display_scene_node_->setOrientation(orient);
  
  // make sure the node is visible
  text_to_display_->setVisible(true);
}

void MotionPlanningDisplay::clearTrajectoryTrail(void)
{
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    delete trajectory_trail_[i];
  trajectory_trail_.clear();
}

void MotionPlanningDisplay::changedLoopDisplay()
{
}

void MotionPlanningDisplay::changedShowTrail(void)
{
  clearTrajectoryTrail();
  if (!trail_display_property_->getBool() || !planning_scene_monitor_)
    return;
  boost::shared_ptr<TrajectoryMessageToDisplay> t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;
  if (!t)
    return;
  
  trajectory_trail_.resize(t->trajectory_.size());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {  
    rviz::Robot *r = new rviz::Robot(planning_scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), NULL); 
    r->load(*getKinematicModel()->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool() );
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool() );
    r->update(PlanningLinkUpdater(t->trajectory_[i]));
    r->setVisible(true);
    trajectory_trail_[i] = r;
  }
}

void MotionPlanningDisplay::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setAlpha(robot_path_alpha_property_->getFloat());
}

void MotionPlanningDisplay::renderWorkspaceBox(void)
{
  if (!frame_ || !show_workspace_property_->getBool())
  {
    if (workspace_box_)
      workspace_box_.reset();
    return;
  }
  
  if (!workspace_box_)
  {
    workspace_box_.reset(new rviz::Shape(rviz::Shape::Cube,
                                         context_->getSceneManager(), planning_scene_node_));
    workspace_box_->setColor(0.0f, 0.0f, 0.6f, 0.3f);
  }
  
  Ogre::Vector3 center(frame_->ui_->wcenter_x->value(),
                       frame_->ui_->wcenter_y->value(),
                       frame_->ui_->wcenter_z->value());
  Ogre::Vector3 extents(frame_->ui_->wsize_x->value(),
                        frame_->ui_->wsize_y->value(),
                        frame_->ui_->wsize_z->value());
  workspace_box_->setScale(extents);
  workspace_box_->setPosition(center);
}

void MotionPlanningDisplay::changedTrajectoryTopic(void)
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
    trajectory_topic_sub_ = update_nh_.subscribe(trajectory_topic_property_->getStdString(), 2, &MotionPlanningDisplay::incomingDisplayTrajectory, this);
}

void MotionPlanningDisplay::computeMetrics(double payload)
{
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
  {
    computeMetricsInternal(computed_metrics_[std::make_pair(true, eef[i].parent_group)], eef[i], *getQueryStartState(), payload);
    computeMetricsInternal(computed_metrics_[std::make_pair(false, eef[i].parent_group)], eef[i], *getQueryGoalState(), payload);
  }
}

void MotionPlanningDisplay::computeMetrics(bool start, const std::string &group, double payload)
{ 
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
    if (eef[i].parent_group == group)
      computeMetricsInternal(computed_metrics_[std::make_pair(start, group)], eef[i],
                             start ? *getQueryStartState() : *getQueryGoalState(), payload);
  if (start)
    updateQueryStartState();
  else
    updateQueryGoalState();
}

void MotionPlanningDisplay::computeMetricsInternal(std::map<std::string, double> &metrics, const robot_interaction::RobotInteraction::EndEffector &ee,
                                                   const kinematic_state::KinematicState &state, double payload)
{ 
  metrics.clear();
  dynamics_solver::DynamicsSolverPtr ds;
  std::map<std::string, dynamics_solver::DynamicsSolverPtr>::const_iterator it = dynamics_solver_.find(ee.parent_group);
  if (it != dynamics_solver_.end())
    ds = it->second;
  
  // Max payload
  if (ds)
  {
    double max_payload;
    unsigned int saturated_joint;
    std::vector<double> joint_values;
    state.getJointStateGroup(ee.parent_group)->getVariableValues(joint_values);
    if (ds->getMaxPayload(joint_values, max_payload, saturated_joint))
    {
      metrics["max_payload"] = max_payload;      
      metrics["saturated_joint"] = saturated_joint;      
    } 
    std::vector<double> joint_torques;
    joint_torques.resize(joint_values.size());    
    if (ds->getPayloadTorques(joint_values, payload, joint_torques))
    {
      for (std::size_t i = 0 ; i < joint_torques.size() ; ++i)
      {
        std::stringstream stream;
        stream << "torque[" << i << "]";        
        metrics[stream.str()] = joint_torques[i];
      }
    }
  }
  
  if (kinematics_metrics_)
  {
    double manipulability_index, manipulability;
    if (kinematics_metrics_->getManipulabilityIndex(state, ee.parent_group, manipulability_index))
      metrics["manipulability_index"] = manipulability_index;
    if (kinematics_metrics_->getManipulability(state, ee.parent_group, manipulability))
      metrics["manipulability"] = manipulability;
  }
}

static void copyItemIfExists(const std::map<std::string, double> &source, std::map<std::string, double> &dest, const std::string &key)
{
  std::map<std::string, double>::const_iterator it = source.find(key);
  if (it != source.end())
    dest[key] = it->second;
}

void MotionPlanningDisplay::displayMetrics(bool start)
{    
  if (!robot_interaction_ || !planning_scene_monitor_)
    return;
  
  static const Ogre::Quaternion orientation( 1.0, 0.0, 0.0, 0.0 );
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
  {
    Ogre::Vector3 position(0.0, 0.0, 0.0);
    std::map<std::string, double> text_table; 
    const std::map<std::string, double> &metrics_table = computed_metrics_[std::make_pair(start, eef[i].parent_group)];
    if (compute_weight_limit_property_->getBool())
    {    
      copyItemIfExists(metrics_table, text_table, "max_payload");
      copyItemIfExists(metrics_table, text_table, "saturated_joint");
    }
    if (show_manipulability_index_property_->getBool())
      copyItemIfExists(metrics_table, text_table, "manipulability_index");
    if (show_manipulability_property_->getBool())
      copyItemIfExists(metrics_table, text_table, "manipulability");
    if (show_joint_torques_property_->getBool())
    {
      std::size_t nj = getKinematicModel()->getJointModelGroup(eef[i].parent_group)->getJointModelNames().size();
      for(size_t j = 0 ; j < nj ; ++j)
      {
        std::stringstream stream;
        stream << "torque[" << j << "]";          
        copyItemIfExists(metrics_table, text_table, stream.str());          
      }
    }
    
    const kinematic_state::LinkState *ls = NULL;
    const kinematic_model::JointModelGroup *jmg = getKinematicModel()->getJointModelGroup(eef[i].parent_group);
    if (jmg)
      if (!jmg->getLinkModelNames().empty())
        ls = start ?
          getQueryStartState()->getLinkState(jmg->getLinkModelNames().back()) :
          getQueryGoalState()->getLinkState(jmg->getLinkModelNames().back());
    if (ls)
    {
      const Eigen::Vector3d &t = ls->getGlobalLinkTransform().translation();
      position[0] = t.x();
      position[1] = t.y();
      position[2] = t.z() + 0.2;
    }
    if (start)
      displayTable(text_table, query_start_color_property_->getOgreColor(), position, orientation);
    else
      displayTable(text_table, query_goal_color_property_->getOgreColor(), position, orientation);
    text_display_for_start_ = start;
  }
}

void MotionPlanningDisplay::changedQueryStartState(void)
{
  if (!planning_scene_monitor_)
    return;
  
  if (query_start_state_property_->getBool())
  {
    if (isEnabled())
    {
      // update link poses
      query_robot_start_->update(getQueryStartState());
      query_robot_start_->setVisible(true);  
      
      // update link colors
      getPlanningScene()->getCollidingLinks(collision_links_start_, *getQueryStartState());
      updateLinkColors();
      
      // update metrics text 
      displayMetrics(true);
    }
  }
  else
    query_robot_start_->setVisible(false);
  context_->queueRender();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this));
}

void MotionPlanningDisplay::publishInteractiveMarkers(void)
{
  if (robot_interaction_)
  {
    robot_interaction_->clearInteractiveMarkers();
    if (query_start_state_property_->getBool())
      robot_interaction_->addInteractiveMarkers(query_start_state_);
    if (query_goal_state_property_->getBool())
      robot_interaction_->addInteractiveMarkers(query_goal_state_);
    robot_interaction_->publishInteractiveMarkers();
  }
}

void MotionPlanningDisplay::changedQueryStartColor(void)
{
  changedQueryStartState();
}

void MotionPlanningDisplay::changedQueryStartAlpha(void)
{
  query_robot_start_->setAlpha(query_start_alpha_property_->getFloat());
}

void MotionPlanningDisplay::changedQueryGoalState(void)
{
  if (!planning_scene_monitor_)
    return;  
  if (query_goal_state_property_->getBool())
  {
    if (isEnabled())
    { 
      // update link poses
      query_robot_goal_->update(getQueryGoalState());
      query_robot_goal_->setVisible(true);  
      
      // update link colors
      getPlanningScene()->getCollidingLinks(collision_links_goal_, *getQueryGoalState());
      updateLinkColors();
      
      // update metrics text 
      displayMetrics(false);
    }
  }
  else
    query_robot_goal_->setVisible(false);
  context_->queueRender();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this));
}

void MotionPlanningDisplay::changedQueryGoalColor(void)
{
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryGoalAlpha(void)
{
  query_robot_goal_->setAlpha(query_goal_alpha_property_->getFloat());
}

void MotionPlanningDisplay::changedQueryCollidingLinkColor(void)
{
  changedQueryStartState();
  changedQueryGoalState();
}

void MotionPlanningDisplay::updateQueryStartState(robot_interaction::RobotInteraction::InteractionHandler *)
{ 
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(true, group, metrics_set_payload_property_->getFloat());
  updateQueryStartState();
}

void MotionPlanningDisplay::updateQueryGoalState(robot_interaction::RobotInteraction::InteractionHandler *)
{
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(false, group, metrics_set_payload_property_->getFloat());
  updateQueryGoalState();
}

void MotionPlanningDisplay::updateQueryStartState(void)
{
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryStartState, this));  
  context_->queueRender();
}

void MotionPlanningDisplay::updateQueryGoalState(void)
{ 
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryGoalState, this));  
  context_->queueRender();
}

void MotionPlanningDisplay::setQueryStartState(const kinematic_state::KinematicStatePtr &start)
{
  query_start_state_->setState(*start);
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(true, group, metrics_set_payload_property_->getFloat());
  updateQueryStartState();
}

void MotionPlanningDisplay::setQueryGoalState(const kinematic_state::KinematicStatePtr &goal)
{
  query_goal_state_->setState(*goal);
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(false, group, metrics_set_payload_property_->getFloat());
  updateQueryGoalState();
}

bool MotionPlanningDisplay::isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
{
  if (frame_->ui_->collision_aware_ik->isChecked() && planning_scene_monitor_)
  { 
    group->setVariableValues(ik_solution);
    return !planning_scene_monitor_->getPlanningScene()->isStateColliding(*group->getKinematicState(), group->getName());
  }
  else
    return true;
}

void MotionPlanningDisplay::updateLinkColors(void)
{  
  unsetAllColors(&query_robot_start_->getRobot());
  unsetAllColors(&query_robot_goal_->getRobot());
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
  {
    setGroupColor(&query_robot_start_->getRobot(), group, query_start_color_property_->getColor());
    setGroupColor(&query_robot_goal_->getRobot(), group, query_goal_color_property_->getColor());
    for (std::size_t i = 0 ; i < collision_links_start_.size() ; ++i)
      setLinkColor(&query_robot_start_->getRobot(), collision_links_start_[i], query_colliding_link_color_property_->getColor());
    for (std::size_t i = 0 ; i < collision_links_goal_.size() ; ++i)
      setLinkColor(&query_robot_goal_->getRobot(), collision_links_goal_[i], query_colliding_link_color_property_->getColor());
  }
}

void MotionPlanningDisplay::changedPlanningGroup(void)
{
  if (robot_interaction_)
    robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());
  computeMetrics(metrics_set_payload_property_->getFloat());
  updateLinkColors();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this));
  frame_->changePlanningGroup();
}

void MotionPlanningDisplay::changedWorkspace(void)
{
  renderWorkspaceBox();
}

std::string MotionPlanningDisplay::getCurrentPlanningGroup(void) const
{
  return planning_group_property_->getStdString();
}

void MotionPlanningDisplay::changedStateDisplayTime()
{
}

void MotionPlanningDisplay::displayRobotTrajectory(const kinematic_state::KinematicStatePtr &start_state,
                                                   const std::vector<kinematic_state::KinematicStatePtr> &trajectory)
{  
  trajectory_message_to_display_.reset(new TrajectoryMessageToDisplay(start_state, trajectory));
}

void MotionPlanningDisplay::changedDisplayPathVisualEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
    display_path_robot_->setVisible(displaying_trajectory_message_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setVisualVisible( display_path_visual_enabled_property_->getBool() );
  }
}

// ******************************************************************************************
// Collision Visible
// ******************************************************************************************

void MotionPlanningDisplay::changedDisplayPathCollisionEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
    display_path_robot_->setVisible(displaying_trajectory_message_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  }
}

void MotionPlanningDisplay::onRobotModelLoaded(void)
{ 
  PlanningSceneDisplay::onRobotModelLoaded();
  
  robot_interaction_.reset(new robot_interaction::RobotInteraction(getKinematicModel()));
  display_path_robot_->load(*getKinematicModel()->getURDF());
  query_robot_start_->load(*getKinematicModel()->getURDF());
  query_robot_goal_->load(*getKinematicModel()->getURDF());
  
  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(getPlanningScene()->getCurrentState()));
  query_start_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("start", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *getQueryStartState(), planning_scene_monitor_->getTFClient()));
  query_start_state_->setUpdateCallback(boost::bind(&MotionPlanningDisplay::updateQueryStartState, this, _1));
  query_goal_state_->setUpdateCallback(boost::bind(&MotionPlanningDisplay::updateQueryGoalState, this, _1));
  query_start_state_->setIKValidityCallback(boost::bind(&MotionPlanningDisplay::isIKSolutionCollisionFree, this, _1, _2));
  query_goal_state_->setIKValidityCallback(boost::bind(&MotionPlanningDisplay::isIKSolutionCollisionFree, this, _1, _2));

  const std::vector<std::string> &groups = getKinematicModel()->getJointModelGroupNames();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    planning_group_property_->addOptionStd(groups[i]);
  planning_group_property_->sortOptions();
  if (!groups.empty() && planning_group_property_->getStdString().empty())
    planning_group_property_->setStdString(groups[0]);
  
  robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());
  
  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(getKinematicModel()));  
  computeMetrics(metrics_set_payload_property_->getFloat());
  
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    if (getKinematicModel()->getJointModelGroup(groups[i])->isChain()) 
    {
      dynamics_solver_[groups[i]].reset(new dynamics_solver::DynamicsSolver());
      if (!dynamics_solver_[groups[i]]->initialize(getKinematicModel()->getURDF(),
                                                   getKinematicModel()->getSRDF(),
                                                   groups[i]))
      {
        dynamics_solver_[groups[i]].reset();
        dynamics_solver_.erase(groups[i]);
      }
    }  
  
  changedQueryStartState();
  changedQueryGoalState(); 
}

void MotionPlanningDisplay::onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  PlanningSceneDisplay::onSceneMonitorReceivedUpdate(update_type);
  
  std::string group = planning_group_property_->getStdString();
  if (query_start_state_property_->getBool() && !group.empty())
  {
    
    kinematic_state::JointStateGroup *jsg = getQueryStartState()->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getVariableValues(joint_state_values);
      query_start_state_->setState(getPlanningScene()->getCurrentState());
      getQueryStartState()->getJointStateGroup(group)->setVariableValues(joint_state_values);
      updateQueryStartState();
    }
  }
  
  if (query_goal_state_property_->getBool() && !group.empty())
  {
    kinematic_state::JointStateGroup *jsg = getQueryGoalState()->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getVariableValues(joint_state_values);
      query_goal_state_->setState(getPlanningScene()->getCurrentState());
      getQueryGoalState()->getJointStateGroup(group)->setVariableValues(joint_state_values);
      updateQueryGoalState();
    }
  }
  
  if (frame_)
    frame_->sceneUpdate(update_type);
}

// ******************************************************************************************
// Enable
// ******************************************************************************************
void MotionPlanningDisplay::onEnable()
{
  PlanningSceneDisplay::onEnable();
  
  display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  display_path_robot_->setVisible(displaying_trajectory_message_);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    trajectory_trail_[i]->setVisualVisible( display_path_visual_enabled_property_->getBool() );
    trajectory_trail_[i]->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
    trajectory_trail_[i]->setVisible(true);
  }
  
  text_to_display_->setVisible(false);
  
  changedPlanningGroup();
  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  frame_->enable();
  
  int_marker_display_->setEnabled(true);
  
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this));
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void MotionPlanningDisplay::onDisable()
{
  robot_interaction_->clear();
  int_marker_display_->setEnabled(false);
  
  display_path_robot_->setVisible(false);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setVisible(false);
  
  query_robot_start_->setVisible(false);
  query_robot_goal_->setVisible(false); 
  frame_->disable(); 
  text_to_display_->setVisible(false);
  
  PlanningSceneDisplay::onDisable();
}

void MotionPlanningDisplay::executeMainLoopJobs(void)
{  
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void(void)> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }   
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    catch(...)
    {
      ROS_ERROR("Exception caught executing main loop job");
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_lock_.unlock();
}

void MotionPlanningDisplay::queueRenderSceneGeometry(void)
{
  planning_scene_needs_render_ = true;
}

float MotionPlanningDisplay::getStateDisplayTime(void)
{
  std::string tm = state_display_time_property_->getStdString();
  if (tm == "REALTIME")
    return -1.0;
  else
  {
    boost::replace_all(tm, "s", "");
    boost::trim(tm);
    float t = 0.05f;
    try
    {
      t = boost::lexical_cast<float>(tm);
    }
    catch(const boost::bad_lexical_cast &ex)
    {
      state_display_time_property_->setStdString("0.05 s");
    }
    return t;
  }
}

// ******************************************************************************************
// Update
// ******************************************************************************************
void MotionPlanningDisplay::update(float wall_dt, float ros_dt)
{
  int_marker_display_->update(wall_dt, ros_dt);
  frame_->updateSceneMarkers(wall_dt, ros_dt);
  frame_->updateGoalPoseMarkers(wall_dt, ros_dt);
  
  Display::update(wall_dt, ros_dt);
  
  executeMainLoopJobs();
  
  if (!planning_scene_monitor_)
    return;
  
  if (!animating_path_ && !trajectory_message_to_display_ && loop_display_property_->getBool() && displaying_trajectory_message_)
  {
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = std::numeric_limits<float>::infinity();
  }
  
  if (!animating_path_ && trajectory_message_to_display_)
  {
    planning_scene_monitor_->updateFrameTransforms();
    displaying_trajectory_message_ = trajectory_message_to_display_;
    display_path_robot_->setVisible(isEnabled());
    trajectory_message_to_display_.reset();
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = std::numeric_limits<float>::infinity();
    display_path_robot_->update(displaying_trajectory_message_->start_state_);
  }
  
  if (animating_path_)
  { 
    float tm = getStateDisplayTime();
    if (tm < 0.0) // if we should use realtime
    {
      if ((std::size_t) (current_state_ + 1) < displaying_trajectory_message_->time_from_start_.size())
        tm = displaying_trajectory_message_->time_from_start_[current_state_ + 1];
      else
        tm = 0.0f;
    }
    if (current_state_time_ > tm)
    {
      ++current_state_;
      if ((std::size_t) current_state_ < displaying_trajectory_message_->trajectory_.size())
        display_path_robot_->update(displaying_trajectory_message_->trajectory_[current_state_]);
      else
        animating_path_ = false;
      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
  
  current_scene_time_ += wall_dt;
  if (current_scene_time_ > scene_display_time_property_->getFloat())
  {
    renderPlanningScene();  
    current_scene_time_ = 0.0f;
  }
  
  renderWorkspaceBox();
}

void MotionPlanningDisplay::load( const rviz::Config& config )
{
  PlanningSceneDisplay::load(config);
  if (frame_)
  { 
    QString host;
    if (config.mapGetString( "MoveIt_Warehouse_Host", &host))
      frame_->ui_->database_host->setText(host);
    int port;
    if (config.mapGetInt( "MoveIt_Warehouse_Port", &port))
      frame_->ui_->database_port->setValue(port);
    float d;
    if (config.mapGetFloat( "MoveIt_Planning_Time", &d))
      frame_->ui_->planning_time->setValue(d);
    if (config.mapGetFloat( "MoveIt_Goal_Tolerance", &d))
      frame_->ui_->goal_tolerance->setValue(d);
  }
}

void MotionPlanningDisplay::save( rviz::Config config ) const
{
  PlanningSceneDisplay::save(config);
  if (frame_)
  {
    config.mapSetValue( "MoveIt_Warehouse_Host", frame_->ui_->database_host->text());
    config.mapSetValue( "MoveIt_Warehouse_Port", frame_->ui_->database_port->value());
    config.mapSetValue( "MoveIt_Planning_Time", frame_->ui_->planning_time->value());
    config.mapSetValue( "MoveIt_Goal_Tolerance", frame_->ui_->goal_tolerance->value());
  }
}

void MotionPlanningDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  if (planning_scene_monitor_)
    if (msg->model_id != getKinematicModel()->getName())
      ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
               msg->model_id.c_str(), getKinematicModel()->getName().c_str());
  
  trajectory_message_to_display_.reset(new TrajectoryMessageToDisplay(msg, getPlanningScene()));
  
  if (trail_display_property_->getBool())
    changedShowTrail();
}

void MotionPlanningDisplay::fixedFrameChanged(void)
{
  PlanningSceneDisplay::fixedFrameChanged(); 
  if (int_marker_display_)
    int_marker_display_->setFixedFrame(fixed_frame_);
  changedPlanningGroup();
}


} // namespace moveit_rviz_plugin
