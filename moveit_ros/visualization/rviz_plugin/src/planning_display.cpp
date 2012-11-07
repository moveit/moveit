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

#include <moveit/rviz_plugin/planning_display.h>
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
#include <tf/transform_broadcaster.h>

#include <moveit/kinematic_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include "planning_link_updater.h"
#include "ui_moveit_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

class RvizRobotInteractionHandler : public robot_interaction::RobotInteraction::InteractionHandler
{
public:
  RvizRobotInteractionHandler(PlanningDisplay *pdisplay, rviz::DisplayContext *context) :
    robot_interaction::RobotInteraction::InteractionHandler(),
    planning_display_(pdisplay),
    context_(context)
  {
  }
  
  virtual void handleEndEffector(const robot_interaction::RobotInteraction::EndEffector& eef, int id,
                                 const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  { 
    geometry_msgs::PoseStamped tpose;
    if (!handleGeneric(feedback, tpose))
      return;
    
    bool start = id == 0;
    if (start)
    {
      if (!robot_interaction::RobotInteraction::updateState(*planning_display_->getQueryStartState(), eef, tpose.pose))
      {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
          error_state_.insert(std::make_pair(eef.group, id));
      }
      else
        error_state_.erase(std::make_pair(eef.group, id));
      planning_display_->getQueryStartState()->updateLinkTransforms();
      planning_display_->updateQueryStartState();
    }
    else
    {
      if (!robot_interaction::RobotInteraction::updateState(*planning_display_->getQueryGoalState(), eef, tpose.pose))
      {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
          error_state_.insert(std::make_pair(eef.group, id));
      }
      else
        error_state_.erase(std::make_pair(eef.group, id));
      planning_display_->getQueryGoalState()->updateLinkTransforms();
      planning_display_->updateQueryGoalState();
    }
  }
  
  virtual void handleVirtualJoint(const robot_interaction::RobotInteraction::VirtualJoint& vj, int id,
                                  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    geometry_msgs::PoseStamped tpose;
    if (!handleGeneric(feedback, tpose))
      return;
    
    bool start = id == 0;
    if (start)
    {
      robot_interaction::RobotInteraction::updateState(*planning_display_->getQueryStartState(), vj, tpose.pose);
      planning_display_->getQueryStartState()->updateLinkTransforms();
      planning_display_->updateQueryStartState();
    }
    else
    {
      robot_interaction::RobotInteraction::updateState(*planning_display_->getQueryGoalState(), vj, tpose.pose);
      planning_display_->getQueryGoalState()->updateLinkTransforms();
      planning_display_->updateQueryGoalState();
    }
  }
  
  virtual bool inError(const robot_interaction::RobotInteraction::EndEffector& eef, int id)
  {
    return error_state_.find(std::make_pair(eef.group, id)) != error_state_.end();
  }
  
  virtual bool inError(const robot_interaction::RobotInteraction::VirtualJoint& vj, int id)
  {
    return false;
  }
  
private:

  bool handleGeneric(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, geometry_msgs::PoseStamped &tpose)
  {
    if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
      error_state_.clear();
    
    std::string planning_frame = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();
    tpose.header = feedback->header;
    tpose.pose = feedback->pose;
    if (feedback->header.frame_id != planning_frame)
    {
      try
      {
        context_->getTFClient()->transformPose(planning_frame, tpose, tpose);
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", tpose.header.frame_id.c_str(), planning_frame.c_str());
        return false;
      }
    }
    return true;
  }
  
  PlanningDisplay *planning_display_;
  rviz::DisplayContext *context_;
  std::set<std::pair<std::string, int> > error_state_;
};

PlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const moveit_msgs::DisplayTrajectory::ConstPtr &message,
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

PlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const kinematic_state::KinematicStatePtr &start_state,
                                                                        const std::vector<kinematic_state::KinematicStatePtr> &trajectory) :
  start_state_(start_state), trajectory_(trajectory)
{
}

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
PlanningDisplay::PlanningDisplay() :
  Display(),
  frame_(NULL),
  frame_dock_(NULL),
  show_planning_frame_(true),
  animating_path_(false),
  current_scene_time_(0.0f),
  planning_scene_needs_render_(true),
  int_marker_display_(NULL)
{  
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                              this,
                              SLOT( changedRobotDescription() ), this );

  // Category Groups
  plan_category_  = new rviz::Property( "Planning Request",   QVariant(), "", this );
  metrics_category_ = new rviz::Property( "Planning Metrics", QVariant(), "", this );
  path_category_  = new rviz::Property( "Planned Path",   QVariant(), "", this );
  scene_category_ = new rviz::Property( "Planning Scene", QVariant(), "", this );


  // Metrics category -----------------------------------------------------------------------------------------
  compute_weight_limit_property_ = new rviz::BoolProperty( "Show Weight Limit", false, "Shows the weight limit at a particular pose for an end-effector",
                                                           metrics_category_,
                                                           SLOT( changedShowWeightLimit() ), this );
  
  show_manipulability_index_property_ = new rviz::BoolProperty( "Show Manipulability Index", false, "Shows the manipulability index for an end-effector",
                                                                metrics_category_,
                                                                SLOT( changedShowManipulabilityIndex() ), this );
  
  show_manipulability_region_property_ = new rviz::BoolProperty( "Show Manipulability Region", false, "Shows the manipulability region for an end-effector",
                                                                 metrics_category_,
                                                                 SLOT( changedShowManipulabilityRegion() ), this );

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
  // Planning scene category -------------------------------------------------------------------------------------------

  scene_name_property_ =
    new rviz::StringProperty( "Scene Name", "(noname)", "Shows the name of the planning scene",
                              scene_category_,
                              SLOT( changedSceneName() ), this );
  scene_name_property_->setShouldBeSaved(false);
  
  root_link_name_property_ =
    new rviz::StringProperty( "Robot Root Link", "", "Shows the name of the root link for the robot model",
                              scene_category_,
                              SLOT( changedRootLinkName() ), this );
  root_link_name_property_->setReadOnly(true);
  
  scene_enabled_property_ =
    new rviz::BoolProperty( "Show Scene Geometry", true, "Indicates whether planning scenes should be displayed",
                            scene_category_,
                            SLOT( changedSceneEnabled() ), this );

  scene_robot_enabled_property_ =
    new rviz::BoolProperty( "Show Scene Robot", true, "Indicates whether the robot state specified by the planning scene should be displayed",
                            scene_category_,
                            SLOT( changedSceneRobotEnabled() ), this );

  robot_scene_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                             scene_category_,
                             SLOT( changedRobotSceneAlpha() ), this );
  robot_scene_alpha_property_->setMin( 0.0 );
  robot_scene_alpha_property_->setMax( 1.0 );

  scene_alpha_property_ =
    new rviz::FloatProperty( "Scene Alpha", 0.9f, "Specifies the alpha for the robot links",
                             scene_category_,
                             SLOT( changedSceneAlpha() ), this );
  scene_alpha_property_->setMin( 0.0 );
  scene_alpha_property_->setMax( 1.0 );

  scene_color_property_ = new rviz::ColorProperty( "Scene Color", QColor(50, 230, 50), "The color for the planning scene obstacles (if a color is not defined)",
                                                   scene_category_,
                                                   SLOT( changedSceneColor() ), this );
  attached_body_color_property_ = new rviz::ColorProperty( "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                           scene_category_,
                                                           SLOT( changedAttachedBodyColor() ), this );
  
  scene_display_time_property_ =
    new rviz::FloatProperty( "Scene Display Time", 0.2f, "The amount of wall-time to wait in between rendering updates to the planning scene (if any)",
                             scene_category_,
                             SLOT( changedSceneDisplayTime() ), this );
  scene_display_time_property_->setMin(0.0001);


  planning_scene_topic_property_ =
    new rviz::RosTopicProperty( "Planning Scene Topic", "planning_scene",
                                ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                "The topic on which the moveit_msgs::PlanningScene messages are received",
                                scene_category_,
                                SLOT( changedPlanningSceneTopic() ), this );

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
PlanningDisplay::~PlanningDisplay()
{
  clearTrajectoryTrail();
  delete text_to_display_;
  delete int_marker_display_;
  delete frame_dock_;
  delete display_path_robot_;
  delete planning_scene_robot_;
  delete query_robot_start_;
  delete query_robot_goal_;
}

void PlanningDisplay::onInitialize(void)
{  
  Display::onInitialize();
  
  // the scene node that contains everything
  planning_scene_node_ = scene_node_->createChildSceneNode();
  
  rendered_geometry_node_ = planning_scene_node_->createChildSceneNode();
  rendered_geometry_node_->setVisible(scene_enabled_property_->getBool()); 

  text_display_scene_node_ = planning_scene_node_->createChildSceneNode();
    
  display_path_robot_ = new rviz::Robot(planning_scene_node_, context_, "Planned Path", path_category_ );
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  display_path_robot_->setVisible(false); 

  planning_scene_robot_ = new rviz::Robot(planning_scene_node_, context_, "Planning Scene", scene_category_ );
  planning_scene_robot_->setCollisionVisible(false);
  planning_scene_robot_->setVisualVisible(true);
  planning_scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );

  query_robot_start_ = new rviz::Robot(planning_scene_node_, context_, "Planning Request Start", NULL );
  query_robot_start_->setCollisionVisible(false);
  query_robot_start_->setVisualVisible(true);
  query_robot_start_->setVisible( query_start_state_property_->getBool() );

  query_robot_goal_ = new rviz::Robot(planning_scene_node_, context_, "Planning Request Goal", NULL );
  query_robot_goal_->setCollisionVisible(false);
  query_robot_goal_->setVisualVisible(true);
  query_robot_goal_->setVisible( query_goal_state_property_->getBool() );
  
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new PlanningFrame(this, context_, window_context ? window_context->getParentWindow() : NULL);
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
  
  /*
  text_coll_object_ = context_->getSelectionManager()->createHandle();
  context_->getSelectionManager()->addPickTechnique( text_coll_object_, text_to_display_->getMaterial() );
  
  // we should add a selection manager at some point
  SelectionHandlerPtr handler( new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id)) );
  context_->getSelectionManager()->addObject( coll, handler );;
  */ 
}

void PlanningDisplay::reset(void)
{ 
  clearTrajectoryTrail();
  planning_scene_render_.reset();
  text_to_display_->setVisible(false);
  
  display_path_robot_->clear();
  planning_scene_robot_->clear();
  query_robot_start_->clear();
  query_robot_goal_->clear();

  loadRobotModel();
  frame_->disable();
  if (show_planning_frame_)
    frame_->enable();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  Display::reset();
}

void PlanningDisplay::addBackgroundJob(const boost::function<void(void)> &job)
{
  background_process_.addJob(job);
}

void PlanningDisplay::addMainLoopJob(const boost::function<void(void)> &job)
{
  boost::mutex::scoped_lock slock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

const dynamics_solver::DynamicsSolverPtr& PlanningDisplay::getDynamicsSolver(const std::string &group)
{
  std::map<std::string, dynamics_solver::DynamicsSolverPtr>::const_iterator it = dynamics_solver_.find(group);
  if (it == dynamics_solver_.end())
  { 
    static dynamics_solver::DynamicsSolverPtr empty;
    return empty;
  }
  else
    return it->second;
}

void PlanningDisplay::changedAttachedBodyColor(void)
{
  queueRenderSceneGeometry();
}

void PlanningDisplay::changedSceneColor(void)
{
  queueRenderSceneGeometry();
}

void PlanningDisplay::changedShowWeightLimit(void)
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

void PlanningDisplay::changedShowManipulabilityIndex(void)
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

void PlanningDisplay::changedShowManipulabilityRegion(void)
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

void PlanningDisplay::changedShowJointTorques(void)
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

void PlanningDisplay::changedMetricsSetPayload(void)
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

void PlanningDisplay::displayTable(const std::map<std::string, double> &values,
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

void PlanningDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

// ******************************************************************************************
// Scene Name
// ******************************************************************************************
void PlanningDisplay::changedSceneName(void)
{
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
    planning_scene_monitor_->getPlanningScene()->setName(scene_name_property_->getStdString());
}

// ******************************************************************************************
// Root Link Name
// ******************************************************************************************
void PlanningDisplay::changedRootLinkName()
{  
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
    root_link_name_property_->setStdString( planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLinkName() );
}

void PlanningDisplay::clearTrajectoryTrail(void)
{
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    delete trajectory_trail_[i];
  trajectory_trail_.clear();
}

void PlanningDisplay::changedLoopDisplay()
{
}

void PlanningDisplay::changedShowTrail(void)
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
    r->load(*planning_scene_monitor_->getKinematicModel()->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool() );
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool() );
    r->update(PlanningLinkUpdater(t->trajectory_[i]));
    r->setVisible(true);
    trajectory_trail_[i] = r;
  }
}

// ******************************************************************************************
// Robot Path Alpha
// ******************************************************************************************
void PlanningDisplay::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setAlpha(robot_path_alpha_property_->getFloat());
}

void PlanningDisplay::renderWorkspaceBox(void)
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

void PlanningDisplay::renderPlanningScene(void)
{
  if (planning_scene_render_ && planning_scene_needs_render_)
  {
    QColor color = scene_color_property_->getColor();
    rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
    color = attached_body_color_property_->getColor();
    rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());
    
    planning_scene_monitor_->lockScene();
    try
    {
      planning_scene_render_->renderPlanningScene(planning_scene_monitor_->getPlanningScene(),
                                                  env_color, attached_color,
                                                  scene_alpha_property_->getFloat(),
                                                  robot_scene_alpha_property_->getFloat());
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while rendering planning scene");
    }
    planning_scene_needs_render_ = false;
    planning_scene_monitor_->unlockScene();
    rendered_geometry_node_->setVisible(scene_enabled_property_->getBool());
  }
}

void PlanningDisplay::changedSceneAlpha()
{
  queueRenderSceneGeometry();
}

// ******************************************************************************************
// Scene Robot Alpha
// ******************************************************************************************
void PlanningDisplay::changedRobotSceneAlpha()
{
  planning_scene_robot_->setAlpha(robot_scene_alpha_property_->getFloat());
}

// ******************************************************************************************
// Trajectory Topic
// ******************************************************************************************

void PlanningDisplay::changedTrajectoryTopic(void)
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
    trajectory_topic_sub_ = update_nh_.subscribe(trajectory_topic_property_->getStdString(), 2, &PlanningDisplay::incomingDisplayTrajectory, this);
}

void PlanningDisplay::changedPlanningSceneTopic(void)
{
  if (planning_scene_monitor_)
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
}

void PlanningDisplay::computeMetrics(double payload)
{
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
  {
    computeMetricsInternal(computed_metrics_[std::make_pair(true, eef[i].group)], eef[i], *getQueryStartState(), payload);
    computeMetricsInternal(computed_metrics_[std::make_pair(false, eef[i].group)], eef[i], *getQueryGoalState(), payload);
  }
}

void PlanningDisplay::computeMetrics(bool start, const std::string &group, double payload)
{ 
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
    if (eef[i].group == group)
      computeMetricsInternal(computed_metrics_[std::make_pair(start, group)], eef[i],
                             start ? *getQueryStartState() : *getQueryGoalState(), payload);
  if (start)
    updateQueryStartState();
  else
    updateQueryGoalState();
}

void PlanningDisplay::computeMetricsInternal(std::map<std::string, double> &metrics, const robot_interaction::RobotInteraction::EndEffector &ee,
                                             const kinematic_state::KinematicState &state, double payload)
{ 
  metrics.clear();
  dynamics_solver::DynamicsSolverPtr ds = getDynamicsSolver(ee.group);
  
  // Max payload
  if (ds)
  {
    double max_payload;
    unsigned int saturated_joint;
    std::vector<double> joint_values;
    state.getJointStateGroup(ee.group)->getVariableValues(joint_values);
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
    double manipulability_index, condition_number;
    if (kinematics_metrics_->getManipulabilityIndex(state, ee.group, manipulability_index))
      metrics["manipulability_index"] = manipulability_index;
    if (kinematics_metrics_->getConditionNumber(state, ee.group, condition_number))
      metrics["condition_number"] = condition_number;
  }
}

static void copyItemIfExists(const std::map<std::string, double> &source, std::map<std::string, double> &dest, const std::string &key)
{
  std::map<std::string, double>::const_iterator it = source.find(key);
  if (it != source.end())
    dest[key] = it->second;
}

void PlanningDisplay::displayMetrics(bool start)
{    
  static const Ogre::Quaternion orientation( 1.0, 0.0, 0.0, 0.0 );
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  
  if (planning_scene_monitor_)
    for (std::size_t i = 0 ; i < eef.size() ; ++i)
    {
      Ogre::Vector3 position(0.0, 0.0, 0.0);
      std::map<std::string, double> text_table; 
      const std::map<std::string, double> &metrics_table = computed_metrics_[std::make_pair(start, eef[i].group)];
      if (compute_weight_limit_property_->getBool())
      {    
        copyItemIfExists(metrics_table, text_table, "max_payload");
        copyItemIfExists(metrics_table, text_table, "saturated_joint");
      }
      if (show_manipulability_index_property_->getBool())
        copyItemIfExists(metrics_table, text_table, "manipulability_index");
      if (show_manipulability_region_property_->getBool())
        copyItemIfExists(metrics_table, text_table, "condition_number");
      if (show_joint_torques_property_->getBool())
      {
        std::size_t nj = planning_scene_monitor_->getKinematicModel()->getJointModelGroup(eef[i].group)->getJointModelNames().size();
        for(size_t j = 0 ; j < nj ; ++j)
        {
          std::stringstream stream;
          stream << "torque[" << j << "]";          
          copyItemIfExists(metrics_table, text_table, stream.str());          
        }
      }
      
      const kinematic_state::LinkState *ls = NULL;
      const kinematic_model::JointModelGroup *jmg = planning_scene_monitor_->getKinematicModel()->getJointModelGroup(eef[i].group);
      if (jmg)
        if (!jmg->getLinkModelNames().empty())
          ls = start ? query_start_state_->getLinkState(jmg->getLinkModelNames().back()) : query_goal_state_->getLinkState(jmg->getLinkModelNames().back());
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

void PlanningDisplay::changedQueryStartState(void)
{
  if (!planning_scene_monitor_)
    return;

  if (query_start_state_property_->getBool())
  {
    if (isEnabled())
    {
      // update link poses
      query_robot_start_->update(PlanningLinkUpdater(query_start_state_));
      query_robot_start_->setVisible(true);  
      
      // update link colors
      planning_scene_monitor_->getPlanningScene()->getCollidingLinks(collision_links_start_, *query_start_state_);
      updateLinkColors();
      
      // update metrics text 
      displayMetrics(true);
    }
  }
  else
    query_robot_start_->setVisible(false);
  context_->queueRender();
  addBackgroundJob(boost::bind(&PlanningDisplay::publishInteractiveMarkers, this));
}

void PlanningDisplay::publishInteractiveMarkers(void)
{
  robot_interaction_->clearInteractiveMarkers();
  if (query_start_state_property_->getBool())
    robot_interaction_->addInteractiveMarkers(*query_start_state_, 0);
  if (query_goal_state_property_->getBool())
    robot_interaction_->addInteractiveMarkers(*query_goal_state_, 1);
  robot_interaction_->publishInteractiveMarkers();
}

void PlanningDisplay::changedQueryStartColor(void)
{
  changedQueryStartState();
}

void PlanningDisplay::changedQueryStartAlpha(void)
{
  query_robot_start_->setAlpha(query_start_alpha_property_->getFloat());
}

void PlanningDisplay::changedQueryGoalState(void)
{
  if (!planning_scene_monitor_)
    return;  
  if (query_goal_state_property_->getBool())
  {
    if (isEnabled())
    { 
      // update link poses
      query_robot_goal_->update(PlanningLinkUpdater(query_goal_state_));
      query_robot_goal_->setVisible(true);  

      // update link colors
      planning_scene_monitor_->getPlanningScene()->getCollidingLinks(collision_links_goal_, *query_goal_state_);
      updateLinkColors();
      
      // update metrics text 
      displayMetrics(false);
    }
  }
  else
    query_robot_goal_->setVisible(false);
  context_->queueRender();
  addBackgroundJob(boost::bind(&PlanningDisplay::publishInteractiveMarkers, this));
}

void PlanningDisplay::changedQueryGoalColor(void)
{
  changedQueryGoalState();
}

void PlanningDisplay::changedQueryGoalAlpha(void)
{
  query_robot_goal_->setAlpha(query_goal_alpha_property_->getFloat());
}

void PlanningDisplay::changedQueryCollidingLinkColor(void)
{
  changedQueryStartState();
  changedQueryGoalState();
}

void PlanningDisplay::updateQueryStartState(void)
{
  addMainLoopJob(boost::bind(&PlanningDisplay::changedQueryStartState, this));  
  context_->queueRender();
}

void PlanningDisplay::updateQueryGoalState(void)
{ 
  addMainLoopJob(boost::bind(&PlanningDisplay::changedQueryGoalState, this));  
  context_->queueRender();
}

void PlanningDisplay::setQueryStartState(const kinematic_state::KinematicStatePtr &start)
{
  query_start_state_ = start; 
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(true, group, metrics_set_payload_property_->getFloat());
  updateQueryStartState();
}

void PlanningDisplay::setQueryGoalState(const kinematic_state::KinematicStatePtr &goal)
{
  query_goal_state_ = goal;
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(false, group, metrics_set_payload_property_->getFloat());
  updateQueryGoalState();
}

void PlanningDisplay::updateLinkColors(void)
{  
  unsetAllColors(query_robot_start_);
  unsetAllColors(query_robot_goal_);
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
  {
    setGroupColor(query_robot_start_, group, query_start_color_property_->getColor());
    setGroupColor(query_robot_goal_, group, query_goal_color_property_->getColor());
    for (std::size_t i = 0 ; i < collision_links_start_.size() ; ++i)
      setLinkColor(query_robot_start_, collision_links_start_[i], query_colliding_link_color_property_->getColor());
    for (std::size_t i = 0 ; i < collision_links_goal_.size() ; ++i)
      setLinkColor(query_robot_goal_, collision_links_goal_[i], query_colliding_link_color_property_->getColor());
  }
}

void PlanningDisplay::changedPlanningGroup(void)
{
  robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());
  computeMetrics(metrics_set_payload_property_->getFloat());
  updateLinkColors();
  addBackgroundJob(boost::bind(&PlanningDisplay::publishInteractiveMarkers, this));
  frame_->changePlanningGroup();
}

void PlanningDisplay::changedWorkspace(void)
{
  renderWorkspaceBox();
}

std::string PlanningDisplay::getCurrentPlanningGroup(void) const
{
  return planning_group_property_->getStdString();
}

// ******************************************************************************************
// Scene Display Time
// ******************************************************************************************

void PlanningDisplay::changedSceneDisplayTime(){}

// ******************************************************************************************
// State Display Time
// ******************************************************************************************
void PlanningDisplay::changedStateDisplayTime(){}

// ******************************************************************************************
// Scene Robot Visible
// ******************************************************************************************
void PlanningDisplay::changedSceneRobotEnabled(void)
{
  if (isEnabled())
    planning_scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );
}

// ******************************************************************************************
// Scene Visible
// ******************************************************************************************
void PlanningDisplay::changedSceneEnabled()
{
  rendered_geometry_node_->setVisible( scene_enabled_property_->getBool() );
}

void PlanningDisplay::displayRobotTrajectory(const kinematic_state::KinematicStatePtr &start_state,
                                             const std::vector<kinematic_state::KinematicStatePtr> &trajectory)
{  
  trajectory_message_to_display_.reset(new TrajectoryMessageToDisplay(start_state, trajectory));
}

void PlanningDisplay::changedDisplayPathVisualEnabled()
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

void PlanningDisplay::changedDisplayPathCollisionEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
    display_path_robot_->setVisible(displaying_trajectory_message_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  }
}

// ******************************************************************************************
// Set or Unset Link Color - Private Function
// ******************************************************************************************
void PlanningDisplay::setLinkColor(const std::string& link_name, const QColor &color)
{
  setLinkColor(planning_scene_robot_, link_name, color );
}

void PlanningDisplay::unsetLinkColor(const std::string& link_name)
{
  unsetLinkColor(planning_scene_robot_, link_name);
}

void PlanningDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor &color)
{
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
  {
    const kinematic_model::JointModelGroup *jmg = 
      planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string> &links = jmg->getLinkModelNames();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        setLinkColor(robot, links[i], color);
    }
  }
}

void PlanningDisplay::unsetAllColors(rviz::Robot* robot)
{ 
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
  {
    const std::vector<std::string> &links = planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
      unsetLinkColor(robot, links[i]);
  }
}

void PlanningDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name )
{
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
  {
    const kinematic_model::JointModelGroup *jmg = 
      planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string> &links = jmg->getLinkModelNames();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        unsetLinkColor(robot, links[i]);
    }
  }
}

// ******************************************************************************************
// Set Link Color
// ******************************************************************************************
void PlanningDisplay::setLinkColor(rviz::Robot* robot,  const std::string& link_name, const QColor &color )
{
  rviz::RobotLink *link = robot->getLink(link_name);
  
  // Check if link exists
  if (link)
    link->setColor( color.redF(), color.greenF(), color.blueF() );
}

// ******************************************************************************************
// Unset Link Color
// ******************************************************************************************
void PlanningDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name )
{
  rviz::RobotLink *link = robot->getLink(link_name);

  // Check if link exists
  if (link) 
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
void PlanningDisplay::loadRobotModel(void)
{  
  planning_group_property_->clearOptions();
  planning_scene_monitor_.reset(); // this so that the destructor of the PlanningSceneMonitor gets called before a new instance of a scene monitor is constructed  
  robot_interaction_.reset();
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString(),
                                                                                 context_->getFrameManager()->getTFClientPtr()));
  if (planning_scene_monitor_->getPlanningScene())
  {   
    robot_interaction::RobotInteraction::InteractionHandlerPtr ihandler(new RvizRobotInteractionHandler(this, context_));
    robot_interaction_.reset(new robot_interaction::RobotInteraction(planning_scene_monitor_->getKinematicModel(), ihandler));
    display_path_robot_->load(*planning_scene_monitor_->getKinematicModel()->getURDF());
    planning_scene_robot_->load(*planning_scene_monitor_->getKinematicModel()->getURDF());
    query_robot_start_->load(*planning_scene_monitor_->getKinematicModel()->getURDF());
    query_robot_goal_->load(*planning_scene_monitor_->getKinematicModel()->getURDF());

    scene_name_property_->setStdString(planning_scene_monitor_->getPlanningScene()->getName());
    planning_scene_monitor_->addUpdateCallback(boost::bind(&PlanningDisplay::sceneMonitorReceivedUpdate, this, _1));
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState()));
    query_start_state_ = ks;
    query_goal_state_.reset(new kinematic_state::KinematicState(*ks));
    
    const std::vector<std::string> &groups = planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroupNames();
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      planning_group_property_->addOptionStd(groups[i]);
    planning_group_property_->sortOptions();
    if (!groups.empty() && planning_group_property_->getStdString().empty())
      planning_group_property_->setStdString(groups[0]);
    planning_scene_render_.reset(new PlanningSceneRender(context_, rendered_geometry_node_, planning_scene_robot_));

    planning_scene_robot_->update(PlanningLinkUpdater(ks));
    kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(planning_scene_monitor_->getKinematicModel()));  
    
    robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());
    computeMetrics(metrics_set_payload_property_->getFloat());
    
    ///////// should not need this
    std::string content;
    if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
    {
      std::string loc;
      if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
        update_nh_.getParam(loc, content);
    }
    TiXmlDocument doc;
    doc.Parse(content.c_str());
    urdf::Model descr;
    descr.initXml(doc.RootElement());
    boost::shared_ptr<urdf::Model> urdf_model;
    urdf_model.reset(new urdf::Model());  
    urdf_model->initXml(doc.RootElement());
    /// \todo Use ModelInterface when new kdl_parser is available
    ///////// 
    
    
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      if (planning_scene_monitor_->getKinematicModel()->getJointModelGroup(groups[i])->isChain()) 
      {
        dynamics_solver_[groups[i]].reset(new dynamics_solver::DynamicsSolver());
        if (!dynamics_solver_[groups[i]]->initialize(urdf_model,
                                                     planning_scene_monitor_->getKinematicModelLoader()->getSRDF(),
                                                     groups[i]))
        {
          dynamics_solver_[groups[i]].reset();
          dynamics_solver_.erase(groups[i]);
        }
      }  
    
    changedQueryStartState();
    changedQueryGoalState(); 
    
    setStatus( rviz::StatusProperty::Ok, "PlanningScene", "Planning Scene Loaded Successfully" );
  }
  else
  {
    planning_scene_monitor_.reset();
    setStatus( rviz::StatusProperty::Error, "PlanningScene", "No Planning Scene Loaded" );
  }
}

void PlanningDisplay::sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  scene_name_property_->setStdString(planning_scene_monitor_->getPlanningScene()->getName());
  root_link_name_property_->setStdString(planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLinkName());
  planning_scene_needs_render_ = true;
  
  std::string group = planning_group_property_->getStdString();
  if (query_start_state_property_->getBool() && !group.empty())
  {
    kinematic_state::JointStateGroup *jsg = query_start_state_->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getVariableValues(joint_state_values);
      *query_start_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      query_start_state_->getJointStateGroup(group)->setVariableValues(joint_state_values);
      updateQueryStartState();
    }
  }

  if (query_goal_state_property_->getBool() && !group.empty())
  {
    kinematic_state::JointStateGroup *jsg = query_goal_state_->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getVariableValues(joint_state_values);
      *query_goal_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      query_goal_state_->getJointStateGroup(group)->setVariableValues(joint_state_values);
      updateQueryGoalState();
    }
  }
  
  if (frame_)
    frame_->sceneUpdate(update_type);
}

// ******************************************************************************************
// Enable
// ******************************************************************************************
void PlanningDisplay::onEnable()
{
  Display::onEnable();
  
  loadRobotModel();

  display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
  display_path_robot_->setVisible(displaying_trajectory_message_);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    trajectory_trail_[i]->setVisualVisible( display_path_visual_enabled_property_->getBool() );
    trajectory_trail_[i]->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
    trajectory_trail_[i]->setVisible(true);
  }
  
  planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
  rendered_geometry_node_->setVisible(scene_enabled_property_->getBool());

  text_to_display_->setVisible(false);

  changedPlanningGroup();
  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  if (show_planning_frame_)
    frame_->enable();
  
  addMainLoopJob(boost::bind(&PlanningDisplay::calculateOffsetPosition, this));
  
  int_marker_display_->setEnabled(true);
  addBackgroundJob(boost::bind(&PlanningDisplay::publishInteractiveMarkers, this));
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void PlanningDisplay::onDisable()
{
  robot_interaction_->clear();
  int_marker_display_->setEnabled(false);
  if (planning_scene_monitor_)
    planning_scene_monitor_->stopSceneMonitor();
  
  display_path_robot_->setVisible(false);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setVisible(false);
  
  rendered_geometry_node_->setVisible(false);
  planning_scene_robot_->setVisible(false);
  query_robot_start_->setVisible(false);
  query_robot_goal_->setVisible(false); 
  frame_->disable(); 
  text_to_display_->setVisible(false);
  Display::onDisable();
}

void PlanningDisplay::showPlanningFrame(bool show)
{
  if (frame_)
  {
    if (show)
      frame_->enable();
    else
      frame_->disable();
  }
  show_planning_frame_ = show;
}

void PlanningDisplay::executeMainLoopJobs(void)
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

void PlanningDisplay::queueRenderSceneGeometry(void)
{
  planning_scene_needs_render_ = true;
}

float PlanningDisplay::getStateDisplayTime(void)
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
void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  int_marker_display_->update(wall_dt, ros_dt);
  
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
    PlanningLinkUpdater plu(displaying_trajectory_message_->start_state_);
    display_path_robot_->update(plu);
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
      {
        PlanningLinkUpdater plu(displaying_trajectory_message_->trajectory_[current_state_]);
        display_path_robot_->update(plu);
      }
      else
      {
        animating_path_ = false;
      }
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

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void PlanningDisplay::calculateOffsetPosition(void)
{  
  if (!planning_scene_monitor_)
    return;

  ros::Time stamp;
  std::string err_string;
  if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), planning_scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp, &err_string) != tf::NO_ERROR)
    return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, planning_scene_monitor_->getPlanningScene()->getPlanningFrame());

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), planning_scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp))
  {
    try
    {
      context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.toStdString().c_str() );
    }
  }

  Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  const tf::Quaternion &q = pose.getRotation();
  Ogre::Quaternion orientation( q.getW(), q.getX(), q.getY(), q.getZ() );
  planning_scene_node_->setPosition(position);
  planning_scene_node_->setOrientation(orientation);
}

// ******************************************************************************************
// Incoming Display Trajectory
// ******************************************************************************************
void PlanningDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  if (planning_scene_monitor_)
    if (msg->model_id != planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getName())
      ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
               msg->model_id.c_str(), planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getName().c_str());
  
  trajectory_message_to_display_.reset(new TrajectoryMessageToDisplay(msg, planning_scene_monitor_->getPlanningScene()));

  if (trail_display_property_->getBool())
    changedShowTrail();
}

// ******************************************************************************************
// Fixed Frame Changed
// ******************************************************************************************
void PlanningDisplay::fixedFrameChanged(void)
{
  Display::fixedFrameChanged(); 
  if (int_marker_display_)
  {
    // we should just call fixedFrameChanged() instead of reconstructing the display
    delete int_marker_display_;
    int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
    int_marker_display_->initialize(context_);
    int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction::RobotInteraction::INTERACTIVE_MARKER_TOPIC + "/update"));
    if (isEnabled())
      int_marker_display_->setEnabled(true);
  }
  calculateOffsetPosition();  
  changedPlanningGroup();
}


} // namespace moveit_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( moveit_rviz_plugin, MotionPlanning, moveit_rviz_plugin::PlanningDisplay, rviz::Display )
