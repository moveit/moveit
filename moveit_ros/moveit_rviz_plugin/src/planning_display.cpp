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

#include "moveit_rviz_plugin/planning_display.h"
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_factory.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <planning_models/conversions.h>
#include <trajectory_processing/trajectory_tools.h>

#include <boost/format.hpp>

#include "planning_link_updater.h"

namespace moveit_rviz_plugin
{

PlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const moveit_msgs::DisplayTrajectory::ConstPtr &message,
                                                                        const planning_scene::PlanningScenePtr &scene)
{
  start_state_.reset(new planning_models::KinematicState(scene->getCurrentState()));
  planning_models::robotStateToKinematicState(*scene->getTransforms(), message->trajectory_start, *start_state_);
  trajectory_processing::convertToKinematicStates(trajectory_, message->trajectory_start, message->trajectory, *start_state_, scene->getTransforms());
}

PlanningDisplay::TrajectoryMessageToDisplay::TrajectoryMessageToDisplay(const planning_models::KinematicStatePtr &start_state,
                                                                        const std::vector<planning_models::KinematicStatePtr> &trajectory) :
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
  update_display_start_state_(false),
  update_display_goal_state_(false),
  update_offset_transforms_(false),
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
  
  // Planning request category -----------------------------------------------------------------------------------------
  
  planning_group_property_ = new rviz::EditableEnumProperty("Planning Group", "", "The name of the group of links to plan for (from the ones defined in the SRDF)",
                                                            plan_category_,
                                                            SLOT( changedPlanningGroup() ), this );

  query_start_state_property_ = new rviz::BoolProperty( "Query Start State", true, "Shows the start state for the motion planning query",
                                                        plan_category_,
                                                        SLOT( changedQueryStartState() ), this );
  
  query_goal_state_property_ = new rviz::BoolProperty( "Query Goal State", true, "Shows the start state for the motion planning query",
                                                       plan_category_,
                                                       SLOT( changedQueryGoalState() ), this );
  
  // Planning scene category -------------------------------------------------------------------------------------------

  scene_name_property_ =
    new rviz::StringProperty( "Scene Name", "(noname)", "Shows the name of the planning scene",
                              scene_category_,
                              SLOT( changedSceneName() ), this );


  root_link_name_property_ =
    new rviz::StringProperty( "Robot Root Link", "", "Shows the name of the root link for the robot model",
                              scene_category_,
                              SLOT( changedRootLinkName() ), this );


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

  display_path_visual_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Visual", true, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                            path_category_,
                            SLOT( changedDisplayPathVisualEnabled() ), this );

  display_path_collision_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Collision", false, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
                            path_category_,
                            SLOT( changedDisplayPathCollisionEnabled() ), this );

  robot_path_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 0.75f, "Specifies the alpha for the robot links",
                             path_category_,
                             SLOT( changedRobotPathAlpha() ), this );
  robot_path_alpha_property_->setMin( 0.0 );
  robot_path_alpha_property_->setMax( 1.0 );

  state_display_time_property_ =
    new rviz::FloatProperty( "State Display Time", 0.05f, "The amount of wall-time to wait in between displaying states along a received trajectory path",
                             path_category_,
                             SLOT( changedStateDisplayTime() ), this );
  state_display_time_property_->setMin(0.0001);

  loop_display_property_ =
    new rviz::BoolProperty( "Loop Animation", false, "Indicates whether the last received path is to be animated in a loop",
                            path_category_,
                            SLOT( changedLoopDisplay() ), this );

  trajectory_topic_property_ =
    new rviz::RosTopicProperty( "Trajectory Topic", "",
                                ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),
                                "The topic on which the moveit_msgs::DisplayTrajectory messages are received",
                                path_category_,
                                SLOT( changedTrajectoryTopic() ), this );

}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
PlanningDisplay::~PlanningDisplay()
{
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
  frame_ = new PlanningFrame(this, context_, window_context->getParentWindow());
  frame_dock_ = window_context->addPane("Motion Planning", frame_);  

  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(RobotInteraction::INTERACTIVE_MARKER_TOPIC + "/update"));

  robot_interaction_.reset(new RobotInteraction(this, context_));

  text_to_display_ = new rviz::MovableText("EMPTY");
  text_to_display_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_to_display_->setCharacterHeight(0.08);
  text_to_display_->setColor(Ogre::ColourValue(0.1f, 0.9f, 0.5f, 1.0f)); 
  text_to_display_->showOnTop();
  text_display_scene_node_->attachObject(text_to_display_);
  text_coll_object_ = context_->getSelectionManager()->createHandle();
  context_->getSelectionManager()->addPickTechnique( text_coll_object_, text_to_display_->getMaterial() );
    /*
  // we should add a selection manager at some point
  SelectionHandlerPtr handler( new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id)) );
  context_->getSelectionManager()->addObject( coll, handler );;
    */ 

  text_to_display_->setVisible(false);
}

void PlanningDisplay::reset()
{
  planning_scene_render_.reset();
  loadRobotModel();
  frame_->disable();
  if (planning_scene_monitor_)
    frame_->enable();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  Display::reset();
}

void PlanningDisplay::changedShowWeightLimit(void)
{   
  if (query_goal_state_property_->getBool())
    displayMetrics(false);
  if (query_start_state_property_->getBool())
    displayMetrics(true);
}

void PlanningDisplay::changedShowManipulabilityIndex(void)
{
  if (query_goal_state_property_->getBool())
    displayMetrics(false);
  if (query_start_state_property_->getBool())
    displayMetrics(true);
}

void PlanningDisplay::changedShowManipulabilityRegion(void)
{
  if (query_goal_state_property_->getBool())
    displayMetrics(false);
  if (query_start_state_property_->getBool())
    displayMetrics(true);
}

void PlanningDisplay::displayTable(const std::map<std::string, double> &values,
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
  text_display_scene_node_->setPosition(pos);
  text_display_scene_node_->setOrientation(orient);
  
  // make sure the node is visible
  text_to_display_->setVisible(true);
}


// ******************************************************************************************
// Robot Description
// ******************************************************************************************
void PlanningDisplay::setRobotDescription(const std::string &name)
{
  robot_description_property_->setStdString( name );
  changedRobotDescription();
}

const std::string PlanningDisplay::getRobotDescription(void)
{
  return robot_description_property_->getStdString();
}

void PlanningDisplay::changedRobotDescription()
{
  if (isEnabled())
    loadRobotModel();
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

// ******************************************************************************************
// Loop Display
// ******************************************************************************************
void PlanningDisplay::changedLoopDisplay(){}

// ******************************************************************************************
// Robot Path Alpha
// ******************************************************************************************
void PlanningDisplay::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
}

// ******************************************************************************************
// Scene Alpha
// ******************************************************************************************
void PlanningDisplay::renderPlanningScene(void)
{
  if (planning_scene_render_ && planning_scene_needs_render_)
  {
    planning_scene_monitor_->lockScene();
    try
    {
      planning_scene_render_->renderPlanningScene(planning_scene_monitor_->getPlanningScene(),
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
  renderPlanningScene();
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

// ******************************************************************************************
// Planning Scene Topic
// ******************************************************************************************
void PlanningDisplay::setPlanningSceneTopic(const std::string &topic)
{
  planning_scene_topic_property_->setStdString( topic );
  changedPlanningSceneTopic();
}

const std::string PlanningDisplay::getPlanningSceneTopic(void)
{
  return planning_scene_topic_property_->getStdString();
}

void PlanningDisplay::changedPlanningSceneTopic(void)
{
  if (planning_scene_monitor_)
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
}

void PlanningDisplay::getContactLinks(const planning_models::KinematicState &state, std::vector<std::string> &links)
{
  links.clear();
  if (planning_scene_monitor_)
  {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = true;
    req.max_contacts = 10;
    req.max_contacts_per_pair = 1;
    planning_scene_monitor_->getPlanningScene()->checkCollision(req, res, state);
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
      for (std::size_t j = 0 ; j < it->second.size() ; ++j)
      {
        if (it->second[j].body_type_1 == collision_detection::BodyTypes::ROBOT_LINK)
          links.push_back(it->second[j].body_name_1);
        if (it->second[j].body_type_2 == collision_detection::BodyTypes::ROBOT_LINK)
          links.push_back(it->second[j].body_name_2);
      }
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
  const std::vector<RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  
  if (planning_scene_monitor_)
    for (std::size_t i = 0 ; i < eef.size() ; ++i)
    {
      Ogre::Vector3 position(0.0, 0.0, 0.0);
      std::map<std::string, double> text_table; 
      const std::map<std::string, double> &metrics_table = robot_interaction_->getComputedMetrics(start, eef[i].group);
      
      if (compute_weight_limit_property_->getBool())
      {    
        copyItemIfExists(metrics_table, text_table, "max_payload");
        copyItemIfExists(metrics_table, text_table, "saturated_joint");
      }
      if (show_manipulability_index_property_->getBool())
        copyItemIfExists(metrics_table, text_table, "manipulability_index");
      if (show_manipulability_region_property_->getBool())
        copyItemIfExists(metrics_table, text_table, "condition_number");
      
      const planning_models::KinematicState::LinkState *ls = NULL;
      const planning_models::KinematicModel::JointModelGroup *jmg = planning_scene_monitor_->getKinematicModel()->getJointModelGroup(eef[i].group);
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
      displayTable(text_table, position, orientation);
    }
}

void PlanningDisplay::changedQueryStartState(void)
{
  if (query_start_state_property_->getBool())
  {
    if (isEnabled())
    {
      // update link poses
      query_robot_start_->update(PlanningLinkUpdater(query_start_state_));
      query_robot_start_->setVisible(true);  
      

      // update link colors
      const planning_models::KinematicModel::JointModelGroup *jmg = NULL;
      std::string group = planning_group_property_->getStdString();
      if (!group.empty())
        jmg = planning_scene_monitor_->getKinematicModel()->getJointModelGroup(group);
      
      for (std::size_t i = 0 ; i < collision_links_start_.size() ; ++i)
        if (jmg && jmg->hasLinkModel(collision_links_start_[i]))
          setLinkColor(query_robot_start_, collision_links_start_[i], 0.0f, 1.0f, 0.0f);
        else
          unsetLinkColor(query_robot_start_, collision_links_start_[i]);
      
      getContactLinks(*query_start_state_, collision_links_start_);
      for (std::size_t i = 0 ; i < collision_links_start_.size() ; ++i)
        setLinkColor(query_robot_start_, collision_links_start_[i], 1.0f, 0.0f, 0.0f);

      // update metrics text 
      displayMetrics(true);
    }
  }
  else
    query_robot_start_->setVisible(false);  
  robot_interaction_->publishInteractiveMarkers();
}

void PlanningDisplay::changedQueryGoalState(void)
{
  if (query_goal_state_property_->getBool())
  {
    if (isEnabled())
    { 
      // update link poses
      query_robot_goal_->update(PlanningLinkUpdater(query_goal_state_));
      query_robot_goal_->setVisible(true);  


      // update link colors
      const planning_models::KinematicModel::JointModelGroup *jmg = NULL;
      std::string group = planning_group_property_->getStdString();
      if (!group.empty())
        jmg = planning_scene_monitor_->getKinematicModel()->getJointModelGroup(group);
      
      for (std::size_t i = 0 ; i < collision_links_goal_.size() ; ++i)
        if (jmg && jmg->hasLinkModel(collision_links_goal_[i]))
          setLinkColor(query_robot_goal_, collision_links_goal_[i], 1.0f, 0.5f, 0.0f);
        else
          unsetLinkColor(query_robot_goal_, collision_links_goal_[i]);
      
      getContactLinks(*query_goal_state_, collision_links_goal_);
      for (std::size_t i = 0 ; i < collision_links_goal_.size() ; ++i)
        setLinkColor(query_robot_goal_, collision_links_goal_[i], 1.0f, 0.0f, 0.0f);

      // update metrics text 
      displayMetrics(false);
    }
  }
  else
    query_robot_goal_->setVisible(false);
  robot_interaction_->publishInteractiveMarkers();
}

void PlanningDisplay::updateQueryStartState(void)
{
  // we mark the fact that the next call to update() should update the start state 
  update_display_start_state_ = true;
}

void PlanningDisplay::updateQueryGoalState(void)
{ 
  // we mark the fact that the next call to update() should update the goal state 
  update_display_goal_state_ = true;
}

void PlanningDisplay::setQueryStartState(const planning_models::KinematicStatePtr &start)
{
  query_start_state_ = start; 
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    robot_interaction_->computeMetrics(true, group);
  updateQueryStartState();
}

void PlanningDisplay::setQueryGoalState(const planning_models::KinematicStatePtr &goal)
{
  query_goal_state_ = goal;
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    robot_interaction_->computeMetrics(false, group);
  updateQueryGoalState();
}

void PlanningDisplay::changedPlanningGroup(void)
{
  unsetAllColors(query_robot_start_);
  unsetAllColors(query_robot_goal_);
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
  {
    setGroupColor(query_robot_start_, group, 0.0f, 1.0f, 0.0f);
    setGroupColor(query_robot_goal_, group, 1.0f, 0.5f, 0.0f);
    for (std::size_t i = 0 ; i < collision_links_start_.size() ; ++i)
      setLinkColor(query_robot_start_, collision_links_start_[i], 1.0f, 0.0f, 0.0f);
    for (std::size_t i = 0 ; i < collision_links_goal_.size() ; ++i)
      setLinkColor(query_robot_goal_, collision_links_start_[i], 1.0f, 0.0f, 0.0f);
  }
  robot_interaction_->decideActiveComponents();
  robot_interaction_->computeMetrics();
  robot_interaction_->publishInteractiveMarkers();
  frame_->changePlanningGroup();
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

void PlanningDisplay::displayRobotTrajectory(const planning_models::KinematicStatePtr &start_state,
                                             const std::vector<planning_models::KinematicStatePtr> &trajectory)
{  
  trajectory_message_to_display_.reset(new TrajectoryMessageToDisplay(start_state, trajectory));
}

void PlanningDisplay::changedDisplayPathVisualEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
    display_path_robot_->setVisible(displaying_trajectory_message_);
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
  }
}

// ******************************************************************************************
// Set or Unset Link Color - Private Function
// ******************************************************************************************
void PlanningDisplay::setLinkColor( const std::string& link_name, float red, float green, float blue )
{
  setLinkColor(display_path_robot_, link_name, red, green, blue );
  setLinkColor(planning_scene_robot_, link_name, red, green, blue );
}

void PlanningDisplay::unsetLinkColor( const std::string& link_name )
{
  unsetLinkColor(display_path_robot_, link_name);
  unsetLinkColor(planning_scene_robot_, link_name);
}

void PlanningDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, float red, float green, float blue )
{
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
  {
    const planning_models::KinematicModel::JointModelGroup *jmg = 
      planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string> &links = jmg->getLinkModelNames();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        setLinkColor(robot, links[i], red, green, blue);
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
    const planning_models::KinematicModel::JointModelGroup *jmg = 
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
void PlanningDisplay::setLinkColor(rviz::Robot* robot,  const std::string& link_name, float red, float green, float blue )
{
  rviz::RobotLink *link = robot->getLink(link_name);
  
  // Check if link exists
  if (link)
    link->setColor( red, green, blue );
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
  std::string content;
  if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
  {
    std::string loc;
    if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
    {
      update_nh_.getParam(loc, content);
    }
  }

  TiXmlDocument doc;
  doc.Parse(content.c_str());
  if (!doc.RootElement())
  {
    return;
  }

  urdf::Model descr;
  descr.initXml(doc.RootElement());
  display_path_robot_->load(doc.RootElement(), descr);
  planning_scene_robot_->load(doc.RootElement(), descr);
  query_robot_start_->load(doc.RootElement(), descr);
  query_robot_goal_->load(doc.RootElement(), descr);

  loadPlanningSceneMonitor();
  robot_interaction_->decideActiveComponents();
  robot_interaction_->computeMetrics();

  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(planning_scene_monitor_->getKinematicModel()));  

  boost::shared_ptr<urdf::Model> urdf_model;
  urdf_model.reset(new urdf::Model());  
  urdf_model->initXml(doc.RootElement());
  /// \todo Use ModelInterface when new kdl_parser is available
  const std::vector<std::string> &groups = planning_scene_monitor_->getKinematicModel()->getJointModelGroupNames();
  for(std::size_t i = 0 ; i < groups.size() ; ++i)
    if (planning_scene_monitor_->getKinematicModel()->getJointModelGroup(groups[i])->isChain()) 
    {
      dynamics_solver_[groups[i]].reset(new dynamics_solver::DynamicsSolver());
      if (!dynamics_solver_[groups[i]]->initialize(urdf_model,
                                                   planning_scene_monitor_->getKinematicModelLoader()->getSRDF(),
                                                   groups[i]))
        dynamics_solver_[groups[i]].reset();
    }
}

void PlanningDisplay::loadPlanningSceneMonitor(void)
{
  planning_group_property_->clearOptions();
  planning_scene_monitor_.reset(); // this so that the destructor of the PlanningSceneMonitor gets called before a new instance of a scene monitor is constructed  
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString(),
                                                                        context_->getFrameManager()->getTFClientPtr()));
  if (planning_scene_monitor_->getPlanningScene())
  {
    planning_scene_monitor_->getPlanningScene()->setName(scene_name_property_->getStdString());
    planning_scene_monitor_->addUpdateCallback(boost::bind(&PlanningDisplay::sceneMonitorReceivedUpdate, this, _1));
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    planning_models::KinematicStatePtr ks(new planning_models::KinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState()));
    planning_scene_robot_->update(PlanningLinkUpdater(ks));
    query_start_state_ = ks;
    query_goal_state_.reset(new planning_models::KinematicState(*ks));
    
    const std::vector<std::string> &groups = planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroupNames();
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      planning_group_property_->addOptionStd(groups[i]);
    planning_group_property_->sortOptions();
    if (!groups.empty() && planning_group_property_->getStdString().empty())
      planning_group_property_->setStdString(groups[0]);
    changedQueryStartState();
    changedQueryGoalState();

    planning_scene_render_.reset(new PlanningSceneRender(context_, rendered_geometry_node_, planning_scene_robot_));
  }
  else
    planning_scene_monitor_.reset();
}

void PlanningDisplay::sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  scene_name_property_->setStdString(planning_scene_monitor_->getPlanningScene()->getName());
  root_link_name_property_->setStdString(planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLinkName());
  planning_scene_needs_render_ = true;
  
  std::string group = planning_group_property_->getStdString();
  if (query_start_state_property_->getBool() && !group.empty())
  {
    planning_models::KinematicState::JointStateGroup *jsg = query_start_state_->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getGroupStateValues(joint_state_values);
      *query_start_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      query_start_state_->getJointStateGroup(group)->setStateValues(joint_state_values);
    }
  }

  if (query_goal_state_property_->getBool() && !group.empty())
  {
    planning_models::KinematicState::JointStateGroup *jsg = query_goal_state_->getJointStateGroup(group);
    if (jsg)
    {
      std::map<std::string, double> joint_state_values;
      jsg->getGroupStateValues(joint_state_values);
      *query_goal_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      query_goal_state_->getJointStateGroup(group)->setStateValues(joint_state_values);
    }
  }
  
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

  planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
  rendered_geometry_node_->setVisible(scene_enabled_property_->getBool());

  changedPlanningGroup();
  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  frame_->enable();
  update_offset_transforms_ = true;
  int_marker_display_->setEnabled(true);

  robot_interaction_->publishInteractiveMarkers();
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
  rendered_geometry_node_->setVisible(false);
  planning_scene_robot_->setVisible(false);
  query_robot_start_->setVisible(false);
  query_robot_goal_->setVisible(false); 
  frame_->disable(); 
  text_to_display_->setVisible(false);
  Display::onDisable();
}

// ******************************************************************************************
// Update
// ******************************************************************************************
void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  int_marker_display_->update(wall_dt, ros_dt);
  
  Display::update(wall_dt, ros_dt);
  
  if (update_display_start_state_)
  {
    update_display_start_state_ = false;
    changedQueryStartState();
  }
  
  if (update_display_goal_state_)
  {
    update_display_goal_state_ = false;
    changedQueryGoalState();
  }

  if (!planning_scene_monitor_)
    return;

  if (update_offset_transforms_)
    calculateOffsetPosition();
  
  if (!animating_path_ && !trajectory_message_to_display_ && loop_display_property_->getBool() && displaying_trajectory_message_)
  {
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = state_display_time_property_->getFloat() + 1.0f;
  }

  if (!animating_path_ && trajectory_message_to_display_)
  {
    planning_scene_monitor_->updateFrameTransforms();
    displaying_trajectory_message_ = trajectory_message_to_display_;
    display_path_robot_->setVisible(isEnabled());
    trajectory_message_to_display_.reset();
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = state_display_time_property_->getFloat() + 1.0f;
    PlanningLinkUpdater plu(displaying_trajectory_message_->start_state_);
    display_path_robot_->update(plu);
  }

  if (animating_path_)
  { 
    if (current_state_time_ > state_display_time_property_->getFloat())
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
  update_offset_transforms_ = false;
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
}

// ******************************************************************************************
// Fixed Frame Changed
// ******************************************************************************************
void PlanningDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();  
  robot_interaction_->publishInteractiveMarkers();
}


} // namespace moveit_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( moveit_rviz_plugin, MotionPlanning, moveit_rviz_plugin::PlanningDisplay, rviz::Display )
