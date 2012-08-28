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

#include <moveit_rviz_plugin/planning_display.h>
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
#include <rviz/panel_dock_widget.h>
#include <rviz/window_manager_interface.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>
#include <planning_models/conversions.h>
#include <trajectory_processing/trajectory_tools.h>

#include "planning_link_updater.h"

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// Structs
// ******************************************************************************************
struct PlanningDisplay::ReceivedTrajectoryMessage
{
  ReceivedTrajectoryMessage(const moveit_msgs::DisplayTrajectory::ConstPtr &message, const planning_scene::PlanningScenePtr &scene) : message_(message)
  {
    start_state_.reset(new planning_models::KinematicState(scene->getCurrentState()));
    planning_models::robotStateToKinematicState(*scene->getTransforms(), message_->trajectory_start, *start_state_);
    trajectory_processing::convertToKinematicStates(trajectory_, message_->trajectory_start, message_->trajectory, *start_state_, scene->getTransforms());
  }

  moveit_msgs::DisplayTrajectory::ConstPtr message_;
  planning_models::KinematicStatePtr start_state_;
  std::vector<planning_models::KinematicStatePtr> trajectory_;
};

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
PlanningDisplay::PlanningDisplay() :
  Display(),
  frame_(NULL),
  frame_dock_(NULL),
  new_display_trajectory_(false),
  animating_path_(false),
  current_scene_time_(0.0f)
{
  last_scene_render_ = ros::Time::now();


  // Category Groups
  plan_category_  = new rviz::Property( "Planning Request",   QVariant(), "", this );
  path_category_  = new rviz::Property( "Planned Path",   QVariant(), "", this );
  scene_category_ = new rviz::Property( "Planning Scene", QVariant(), "", this );

  // Plannint request category -----------------------------------------------------------------------------------------
  
  planning_group_property_ = new rviz::EditableEnumProperty("Planning Group", "", "The name of the group of links to plan for (from the ones defined in the SRDF)",
                                                            plan_category_,
                                                            SLOT( changedPlanningGroup() ), this );

  query_start_state_property_ = new rviz::BoolProperty( "Query Start State", true, "Shows the start state for the motion planning query",
                                                        plan_category_,
                                                        SLOT( changedQueryStartState() ), this );
  
  query_goal_state_property_ = new rviz::BoolProperty( "Query Goal State", false, "Shows the start state for the motion planning query",
                                                       plan_category_,
                                                       SLOT( changedQueryGoalState() ), this );
  
  // Planning scene category -------------------------------------------------------------------------------------------

  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "", "The name of the ROS parameter where the URDF for the robot is loaded",
                              scene_category_,
                              SLOT( changedRobotDescription() ), this );

  scene_name_property_ =
    new rviz::StringProperty( "Scene Name", "", "Shows the name of the planning scene",
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
    new rviz::RosTopicProperty( "Planning Scene Topic", "",
                                ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                "The topic on which the moveit_msgs::PlanningScene messages are received",
                                scene_category_,
                                SLOT( changedPlanningSceneTopic() ), this );

  // Path category ----------------------------------------------------------------------------------------------------

  display_path_visual_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Visual", "", "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                            path_category_,
                            SLOT( changedDisplayPathVisualEnabled() ), this );

  display_path_collision_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Collision", "", "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
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
  delete frame_dock_;
}

// ******************************************************************************************
//
// ******************************************************************************************
void PlanningDisplay::onInitialize()
{
  display_path_robot_ = new rviz::Robot(context_, "Planned Path", path_category_ );
  display_path_robot_->setVisible(true); 
  display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );

  scene_robot_ = new rviz::Robot(context_, "Planning Scene", scene_category_ );
  scene_robot_->setCollisionVisible(false);
  scene_robot_->setVisualVisible(true);
  scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );

  query_robot_ = new rviz::Robot(context_, "Planning Request", plan_category_ );
  query_robot_->setCollisionVisible(false);
  query_robot_->setVisualVisible(true);
  query_robot_->setVisible( query_start_state_property_->getBool() );
  
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible(scene_enabled_property_->getBool());

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new PlanningFrame(context_, window_context->getParentWindow());
  frame_dock_ = window_context->addPane( "Motion Planning", frame_ );
}

// ******************************************************************************************
//
// ******************************************************************************************
void PlanningDisplay::reset()
{
  loadPlanningSceneMonitor();
  clearRenderedGeometry();
  incoming_trajectory_message_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  new_display_trajectory_ = false;
}

// ******************************************************************************************
//
// ******************************************************************************************
void PlanningDisplay::clearRenderedGeometry()
{
  scene_shapes_.clear();
  for (std::size_t i = 0 ; i < manual_objects_.size() ; ++i)
    context_->getSceneManager()->destroyManualObject(manual_objects_[i]);

  manual_objects_.clear();
  if (!material_name_.empty())
  {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_name_ = "";
  }
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
  if (scene_monitor_ && scene_monitor_->getPlanningScene())
    scene_monitor_->getPlanningScene()->setName(scene_name_property_->getStdString());
}

// ******************************************************************************************
// Root Link Name
// ******************************************************************************************
void PlanningDisplay::changedRootLinkName()
{  
  if (scene_monitor_ && scene_monitor_->getPlanningScene())
    root_link_name_property_->setStdString( scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLinkName() );
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
void PlanningDisplay::changedSceneAlpha()
{
  renderPlanningScene();
}

// ******************************************************************************************
// Scene Robot Alpha
// ******************************************************************************************
void PlanningDisplay::changedRobotSceneAlpha()
{
  scene_robot_->setAlpha(robot_scene_alpha_property_->getFloat());
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
  if (scene_monitor_)
    scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
}

void PlanningDisplay::changedQueryStartState(void)
{
  if (query_start_state_property_->getBool())
  {
    query_goal_state_property_->setBool(false);
    if (isEnabled())
    {
      query_robot_->setVisible(true);
      query_robot_->update(PlanningLinkUpdater(query_start_state_));
      changedPlanningGroup();
    }
  }
  else
    query_robot_->setVisible(isEnabled() && query_goal_state_property_->getBool());
}

void PlanningDisplay::changedQueryGoalState(void)
{
  if (query_goal_state_property_->getBool())
  {
    query_start_state_property_->setBool(false);
    if (isEnabled())
    {
      query_robot_->setVisible(true);
      query_robot_->update(PlanningLinkUpdater(query_goal_state_));
      changedPlanningGroup();
    }
  }
  else
    query_robot_->setVisible(isEnabled() && query_start_state_property_->getBool());

}

void PlanningDisplay::changedPlanningGroup(void)
{
  unsetAllColors(query_robot_);
  if (query_goal_state_property_->getBool())
    setGroupColor(query_robot_, planning_group_property_->getStdString(), 0.0f, 1.0f, 0.0f);
  else
    if (query_start_state_property_->getBool())
      setGroupColor(query_robot_, planning_group_property_->getStdString(), 1.0f, 0.1f, 0.5f);
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
    scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );
}

// ******************************************************************************************
// Scene Visible
// ******************************************************************************************
void PlanningDisplay::changedSceneEnabled()
{
  scene_node_->setVisible( scene_enabled_property_->getBool() );
}

// ******************************************************************************************
// Visual Visible
// ******************************************************************************************
void PlanningDisplay::displayRobotPath(bool visible)
{
  display_path_robot_->setVisible(visible);
}

void PlanningDisplay::changedDisplayPathVisualEnabled()
{
  if (isEnabled())
    display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
}

// ******************************************************************************************
// Collision Visible
// ******************************************************************************************

void PlanningDisplay::changedDisplayPathCollisionEnabled()
{
  if (isEnabled())
    display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );
}

// ******************************************************************************************
// Set or Unset Link Color - Private Function
// ******************************************************************************************
void PlanningDisplay::setLinkColor( const std::string& link_name, float red, float green, float blue )
{
  setLinkColor(display_path_robot_, link_name, red, green, blue );
  setLinkColor(scene_robot_, link_name, red, green, blue );
  setLinkColor(query_robot_, link_name, red, green, blue );
}

void PlanningDisplay::unsetLinkColor( const std::string& link_name )
{
  unsetLinkColor(display_path_robot_, link_name);
  unsetLinkColor(scene_robot_, link_name);
  unsetLinkColor(query_robot_, link_name);
}

void PlanningDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, float red, float green, float blue )
{
  if (scene_monitor_ && scene_monitor_->getPlanningScene())
  {
    const planning_models::KinematicModel::JointModelGroup *jmg = 
      scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroup(group_name);
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
  if (scene_monitor_ && scene_monitor_->getPlanningScene())
  {
    const std::vector<std::string> &links = scene_monitor_->getPlanningScene()->getKinematicModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
      unsetLinkColor(robot, links[i]);
  }
}

void PlanningDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name )
{
  if (scene_monitor_ && scene_monitor_->getPlanningScene())
  {
    const planning_models::KinematicModel::JointModelGroup *jmg = 
      scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroup(group_name);
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
  scene_robot_->load(doc.RootElement(), descr);
  query_robot_->load(doc.RootElement(), descr);

  loadPlanningSceneMonitor();
}

void PlanningDisplay::loadPlanningSceneMonitor(void)
{
  planning_group_property_->clearOptions();
  scene_monitor_.reset();// this so that the destructor of the PlanningSceneMonitor gets called before a new instance of a scene monitor is constructed  
  scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString()));
  if (scene_monitor_->getPlanningScene())
  {
    scene_monitor_->setUpdateCallback(boost::bind(&PlanningDisplay::sceneMonitorReceivedUpdate, this, _1));
    scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    planning_models::KinematicStatePtr ks(new planning_models::KinematicState(scene_monitor_->getPlanningScene()->getCurrentState()));
    scene_robot_->update(PlanningLinkUpdater(ks));
    query_start_state_ = ks;
    query_goal_state_.reset(new planning_models::KinematicState(*ks));
    
    const std::vector<std::string> &groups = scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroupNames();
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      planning_group_property_->addOptionStd(groups[i]);
    planning_group_property_->sortOptions();
    if (!groups.empty() && planning_group_property_->getStdString().empty())
      planning_group_property_->setStdString(groups[0]);
    changedQueryStartState();
    changedQueryGoalState();
  }
  else
    scene_monitor_.reset();
}

void PlanningDisplay::sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  scene_name_property_->setStdString(scene_monitor_->getPlanningScene()->getName());
  root_link_name_property_->setStdString(scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLinkName());
  
  
  
}

// ******************************************************************************************
// Enable
// ******************************************************************************************
void PlanningDisplay::onEnable()
{
  loadRobotModel();

  display_path_robot_->setVisible(true); 
  display_path_robot_->setVisualVisible( display_path_visual_enabled_property_->getBool() );
  display_path_robot_->setCollisionVisible( display_path_collision_enabled_property_->getBool() );

  scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
  scene_node_->setVisible(scene_enabled_property_->getBool());

  query_robot_->setVisible(query_start_state_property_->getBool() || query_goal_state_property_->getBool());

  if (frame_dock_)
    frame_dock_->show();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void PlanningDisplay::onDisable()
{
  if (scene_monitor_)
    scene_monitor_->stopSceneMonitor();
  display_path_robot_->setVisible(false);
  scene_node_->setVisible(false);
  scene_robot_->setVisible(false);
  query_robot_->setVisible(false);
  if (frame_dock_)
    frame_dock_->hide();
}

// ******************************************************************************************
// Render Shape
// ******************************************************************************************
void PlanningDisplay::renderShape(Ogre::SceneNode *node, const shapes::Shape *s, const Eigen::Affine3d &p, const rviz::Color &color, float alpha)
{
  rviz::Shape* ogre_shape = NULL;
  switch (s->type)
  {
  case shapes::SPHERE:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Sphere,
                                   context_->getSceneManager(), node);
      double d = 2.0 * static_cast<const shapes::Sphere*>(s)->radius;
      ogre_shape->setScale(Ogre::Vector3(d, d, d));
    }
    break;
  case shapes::BOX:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Cube,
                                   context_->getSceneManager(), node);
      const double* sz = static_cast<const shapes::Box*>(s)->size;
      ogre_shape->setScale(Ogre::Vector3(sz[0], sz[1], sz[2]));
    }
    break;
  case shapes::CYLINDER:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Cylinder,
                                   context_->getSceneManager(), node);
      double d = 2.0 * static_cast<const shapes::Cylinder*>(s)->radius;
      double z = static_cast<const shapes::Cylinder*>(s)->length;
      ogre_shape->setScale(Ogre::Vector3(d, z, d)); // the shape has z as major axis, but the rendered cylinder has y as major axis (assuming z is upright);
    }
    break;
  case shapes::MESH:
    {
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(s);
      if (mesh->triangle_count > 0)
      {
        // check if we need to construct the material
        if (material_name_.empty())
        {
          material_name_ = "Planning Display Mesh Material";
          material_ = Ogre::MaterialManager::getSingleton().create( material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
          material_->setReceiveShadows(false);
          material_->getTechnique(0)->setLightingEnabled(true);
          material_->setCullingMode(Ogre::CULL_NONE);
          material_->getTechnique(0)->setAmbient(color.r_, color.g_, color.b_);
          material_->getTechnique(0)->setDiffuse(0, 0, 0, alpha);
          if (alpha < 0.9998)
          {
            material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
            material_->getTechnique(0)->setDepthWriteEnabled( false );
          }
          else
          {
            material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
            material_->getTechnique(0)->setDepthWriteEnabled( true );
          }
        }

        std::string name = "Planning Display Mesh " + boost::lexical_cast<std::string>(manual_objects_.size());
        Ogre::ManualObject *manual_object = context_->getSceneManager()->createManualObject(name);
        manual_object->estimateVertexCount(mesh->triangle_count * 3);
        manual_object->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        for (unsigned int i = 0 ; i < mesh->triangle_count ; ++i)
        {
          unsigned int i3 = i * 3;
          for (int k = 0 ; k < 3 ; ++k)
          {
            unsigned int vi = 3 * mesh->triangles[i3 + k];
            const Eigen::Vector3d &v = p * Eigen::Vector3d(mesh->vertices[vi], mesh->vertices[vi + 1], mesh->vertices[vi + 2]);
            manual_object->position(v.x(), v.y(), v.z());
          }
        }
        manual_object->end();
        node->attachObject(manual_object);
        manual_objects_.push_back(manual_object);
      }
    }
    break;
  default:
    break;
  }

  if (ogre_shape)
  {
    ogre_shape->setColor(color.r_, color.g_, color.b_, alpha);
    Ogre::Vector3 position(p.translation().x(), p.translation().y(), p.translation().z());
    Eigen::Quaterniond q(p.rotation());
    Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

    if (s->type == shapes::CYLINDER)
    {
      // in geometric shapes, the z axis of the cylinder is it height;
      // for the rviz shape, the y axis is the height; we add a transform to fix this
      static Ogre::Quaternion fix(Ogre::Radian(M_PI/2.0), Ogre::Vector3(1.0, 0.0, 0.0));
      orientation = fix * orientation;
    }

    ogre_shape->setPosition(position);
    ogre_shape->setOrientation(orientation);
    scene_shapes_.push_back(boost::shared_ptr<rviz::Shape>(ogre_shape));
  }
}

// ******************************************************************************************
// Render Planning Scene
// ******************************************************************************************
void PlanningDisplay::renderPlanningScene(void)
{
  static rviz::Color env_color(0.2f, 0.9f, 0.2f);
  static rviz::Color attached_color(0.6f, 0.6f, 0.6f);

  if (!scene_monitor_)
    return;

  scene_monitor_->lockScene();
  ros::Time last_update = scene_monitor_->getLastUpdateTime();
  scene_monitor_->unlockScene();
  if (last_update <= last_scene_render_)
    return;

  clearRenderedGeometry();

  scene_monitor_->lockScene();
  last_scene_render_ = scene_monitor_->getLastUpdateTime();
  try
  {
    planning_models::KinematicStateConstPtr ks(new planning_models::KinematicState(scene_monitor_->getPlanningScene()->getCurrentState()));
    scene_robot_->update(PlanningLinkUpdater(ks));
    collision_detection::CollisionWorldConstPtr cworld = scene_monitor_->getPlanningScene()->getCollisionWorld();
    const std::vector<std::string> &ids = cworld->getObjectIds();
    for (std::size_t i = 0 ; i < ids.size() ; ++i)
    {
      collision_detection::CollisionWorld::ObjectConstPtr o = cworld->getObject(ids[i]);
      rviz::Color color = env_color;
      if (scene_monitor_->getPlanningScene()->hasColor(ids[i]))
      {
        const std_msgs::ColorRGBA &c = scene_monitor_->getPlanningScene()->getColor(ids[i]);
        color.r_ = c.r; color.g_ = c.g; color.b_ = c.b;
      }
      for (std::size_t j = 0 ; j < o->shapes_.size() ; ++j)
        renderShape(scene_node_, o->shapes_[j].get(), o->shape_poses_[j], color, scene_alpha_property_->getFloat());
    }

    std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
    scene_monitor_->getPlanningScene()->getCurrentState().getAttachedBodies(attached_bodies);
    for (std::size_t i = 0 ; i < attached_bodies.size() ; ++i)
    {
      rviz::Color color = attached_color;
      if (scene_monitor_->getPlanningScene()->hasColor(attached_bodies[i]->getName()))
      {
        const std_msgs::ColorRGBA &c = scene_monitor_->getPlanningScene()->getColor(attached_bodies[i]->getName());
        color.r_ = c.r; color.g_ = c.g; color.b_ = c.b;
      }
      const EigenSTL::vector_Affine3d &ab_t = attached_bodies[i]->getGlobalCollisionBodyTransforms();
      const std::vector<shapes::ShapeConstPtr> &ab_shapes = attached_bodies[i]->getShapes();
      for (std::size_t j = 0 ; j < ab_shapes.size() ; ++j)
      {
        renderShape(scene_robot_->getVisualNode(), ab_shapes[j].get(), ab_t[j], color, robot_scene_alpha_property_->getFloat());
        renderShape(scene_robot_->getCollisionNode(), ab_shapes[j].get(), ab_t[j], color, robot_scene_alpha_property_->getFloat());
      }
    }
  }
  catch(...)
  {
    scene_monitor_->unlockScene();
    throw;
  }
  scene_monitor_->unlockScene();

  scene_node_->setVisible(scene_enabled_property_->getBool());
}

// ******************************************************************************************
// Update
// ******************************************************************************************
void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  if (!scene_monitor_)
    return;

  if (!animating_path_ && !new_display_trajectory_ && loop_display_property_->getBool() && displaying_trajectory_message_)
  {
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = state_display_time_property_->getFloat() + 1.0f;
  }

  if (!animating_path_ && new_display_trajectory_)
  {
    scene_monitor_->updateFrameTransforms();
    displaying_trajectory_message_.reset(new ReceivedTrajectoryMessage(incoming_trajectory_message_, scene_monitor_->getPlanningScene()));
    animating_path_ = true;
    new_display_trajectory_ = false;
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
  if (!scene_monitor_)
    return;

  ros::Time stamp;
  std::string err_string;
  if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp, &err_string) != tf::NO_ERROR)
    return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, scene_monitor_->getPlanningScene()->getPlanningFrame());

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp))
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
  display_path_robot_->setPosition(position);
  display_path_robot_->setOrientation(orientation);
  scene_robot_->setPosition(position);
  scene_robot_->setOrientation(orientation);
  query_robot_->setPosition(position);
  query_robot_->setOrientation(orientation);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

// ******************************************************************************************
// Incoming Display Trajectory
// ******************************************************************************************
void PlanningDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  incoming_trajectory_message_ = msg;
  if (scene_monitor_)
    if (msg->model_id != scene_monitor_->getPlanningScene()->getKinematicModel()->getName())
      ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
               msg->model_id.c_str(), scene_monitor_->getPlanningScene()->getKinematicModel()->getName().c_str());
  new_display_trajectory_ = true;
}

// ******************************************************************************************
// Fixed Frame Changed
// ******************************************************************************************
void PlanningDisplay::fixedFrameChanged()
{
  calculateOffsetPosition();
}


} // namespace moveit_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( moveit_rviz_plugin, MotionPlanning, moveit_rviz_plugin::PlanningDisplay, rviz::Display )
