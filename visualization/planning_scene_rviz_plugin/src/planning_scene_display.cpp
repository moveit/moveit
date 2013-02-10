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

/* Author: Ioan Sucan */

#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreSceneManager.h>

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
PlanningSceneDisplay::PlanningSceneDisplay() :
  Display(),  
  current_scene_time_(0.0f),
  planning_scene_needs_render_(true)
{
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                              this,
                              SLOT( changedRobotDescription() ), this );
  
  planning_scene_topic_property_ =
    new rviz::RosTopicProperty( "Planning Scene Topic", "planning_scene",
                                ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                "The topic on which the moveit_msgs::PlanningScene messages are received",
                                this,
                                SLOT( changedPlanningSceneTopic() ), this );

  // Category Groups
  scene_category_ = new rviz::Property( "Scene Geometry", QVariant(), "", this );
  robot_category_  = new rviz::Property( "Scene Robot",   QVariant(), "", this );

  // Planning scene category -------------------------------------------------------------------------------------------
  scene_name_property_ =
    new rviz::StringProperty( "Scene Name", "(noname)", "Shows the name of the planning scene",
                              scene_category_,
                              SLOT( changedSceneName() ), this );
  scene_name_property_->setShouldBeSaved(false);
  scene_enabled_property_ =
    new rviz::BoolProperty( "Show Scene Geometry", true, "Indicates whether planning scenes should be displayed",
                            scene_category_,
                            SLOT( changedSceneEnabled() ), this );

  scene_alpha_property_ =
    new rviz::FloatProperty( "Scene Alpha", 0.9f, "Specifies the alpha for the robot links",
                             scene_category_,
                             SLOT( changedSceneAlpha() ), this );
  scene_alpha_property_->setMin( 0.0 );
  scene_alpha_property_->setMax( 1.0 );

  scene_color_property_ = new rviz::ColorProperty( "Scene Color", QColor(50, 230, 50), "The color for the planning scene obstacles (if a color is not defined)",
                                                   scene_category_,
                                                   SLOT( changedSceneColor() ), this );

  root_link_name_property_ =
    new rviz::StringProperty( "Robot Root Link", "", "Shows the name of the root link for the robot model",
                              robot_category_,
                              SLOT( changedRootLinkName() ), this );
  root_link_name_property_->setReadOnly(true);
  
  scene_robot_enabled_property_ =
    new rviz::BoolProperty( "Show Scene Robot", true, "Indicates whether the robot state specified by the planning scene should be displayed",
                            robot_category_,
                            SLOT( changedSceneRobotEnabled() ), this );

  robot_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                             robot_category_,
                             SLOT( changedRobotSceneAlpha() ), this );
  robot_alpha_property_->setMin( 0.0 );
  robot_alpha_property_->setMax( 1.0 );

  attached_body_color_property_ = new rviz::ColorProperty( "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                           robot_category_,
                                                           SLOT( changedAttachedBodyColor() ), this );
  
  scene_display_time_property_ =
    new rviz::FloatProperty( "Scene Display Time", 0.2f, "The amount of wall-time to wait in between rendering updates to the planning scene (if any)",
                             scene_category_,
                             SLOT( changedSceneDisplayTime() ), this );
  scene_display_time_property_->setMin(0.0001);
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
PlanningSceneDisplay::~PlanningSceneDisplay()
{ 
  planning_scene_render_.reset();
  context_->getSceneManager()->destroySceneNode(planning_scene_node_->getName());
}

void PlanningSceneDisplay::onInitialize()
{  
  Display::onInitialize();
  
  // the scene node that contains everything
  planning_scene_node_ = scene_node_->createChildSceneNode();
  
  planning_scene_robot_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Planning Scene", robot_category_));
  planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
}

void PlanningSceneDisplay::reset()
{ 
  planning_scene_render_.reset();
  planning_scene_robot_->clear();
  
  loadRobotModel();
  Display::reset();
  
  planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
}

const planning_scene_monitor::PlanningSceneMonitorPtr& PlanningSceneDisplay::getPlanningSceneMonitor()
{
  return planning_scene_monitor_;
}

const robot_model::RobotModelConstPtr& PlanningSceneDisplay::getRobotModel()
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->getRobotModel();
  else
  {
    static robot_model::RobotModelConstPtr empty;
    return empty;
  }
}

planning_scene_monitor::LockedPlanningSceneRO PlanningSceneDisplay::getPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

planning_scene_monitor::LockedPlanningSceneRW PlanningSceneDisplay::getPlanningSceneRW()
{
  return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
}

void PlanningSceneDisplay::changedAttachedBodyColor()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedSceneColor()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void PlanningSceneDisplay::changedSceneName()
{
  planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();
  if (ps)
    ps->setName(scene_name_property_->getStdString());
}

void PlanningSceneDisplay::changedRootLinkName()
{
  if (getRobotModel())
    root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
}

void PlanningSceneDisplay::renderPlanningScene()
{
  if (planning_scene_render_ && planning_scene_needs_render_)
  {
    QColor color = scene_color_property_->getColor();
    rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
    color = attached_body_color_property_->getColor();
    rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());
    
    try
    {
      const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
      planning_scene_render_->renderPlanningScene(ps, env_color, attached_color,
                                                  scene_alpha_property_->getFloat());
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while rendering planning scene");
    }
    planning_scene_needs_render_ = false;
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool()); 
  }
}

void PlanningSceneDisplay::changedSceneAlpha()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedRobotSceneAlpha()
{
  planning_scene_robot_->setAlpha(robot_alpha_property_->getFloat());
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedPlanningSceneTopic()
{
  if (planning_scene_monitor_)
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
}

void PlanningSceneDisplay::changedSceneDisplayTime()
{
}

void PlanningSceneDisplay::changedSceneRobotEnabled()
{
  if (isEnabled())
    planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
}

void PlanningSceneDisplay::changedSceneEnabled()
{
  if (planning_scene_render_)
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool()); 
}

void PlanningSceneDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor &color)
{
  if (getRobotModel())
  {
    const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string> &links = jmg->getLinkModelNames();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        setLinkColor(robot, links[i], color);
    }
  }
}

void PlanningSceneDisplay::unsetAllColors(rviz::Robot* robot)
{ 
  if (getRobotModel())
  {
    const std::vector<std::string> &links = getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0 ; i < links.size() ; ++i)
      unsetLinkColor(robot, links[i]);
  }
}

void PlanningSceneDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name )
{
  if (getRobotModel())
  {
    const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string> &links = jmg->getLinkModelNames();
      for (std::size_t i = 0 ; i < links.size() ; ++i)
        unsetLinkColor(robot, links[i]);
    }
  }
}

void PlanningSceneDisplay::setLinkColor(const std::string& link_name, const QColor &color)
{
  setLinkColor(&planning_scene_robot_->getRobot(), link_name, color );
}

void PlanningSceneDisplay::unsetLinkColor(const std::string& link_name)
{
  unsetLinkColor(&planning_scene_robot_->getRobot(), link_name);
}

void PlanningSceneDisplay::setLinkColor(rviz::Robot* robot,  const std::string& link_name, const QColor &color )
{
  rviz::RobotLink *link = robot->getLink(link_name);
  
  // Check if link exists
  if (link)
    link->setColor( color.redF(), color.greenF(), color.blueF() );
}

void PlanningSceneDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name )
{
  rviz::RobotLink *link = robot->getLink(link_name);

  // Check if link exists
  if (link) 
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
void PlanningSceneDisplay::loadRobotModel()
{
  planning_scene_render_.reset();
  planning_scene_monitor_.reset(); // this so that the destructor of the PlanningSceneMonitor gets called before a new instance of a scene monitor is constructed  
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString(),
                                                                                 context_->getFrameManager()->getTFClientPtr(),
                                                                                 getNameStd() + "_planning_scene_monitor"));
  if (planning_scene_monitor_->getPlanningScene() && planning_scene_monitor_->getPlanningScene()->isConfigured())
  {
    planning_scene_monitor_->addUpdateCallback(boost::bind(&PlanningSceneDisplay::sceneMonitorReceivedUpdate, this, _1));
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    planning_scene_render_.reset(new PlanningSceneRender(planning_scene_node_, context_, planning_scene_robot_));
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool()); 
    onRobotModelLoaded();
    setStatus( rviz::StatusProperty::Ok, "PlanningScene", "Planning Scene Loaded Successfully" );
  }
  else
  {
    planning_scene_monitor_.reset();
    setStatus( rviz::StatusProperty::Error, "PlanningScene", "No Planning Scene Loaded" );
  }
}

void PlanningSceneDisplay::onRobotModelLoaded()
{
  planning_scene_robot_->load(*getRobotModel()->getURDF());
  const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
  planning_scene_robot_->update(robot_state::RobotStatePtr(new robot_state::RobotState(ps->getCurrentState())));

  bool oldState = scene_name_property_->blockSignals(true);
  scene_name_property_->setStdString(ps->getName());
  scene_name_property_->blockSignals(oldState);

  oldState = root_link_name_property_->blockSignals(true);
  root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
  root_link_name_property_->blockSignals(oldState);  
}

void PlanningSceneDisplay::sceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  onSceneMonitorReceivedUpdate(update_type);
}

void PlanningSceneDisplay::onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  bool oldState = scene_name_property_->blockSignals(true);
  scene_name_property_->setStdString(getPlanningSceneRO()->getName());
  scene_name_property_->blockSignals(oldState);

  oldState = root_link_name_property_->blockSignals(true);
  root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
  root_link_name_property_->blockSignals(oldState);
  
  planning_scene_needs_render_ = true;
}

void PlanningSceneDisplay::onEnable()
{
  Display::onEnable();
  
  loadRobotModel();
  
  planning_scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
  if (planning_scene_render_)
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool()); 

  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void PlanningSceneDisplay::onDisable()
{
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->stopSceneMonitor();
    if (planning_scene_render_)
      planning_scene_render_->getGeometryNode()->setVisible(false);
  }
  planning_scene_robot_->setVisible(false);
  Display::onDisable();
}

void PlanningSceneDisplay::queueRenderSceneGeometry()
{
  planning_scene_needs_render_ = true;
}

void PlanningSceneDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  
  if (!planning_scene_monitor_)
    return;
  
  current_scene_time_ += wall_dt;
  if (current_scene_time_ > scene_display_time_property_->getFloat())
  {
    renderPlanningScene();
    current_scene_time_ = 0.0f;
  }
}

void PlanningSceneDisplay::load( const rviz::Config& config )
{
  Display::load(config);
}

void PlanningSceneDisplay::save( rviz::Config config ) const
{
  Display::save(config);
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void PlanningSceneDisplay::calculateOffsetPosition()
{  
  if (!getRobotModel())
    return;

  ros::Time stamp;
  std::string err_string;
  if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), getRobotModel()->getModelFrame(), stamp, &err_string) != tf::NO_ERROR)
    return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, getRobotModel()->getModelFrame());

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), getRobotModel()->getModelFrame(), stamp))
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

void PlanningSceneDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged(); 
  calculateOffsetPosition();  
}


} // namespace moveit_rviz_plugin
