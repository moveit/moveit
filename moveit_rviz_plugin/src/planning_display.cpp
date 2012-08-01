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
#include <rviz/robot/link_updater.h>
#include <rviz/properties/property.h>
#include "rviz/properties/string_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/ros_topic_property.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>
#include <planning_models/conversions.h>
#include <trajectory_processing/trajectory_tools.h>

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// ******************************************************************************************
// Sub Class
// ******************************************************************************************
// ******************************************************************************************
class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:

  // ******************************************************************************************
  // Sub class constructor
  // ******************************************************************************************
  PlanningLinkUpdater(const planning_models::KinematicState* state)
    : kinematic_state_(state)
  {
  }

  // ******************************************************************************************
  //
  // ******************************************************************************************
  virtual bool getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                 Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation, bool& apply_offset_transforms) const
  {
    const planning_models::KinematicState::LinkState* link_state = kinematic_state_->getLinkState( link_name );

    if ( !link_state )
    {
      return false;
    }

    const Eigen::Vector3d &robot_visual_position = link_state->getGlobalLinkTransform().translation();
    Eigen::Quaterniond robot_visual_orientation(link_state->getGlobalLinkTransform().rotation());
    visual_position = Ogre::Vector3( robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z() );
    visual_orientation = Ogre::Quaternion( robot_visual_orientation.w(), robot_visual_orientation.x(), robot_visual_orientation.y(), robot_visual_orientation.z() );

    const Eigen::Vector3d &robot_collision_position = link_state->getGlobalCollisionBodyTransform().translation();
    Eigen::Quaterniond robot_collision_orientation(link_state->getGlobalCollisionBodyTransform().rotation());
    collision_position = Ogre::Vector3( robot_collision_position.x(), robot_collision_position.y(), robot_collision_position.z() );
    collision_orientation = Ogre::Quaternion( robot_collision_orientation.w(), robot_collision_orientation.x(), robot_collision_orientation.y(), robot_collision_orientation.z() );

    return true;
  }


  // ******************************************************************************************
  // Private
  // ******************************************************************************************
private:
  const planning_models::KinematicState* kinematic_state_;

};


// ******************************************************************************************
// ******************************************************************************************
// Base Class
// ******************************************************************************************
// ******************************************************************************************


// ******************************************************************************************
// Structs
// ******************************************************************************************
struct PlanningDisplay::ReceivedTrajectoryMessage
{
  ReceivedTrajectoryMessage(const moveit_msgs::DisplayTrajectory::ConstPtr &message, const planning_scene::PlanningScenePtr &scene) : message_(message)
  {
    start_state_.reset(new planning_models::KinematicState(scene->getCurrentState()));
    planning_models::robotStateToKinematicState(*scene->getTransforms(), message_->trajectory_start, *start_state_);
    trajectory_processing::convertToKinematicStates(trajectory_, message_->trajectory_start, message_->trajectory, scene->getCurrentState(), scene->getTransforms());
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
  new_display_trajectory_(false), animating_path_(false), current_scene_time_(0.0f)
{
  last_scene_render_ = ros::Time::now();


  // Category Groups
  scene_category_ = new rviz::Property( "Planning Scene", QVariant(), "", this );
  path_category_  = new rviz::Property( "Planned Path",   QVariant(), "", this );

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

  visual_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Visual", "", "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                            path_category_,
                            SLOT( changedVisualEnabled() ), this );

  collision_enabled_property_ =
    new rviz::BoolProperty( "Show Robot Collision", "", "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
                            path_category_,
                            SLOT( changedCollisionEnabled() ), this );

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

  /*robot_->setPropertyManager(property_manager_, path_category_);
    scene_robot_->setPropertyManager(property_manager_, scene_category_);
  */

}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();
  //  delete robot_;
  //  delete scene_robot_;
}

// ******************************************************************************************
//
// ******************************************************************************************
void PlanningDisplay::onInitialize()
{
  robot_ = new rviz::Robot(context_, "Motion Plan", path_category_ );
  scene_robot_ = new rviz::Robot(context_, "Planning Scene", scene_category_ );
  scene_robot_->setCollisionVisible(false);
  scene_robot_->setVisualVisible(true);
  scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible(scene_enabled_property_->getBool());
}

// ******************************************************************************************
//
// ******************************************************************************************
void PlanningDisplay::reset()
{
  if (scene_monitor_)
    scene_monitor_->getPlanningScene()->getCollisionWorld()->clearObjects();
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
}

const std::string PlanningDisplay::getRobotDescription(void)
{
  return robot_description_property_->getStdString();
}

void PlanningDisplay::changedRobotDescription()
{
  if (isEnabled())
  {
    load();

    context_->queueRender();
  }
}

// ******************************************************************************************
// Scene Name
// ******************************************************************************************
void PlanningDisplay::setSceneName(const std::string &name)
{
  scene_name_property_->setStdString( name );
}

const std::string PlanningDisplay::getSceneName(void)
{
  return scene_name_property_->getStdString();
}

void PlanningDisplay::changedSceneName(){}

// ******************************************************************************************
// Root Link Name
// ******************************************************************************************
void PlanningDisplay::setRootLinkName(const std::string &name)
{
  root_link_name_property_->setStdString( name );
}

const std::string PlanningDisplay::getRootLinkName(void)
{
  return root_link_name_property_->getStdString();
}

void PlanningDisplay::changedRootLinkName(){}

// ******************************************************************************************
// Loop Display
// ******************************************************************************************
void PlanningDisplay::setLoopDisplay(bool loop_display)
{
  loop_display_property_->setValue( loop_display );
}

bool PlanningDisplay::getLoopDisplay()
{
  return loop_display_property_->getBool();
}

void PlanningDisplay::changedLoopDisplay(){}

// ******************************************************************************************
// Robot Path Alpha
// ******************************************************************************************
void PlanningDisplay::setRobotPathAlpha(float alpha)
{
  robot_path_alpha_property_->setValue( alpha );
  robot_->setAlpha(robot_path_alpha_property_->getFloat());
  context_->queueRender();
}

float PlanningDisplay::getRobotPathAlpha()
{
  return robot_path_alpha_property_->getFloat();
}

void PlanningDisplay::changedRobotPathAlpha()
{
  robot_->setAlpha(robot_path_alpha_property_->getFloat());
  context_->queueRender();
}

// ******************************************************************************************
// Scene Alpha
// ******************************************************************************************
void PlanningDisplay::setSceneAlpha(float alpha)
{
  scene_alpha_property_->setFloat( alpha );
}

float PlanningDisplay::getSceneAlpha()
{
  return scene_alpha_property_->getFloat();
}

void PlanningDisplay::changedSceneAlpha()
{
  renderPlanningScene();
  context_->queueRender();
}

// ******************************************************************************************
// Scene Robot Alpha
// ******************************************************************************************
void PlanningDisplay::setSceneRobotAlpha(float alpha)
{
  robot_scene_alpha_property_->setFloat( alpha );
}

float PlanningDisplay::getSceneRobotAlpha()
{
  return robot_scene_alpha_property_->getFloat();
}

void PlanningDisplay::changedRobotSceneAlpha()
{
  scene_robot_->setAlpha(robot_scene_alpha_property_->getFloat());
  context_->queueRender();
}

// ******************************************************************************************
// Trajectory Topic
// ******************************************************************************************
void PlanningDisplay::setTrajectoryTopic(const std::string& topic)
{
  unsubscribe();
  trajectory_topic_property_->setStdString( topic );
  subscribe();
}

const std::string PlanningDisplay::getTrajectoryTopic()
{
  return trajectory_topic_property_->getMessageType().toStdString();
}

void PlanningDisplay::changedTrajectoryTopic()
{
  unsubscribe();
  // TODO: is this ok?
  subscribe();
}

// ******************************************************************************************
// Planning Scene Topic
// ******************************************************************************************
void PlanningDisplay::setPlanningSceneTopic(const std::string &topic)
{
  planning_scene_topic_property_->setStdString( topic );
}

const std::string PlanningDisplay::getPlanningSceneTopic(void) {
  return planning_scene_topic_property_->getStdString();
}

void PlanningDisplay::changedPlanningSceneTopic()
{
  if (scene_monitor_)
    scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
}

// ******************************************************************************************
// Scene Display Time
// ******************************************************************************************
void PlanningDisplay::setSceneDisplayTime(float time)
{
  scene_display_time_property_->setFloat( time );
}

float PlanningDisplay::getSceneDisplayTime()
{
  return scene_display_time_property_->getFloat();
}

void PlanningDisplay::changedSceneDisplayTime(){}

// ******************************************************************************************
// State Display Time
// ******************************************************************************************
void PlanningDisplay::setStateDisplayTime(float time)
{
  state_display_time_property_->setFloat( time );
}

float PlanningDisplay::getStateDisplayTime()
{
  return state_display_time_property_->getFloat();
}

void PlanningDisplay::changedStateDisplayTime(){}

// ******************************************************************************************
// Scene Robot Visible
// ******************************************************************************************
void PlanningDisplay::setSceneRobotVisible(bool visible)
{
  scene_robot_enabled_property_->setValue( visible );
}

bool PlanningDisplay::getSceneRobotVisible()
{
  return scene_robot_enabled_property_->getBool();
}

// TODO: is this right?
void PlanningDisplay::changedSceneRobotEnabled()
{
  scene_robot_->setVisible( scene_robot_enabled_property_->getBool() );
  context_->queueRender();
}

// ******************************************************************************************
// Scene Visible
// ******************************************************************************************
void PlanningDisplay::setSceneVisible(bool visible)
{
  scene_enabled_property_->setValue( visible );
}

bool PlanningDisplay::getSceneVisible()
{
  return scene_enabled_property_->getBool();
}

void PlanningDisplay::changedSceneEnabled()
{
  scene_node_->setVisible( scene_enabled_property_->getBool() );
  context_->queueRender();
}

// ******************************************************************************************
// Visual Visible
// ******************************************************************************************
void PlanningDisplay::setVisualVisible(bool visible)
{
  robot_->setVisualVisible(visible);
  context_->queueRender();
}

bool PlanningDisplay::getVisualVisible()
{
  return robot_->isVisualVisible();
}

void PlanningDisplay::changedVisualEnabled(){} // TODO ??

// ******************************************************************************************
// Collision Visible
// ******************************************************************************************
void PlanningDisplay::setCollisionVisible(bool visible)
{
  robot_->setCollisionVisible(visible);
  context_->queueRender();
}

bool PlanningDisplay::getCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void PlanningDisplay::changedCollisionEnabled(){} // TODO?

// ******************************************************************************************
// Link Color
// ******************************************************************************************
void PlanningDisplay::setLinkColor( const std::string& link_name, float red, float green, float blue )
{
  rviz::RobotLink* link = robot_->getLink( link_name );

  // Check if link exists
  if( link )
  {
    link->setColor( red, green, blue );
    context_->queueRender();
  }

  // Repeat for scene_robot -------------------------------------
  link = scene_robot_->getLink( link_name );

  // Check if link exists
  if( link )
  {
    link->setColor( red, green, blue );
    context_->queueRender();
  }

}

void PlanningDisplay::unsetLinkColor( const std::string& link_name )
{
  rviz::RobotLink* link = robot_->getLink( link_name );

  // Check if link exists
  if( link )
  {
    link->unsetColor();
    context_->queueRender();
  }

  // Repeat for scene_robot -------------------------------------
  link = scene_robot_->getLink( link_name );

  // Check if link exists
  if( link )
  {
    link->unsetColor();
    context_->queueRender();
  }

}

// ******************************************************************************************
// Load
// ******************************************************************************************
void PlanningDisplay::load()
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
  robot_->load(doc.RootElement(), descr);
  scene_robot_->load(doc.RootElement(), descr);

  scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString()));

  if (scene_monitor_->getPlanningScene())
  {
    scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    scene_robot_->update(PlanningLinkUpdater(&scene_monitor_->getPlanningScene()->getCurrentState()));
  }
  else
    scene_monitor_.reset();
}

// ******************************************************************************************
// Enable
// ******************************************************************************************
void PlanningDisplay::onEnable()
{
  subscribe();
  load();
  robot_->setVisible(true);
  scene_node_->setVisible(scene_enabled_property_->getBool());
  scene_robot_->setVisible(scene_robot_enabled_property_->getBool());
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void PlanningDisplay::onDisable()
{
  unsubscribe();
  if (scene_monitor_)
    scene_monitor_->stopSceneMonitor();
  robot_->setVisible(false);
  scene_node_->setVisible(false);
  scene_robot_->setVisible(false);
}

// ******************************************************************************************
// Subscribe
// ******************************************************************************************
void PlanningDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!trajectory_topic_property_->getStdString().empty())
  {
    sub_ = update_nh_.subscribe(trajectory_topic_property_->getStdString(), 2, &PlanningDisplay::incomingDisplayTrajectory, this);
  }
}

// ******************************************************************************************
// Unsubscribe
// ******************************************************************************************
void PlanningDisplay::unsubscribe()
{
  sub_.shutdown();
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
    scene_robot_->update(PlanningLinkUpdater(&scene_monitor_->getPlanningScene()->getCurrentState()));
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
      const std::vector<Eigen::Affine3d> &ab_t = attached_bodies[i]->getGlobalCollisionBodyTransforms();
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

  bool render = false;
  bool offset = false;

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
    PlanningLinkUpdater plu(displaying_trajectory_message_->start_state_.get());
    robot_->update(plu);

    offset = true;
    render = true;
  }

  if (animating_path_)
  {
    if (current_state_time_ > state_display_time_property_->getFloat())
    {
      ++current_state_;
      if ((std::size_t) current_state_ < displaying_trajectory_message_->trajectory_.size())
      {
        PlanningLinkUpdater plu(displaying_trajectory_message_->trajectory_[current_state_].get());
        robot_->update(plu);
        render = true;
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
    render = true;
    offset = true;
    setSceneName(scene_monitor_->getPlanningScene()->getName());
    setRootLinkName(scene_monitor_->getPlanningScene()->getKinematicModel()->getRootLink()->getName());
    renderPlanningScene();
    current_scene_time_ = 0.0f;
  }

  if (offset)
    calculateOffsetPosition();
  if (render)
    context_->queueRender();
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void PlanningDisplay::calculateOffsetPosition()
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
  robot_->setPosition(position);
  robot_->setOrientation(orientation);
  scene_robot_->setPosition(position);
  scene_robot_->setOrientation(orientation);
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
