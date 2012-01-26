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

#include "planning_display.h"
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/link_updater.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>
#include <planning_models/conversions.h>

namespace motion_planning_rviz_plugin
{

class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:
  PlanningLinkUpdater(const planning_models::KinematicState* state)
    : kinematic_state_(state)
  {}

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

private:
  const planning_models::KinematicState* kinematic_state_;
};

struct PlanningDisplay::ReceivedTrajectoryMessage
{
  ReceivedTrajectoryMessage(const moveit_msgs::DisplayTrajectory::ConstPtr &message, const planning_scene::PlanningScenePtr &scene) : message_(message)
  {
    start_state_.reset(new planning_models::KinematicState(scene->getCurrentState()));
    planning_models::robotStateToKinematicState(*scene->getTransforms(), message_->robot_state, *start_state_);

    std::size_t state_count = std::max(message_->trajectory.joint_trajectory.points.size(),
                                       message_->trajectory.multi_dof_joint_trajectory.points.size());
    for (std::size_t i = 0 ; i < state_count ; ++i)
    {
      moveit_msgs::RobotState rs;
      planning_models::robotTrajectoryPointToRobotState(message_->trajectory, i, rs);
      planning_models::KinematicStatePtr state(new planning_models::KinematicState(*start_state_));
      planning_models::robotStateToKinematicState(*scene->getTransforms(), rs, *state);
      trajectory_.push_back(state);
    }
  }

  moveit_msgs::DisplayTrajectory::ConstPtr message_;
  planning_models::KinematicStatePtr start_state_;
  std::vector<planning_models::KinematicStatePtr> trajectory_;
};

PlanningDisplay::PlanningDisplay() :
    Display(), robot_path_alpha_(0.75f), robot_scene_alpha_(0.5f), scene_alpha_(0.9f), loop_display_(false),
    display_scene_(true), display_scene_robot_(true), state_display_time_(0.05f), scene_display_time_(0.2f),
    new_display_trajectory_(false), animating_path_(false), current_scene_time_(0.0f)
{
    last_scene_render_ = ros::Time::now();
}

PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();
  delete robot_;
  delete scene_robot_;
}

void PlanningDisplay::onInitialize()
{
  robot_ = new rviz::Robot(vis_manager_, "Motion Plan " + name_);
  scene_robot_ = new rviz::Robot(vis_manager_, "Planning Scene " + name_);
  scene_robot_->setCollisionVisible(false);
  scene_robot_->setVisualVisible(true);
  scene_robot_->setVisible(display_scene_robot_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible(display_scene_);
}

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

void PlanningDisplay::clearRenderedGeometry()
{
  scene_shapes_.clear();
  for (std::size_t i = 0 ; i < manual_objects_.size() ; ++i)
    vis_manager_->getSceneManager()->destroyManualObject(manual_objects_[i]);
  manual_objects_.clear();
  if (!material_name_.empty())
  {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_name_ = "";
  }
}

void PlanningDisplay::setSceneName(const std::string &name)
{
  scene_name_ = name;
  propertyChanged(scene_name_property_);
}

void PlanningDisplay::setRobotDescription(const std::string& description_param)
{
  description_param_ = description_param;
  propertyChanged(robot_description_property_);
  if (isEnabled())
  {
    load();
    causeRender();
  }
}

void  PlanningDisplay::setLoopDisplay(bool loop_display)
{
  loop_display_ = loop_display;
  propertyChanged(loop_display_property_);
}

void PlanningDisplay::setRobotAlpha(float alpha)
{
  robot_path_alpha_ = alpha;
  robot_->setAlpha(robot_path_alpha_);
  propertyChanged(robot_path_alpha_property_);
  causeRender();
}

void PlanningDisplay::setSceneAlpha(float alpha)
{
  scene_alpha_ = alpha;
  propertyChanged(scene_alpha_property_);
  renderPlanningScene();
  causeRender();
}

void PlanningDisplay::setSceneRobotAlpha(float alpha)
{
  robot_scene_alpha_ = alpha;
  scene_robot_->setAlpha(robot_scene_alpha_);
  propertyChanged(robot_scene_alpha_property_);
  causeRender();
}

void PlanningDisplay::setTrajectoryTopic(const std::string& topic)
{
  unsubscribe();
  display_trajectory_topic_ = topic;
  subscribe();
  propertyChanged(trajectory_topic_property_);
}

void PlanningDisplay::setPlanningSceneTopic(const std::string &topic)
{
  planning_scene_topic_ = topic;
  if (scene_monitor_)
      scene_monitor_->startSceneMonitor(planning_scene_topic_, planning_scene_diff_topic_);
  propertyChanged(planning_scene_topic_property_);
}

void PlanningDisplay::setPlanningSceneDiffTopic(const std::string &topic)
{
  planning_scene_diff_topic_ = topic;
  if (scene_monitor_)
      scene_monitor_->startSceneMonitor(planning_scene_topic_, planning_scene_diff_topic_);
  propertyChanged(planning_scene_diff_topic_property_);
}

void PlanningDisplay::setSceneDisplayTime(float time)
{
  scene_display_time_ = time;
  propertyChanged(scene_display_time_property_);
}

void PlanningDisplay::setStateDisplayTime(float time)
{
  state_display_time_ = time;
  propertyChanged(state_display_time_property_);
}

void PlanningDisplay::setSceneRobotVisible(bool visible)
{
    display_scene_robot_ = visible;
    scene_robot_->setVisible(visible);
    propertyChanged(scene_robot_enabled_property_);
    causeRender();
}

void PlanningDisplay::setSceneVisible(bool visible)
{
  display_scene_ = visible;
  scene_node_->setVisible(visible);
  propertyChanged(scene_enabled_property_);
  causeRender();
}

void PlanningDisplay::setVisualVisible(bool visible)
{
  robot_->setVisualVisible(visible);
  propertyChanged(visual_enabled_property_);
  causeRender();
}

void PlanningDisplay::setCollisionVisible(bool visible)
{
  robot_->setCollisionVisible(visible);
  propertyChanged(collision_enabled_property_);
  causeRender();
}

bool PlanningDisplay::getSceneVisible()
{
  return display_scene_;
}

bool PlanningDisplay::getSceneRobotVisible()
{
  return display_scene_robot_;
}


bool PlanningDisplay::getVisualVisible()
{
  return robot_->isVisualVisible();
}

bool PlanningDisplay::getCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void PlanningDisplay::load()
{
  std::string content;
  if (!update_nh_.getParam(description_param_, content))
  {
    std::string loc;
    if (update_nh_.searchParam(description_param_, loc))
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

  scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(description_param_, vis_manager_->getTFClient()));

  if (scene_monitor_->getPlanningScene())
  {
    scene_monitor_->startSceneMonitor(planning_scene_topic_, planning_scene_diff_topic_);
    scene_robot_->update(PlanningLinkUpdater(&scene_monitor_->getPlanningScene()->getCurrentState()));
  }
  else
    scene_monitor_.reset();
}

void PlanningDisplay::onEnable()
{
  subscribe();
  load();
  robot_->setVisible(true);
  scene_node_->setVisible(display_scene_);
  scene_robot_->setVisible(display_scene_robot_);
}

void PlanningDisplay::onDisable()
{
  unsubscribe();
  if (scene_monitor_)
      scene_monitor_->stopSceneMonitor();
  robot_->setVisible(false);
  scene_node_->setVisible(false);
  scene_robot_->setVisible(false);
}

void PlanningDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!display_trajectory_topic_.empty())
  {
    sub_ = update_nh_.subscribe(display_trajectory_topic_, 2, &PlanningDisplay::incomingDisplayTrajectory, this);
  }
}

void PlanningDisplay::unsubscribe()
{
  sub_.shutdown();
}

void PlanningDisplay::renderShape(Ogre::SceneNode *node, const shapes::Shape *s, const Eigen::Affine3d &p, const rviz::Color &color, float alpha)
{
    rviz::Shape* ogre_shape = NULL;
    switch (s->type)
    {
    case shapes::SPHERE:
        {
            ogre_shape = new rviz::Shape(rviz::Shape::Sphere,
					 vis_manager_->getSceneManager(), node);
            double d = 2.0 * static_cast<const shapes::Sphere*>(s)->radius;
            ogre_shape->setScale(Ogre::Vector3(d, d, d));
        }
        break;
    case shapes::BOX:
        {
            ogre_shape = new rviz::Shape(rviz::Shape::Cube,
					 vis_manager_->getSceneManager(), node);
            const double* sz = static_cast<const shapes::Box*>(s)->size;
            ogre_shape->setScale(Ogre::Vector3(sz[0], sz[1], sz[2]));
        }
        break;
    case shapes::CYLINDER:
        {
            ogre_shape = new rviz::Shape(rviz::Shape::Cylinder,
					 vis_manager_->getSceneManager(), node);
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
                Ogre::ManualObject *manual_object = vis_manager_->getSceneManager()->createManualObject(name);
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

void PlanningDisplay::renderPlanningScene()
{
    static rviz::Color colors[] = {
        rviz::Color(0.2f, 0.9f, 0.2f),
        rviz::Color(0.0f, 0.1f, 0.9f),
        rviz::Color(0.0f, 0.9f, 0.9f),
        rviz::Color(0.9f, 0.9f, 0.0f),
        rviz::Color(0.9f, 0.0f, 0.2f)
    };
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
            const rviz::Color &color = colors[i % (sizeof(colors)/sizeof(rviz::Color))];
            for (std::size_t j = 0 ; j < o->shapes_.size() ; ++j)
                renderShape(scene_node_, o->shapes_[j], o->shape_poses_[j], color, scene_alpha_);
        }

        std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
        scene_monitor_->getPlanningScene()->getCurrentState().getAttachedBodies(attached_bodies);
        for (std::size_t i = 0 ; i < attached_bodies.size() ; ++i)
        {
            const std::vector<Eigen::Affine3d> &ab_t = attached_bodies[i]->getGlobalCollisionBodyTransforms();
            const std::vector<shapes::Shape*> &ab_shapes = attached_bodies[i]->getShapes();
            for (std::size_t j = 0 ; j < ab_shapes.size() ; ++j)
            {
                renderShape(scene_robot_->getVisualNode(), ab_shapes[j], ab_t[j], attached_color, robot_scene_alpha_);
                renderShape(scene_robot_->getCollisionNode(), ab_shapes[j], ab_t[j], attached_color, robot_scene_alpha_);
            }
        }
    }
    catch(...)
    {
        scene_monitor_->unlockScene();
        throw;
    }
    scene_monitor_->unlockScene();

    scene_node_->setVisible(display_scene_);
}

void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  if (!scene_monitor_)
    return;

  bool render = false;
  bool offset = false;

  if (!animating_path_ && !new_display_trajectory_ && loop_display_ && displaying_trajectory_message_)
  {
      animating_path_ = true;
      current_state_ = -1;
      current_state_time_ = state_display_time_ + 1.0f;
  }

  if (!animating_path_ && new_display_trajectory_)
  {
      scene_monitor_->updateFrameTransforms();
      displaying_trajectory_message_.reset(new ReceivedTrajectoryMessage(incoming_trajectory_message_, scene_monitor_->getPlanningScene()));
      animating_path_ = true;
      new_display_trajectory_ = false;
      current_state_ = -1;
      current_state_time_ = state_display_time_ + 1.0f;
      robot_->update(PlanningLinkUpdater(displaying_trajectory_message_->start_state_.get()));
      offset = true;
      render = true;
  }

  if (animating_path_)
  {
    if (current_state_time_ > state_display_time_)
    {
      ++current_state_;
      if ((std::size_t) current_state_ < displaying_trajectory_message_->trajectory_.size())
      {
        robot_->update(PlanningLinkUpdater(displaying_trajectory_message_->trajectory_[current_state_].get()));
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
  if (current_scene_time_ > scene_display_time_)
  {
    render = true;
    offset = true;
    setSceneName(scene_monitor_->getPlanningScene()->getName());
    renderPlanningScene();
    current_scene_time_ = 0.0f;
  }

  if (offset)
    calculateOffsetPosition();
  if (render)
    causeRender();
}

void PlanningDisplay::calculateOffsetPosition()
{
  if (!scene_monitor_)
      return;
  ros::Time stamp;
  std::string err_string;
  if (vis_manager_->getTFClient()->getLatestCommonTime(fixed_frame_, scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp, &err_string) != tf::NO_ERROR)
      return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, scene_monitor_->getPlanningScene()->getPlanningFrame());

  if (vis_manager_->getTFClient()->canTransform(fixed_frame_, scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp))
  {
    try
    {
      vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.c_str() );
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

void PlanningDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  incoming_trajectory_message_ = msg;
  if (scene_monitor_)
    if (msg->model_id != scene_monitor_->getPlanningScene()->getKinematicModel()->getName())
      ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
               msg->model_id.c_str(), scene_monitor_->getPlanningScene()->getKinematicModel()->getName().c_str());
  new_display_trajectory_ = true;
}

void PlanningDisplay::fixedFrameChanged()
{
  calculateOffsetPosition();
}

void PlanningDisplay::createProperties()
{

  /// generic properties
  robot_description_property_ = property_manager_->createProperty<rviz::StringProperty> ("Robot Description", property_prefix_,
                                                                                         boost::bind(&PlanningDisplay::getRobotDescription, this),
                                                                                         boost::bind(&PlanningDisplay::setRobotDescription, this, _1), parent_category_,
                                                                                         this);
  setPropertyHelpText(robot_description_property_, "The name of the ROS parameter where the URDF for the robot is loaded");


  scene_category_ = property_manager_->createCategory("Planning Scene", property_prefix_, parent_category_);
  path_category_ = property_manager_->createCategory("Planned Path", property_prefix_, parent_category_);
  const std::string scene_prefix = property_prefix_ + "_scene";
  const std::string path_prefix = property_prefix_ + "_path";

  ///// planning scene

  scene_name_property_ = property_manager_->createProperty<rviz::StringProperty> ("Scene Name", scene_prefix,
                                                                                  boost::bind(&PlanningDisplay::getSceneName, this),
                                                                                  boost::bind(&PlanningDisplay::setSceneName, this, _1), scene_category_, this);
  setPropertyHelpText(scene_name_property_, "Shows the name of the planning scene");

  scene_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Show Scene Geometry", scene_prefix,
                                                                                   boost::bind(&PlanningDisplay::getSceneVisible, this),
                                                                                   boost::bind(&PlanningDisplay::setSceneVisible, this, _1), scene_category_, this);
  setPropertyHelpText(scene_enabled_property_, "Indicates whether planning scenes should be displayed");
  scene_robot_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Show Scene Robot", scene_prefix,
                                                                                         boost::bind(&PlanningDisplay::getSceneRobotVisible, this),
                                                                                         boost::bind(&PlanningDisplay::setSceneRobotVisible, this, _1), scene_category_, this);
  setPropertyHelpText(scene_robot_enabled_property_, "Indicates whether the robot state specified by the planning scene should be displayed");

  robot_scene_alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Robot Alpha", scene_prefix, boost::bind(&PlanningDisplay::getSceneRobotAlpha, this),
                                                                                        boost::bind(&PlanningDisplay::setSceneRobotAlpha, this, _1), scene_category_, this);
  setPropertyHelpText(robot_scene_alpha_property_, "Specifies the alpha for the robot links");
  rviz::FloatPropertyPtr float_prop = robot_scene_alpha_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->setMax( 1.0 );

  scene_alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Scene Alpha", scene_prefix, boost::bind(&PlanningDisplay::getSceneAlpha, this),
                                                                                  boost::bind(&PlanningDisplay::setSceneAlpha, this, _1), scene_category_, this);
  setPropertyHelpText(scene_alpha_property_, "Specifies the alpha for the robot links");
  float_prop = scene_alpha_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->setMax( 1.0 );

  scene_display_time_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Scene Display Time", path_prefix,
                                                                                         boost::bind(&PlanningDisplay::getSceneDisplayTime, this),
                                                                                         boost::bind(&PlanningDisplay::setSceneDisplayTime, this, _1),
                                                                                         scene_category_, this);
  setPropertyHelpText(scene_display_time_property_, "The amount of wall-time to wait in between rendering updates to the planning scene (if any)");
  float_prop = scene_display_time_property_.lock();
  float_prop->setMin(0.0001);


  planning_scene_topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Planning Scene Topic", path_prefix,
                                                                                                    boost::bind(&PlanningDisplay::getPlanningSceneTopic, this),
                                                                                                    boost::bind(&PlanningDisplay::setPlanningSceneTopic, this, _1),
                                                                                                    scene_category_, this);
  setPropertyHelpText(planning_scene_topic_property_, "The topic on which the moveit_msgs::PlanningScene messages are received");
  rviz::ROSTopicStringPropertyPtr topic_prop = planning_scene_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<moveit_msgs::PlanningScene>());

  planning_scene_diff_topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Planning Scene Diffs Topic", path_prefix,
                                                                                                         boost::bind(&PlanningDisplay::getPlanningSceneDiffTopic, this),
                                                                                                         boost::bind(&PlanningDisplay::setPlanningSceneDiffTopic, this, _1),
                                                                                                         scene_category_, this);
  setPropertyHelpText(planning_scene_diff_topic_property_, "The topic on which the moveit_msgs::PlanningScene diff messages are received");
  topic_prop = planning_scene_diff_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<moveit_msgs::PlanningScene>());


  ///// robot path

  visual_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Show Robot Visual", path_prefix,
                                                                                    boost::bind(&PlanningDisplay::getVisualVisible, this),
                                                                                    boost::bind(&PlanningDisplay::setVisualVisible, this, _1), path_category_, this);
  setPropertyHelpText(visual_enabled_property_, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed");

  collision_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Show Robot Collision", path_prefix,
                                                                                       boost::bind(&PlanningDisplay::getCollisionVisible, this),
                                                                                       boost::bind(&PlanningDisplay::setCollisionVisible, this, _1), path_category_, this);
  setPropertyHelpText(collision_enabled_property_, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed");

  robot_path_alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Robot Alpha", path_prefix, boost::bind(&PlanningDisplay::getRobotAlpha, this),
                                                                                       boost::bind(&PlanningDisplay::setRobotAlpha, this, _1), path_category_, this);
  setPropertyHelpText(robot_path_alpha_property_, "Specifies the alpha for the robot links");
  float_prop = robot_path_alpha_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->setMax( 1.0 );

  state_display_time_property_ = property_manager_->createProperty<rviz::FloatProperty> ("State Display Time", path_prefix,
                                                                                         boost::bind(&PlanningDisplay::getStateDisplayTime, this),
                                                                                         boost::bind(&PlanningDisplay::setStateDisplayTime, this, _1),
                                                                                         path_category_, this);
  setPropertyHelpText(state_display_time_property_, "The amount of wall-time to wait in between displaying states along a received trajectory path");
  float_prop = state_display_time_property_.lock();
  float_prop->setMin(0.0001);

  loop_display_property_ = property_manager_->createProperty<rviz::BoolProperty>("Loop Animation", path_prefix, boost::bind(&PlanningDisplay::getLoopDisplay, this),
                                                                                 boost::bind(&PlanningDisplay::setLoopDisplay, this, _1), path_category_, this);
  setPropertyHelpText(loop_display_property_, "Indicates whether the last received path is to be animated in a loop");

  trajectory_topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Trajectory Topic", path_prefix,
                                                                                                boost::bind(&PlanningDisplay::getTrajectoryTopic, this),
                                                                                                boost::bind(&PlanningDisplay::setTrajectoryTopic, this, _1),
                                                                                                path_category_, this);
  setPropertyHelpText(trajectory_topic_property_, "The topic on which the moveit_msgs::DisplayTrajectory messages are received");
  topic_prop = trajectory_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>());

  robot_->setPropertyManager(property_manager_, path_category_);
  scene_robot_->setPropertyManager(property_manager_, scene_category_);
}

} // namespace motion_planning_rviz_plugin
