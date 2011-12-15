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
#include "rviz/visualization_manager.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/link_updater.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/shape.h>

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

    const btVector3 &robot_visual_position = link_state->getGlobalLinkTransform().getOrigin();
    const btQuaternion &robot_visual_orientation = link_state->getGlobalLinkTransform().getRotation();
    visual_position = Ogre::Vector3( robot_visual_position.getX(), robot_visual_position.getY(), robot_visual_position.getZ() );
    visual_orientation = Ogre::Quaternion( robot_visual_orientation.getW(), robot_visual_orientation.getX(), robot_visual_orientation.getY(), robot_visual_orientation.getZ() );

    const btVector3 &robot_collision_position = link_state->getGlobalCollisionBodyTransform().getOrigin();
    const btQuaternion &robot_collision_orientation = link_state->getGlobalCollisionBodyTransform().getRotation();
    collision_position = Ogre::Vector3( robot_collision_position.getX(), robot_collision_position.getY(), robot_collision_position.getZ() );
    collision_orientation = Ogre::Quaternion( robot_collision_orientation.getW(), robot_collision_orientation.getX(), robot_collision_orientation.getY(), robot_collision_orientation.getZ() );

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
    scene_shapes_.clear();
    incoming_trajectory_message_.reset();
    displaying_trajectory_message_.reset();
    animating_path_ = false;
    new_display_trajectory_ = false;
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
}

void PlanningDisplay::setSceneAlpha(float alpha)
{
  scene_alpha_ = alpha;
  renderPlanningScene();
  propertyChanged(scene_alpha_property_);
  causeRender();
}
    
void PlanningDisplay::setSceneRobotAlpha(float alpha)
{
  robot_scene_alpha_ = alpha;
  scene_robot_->setAlpha(robot_path_alpha_);
  propertyChanged(robot_scene_alpha_property_);
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
  scene_monitor_->startSceneMonitor(planning_scene_topic_, planning_scene_diff_topic_);
  scene_robot_->update(PlanningLinkUpdater(&scene_monitor_->getPlanningScene()->getCurrentState()));
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

void PlanningDisplay::renderPlanningScene()
{
    static rviz::Color colors[] = {
        rviz::Color(0.2f, 0.9f, 0.2f),
        rviz::Color(0.0f, 0.1f, 0.9f),
        rviz::Color(0.0f, 0.9f, 0.9f),
        rviz::Color(0.9f, 0.9f, 0.0f),
        rviz::Color(0.9f, 0.0f, 0.2f)
    };
    if (!scene_monitor_)
	return;
    
    scene_monitor_->lockScene();
    ros::Time last_update = scene_monitor_->getLastUpdate();
    scene_monitor_->unlockScene();
    if (last_update <= last_scene_render_)
	return;
    
    scene_shapes_.clear();
    scene_monitor_->lockScene();
    last_scene_render_ = scene_monitor_->getLastUpdate();
    try
    {
        scene_robot_->update(PlanningLinkUpdater(&scene_monitor_->getPlanningScene()->getCurrentState()));
        collision_detection::CollisionWorldConstPtr cworld = scene_monitor_->getPlanningScene()->getCollisionWorld();
        const std::vector<std::string> &ns = cworld->getNamespaces();
        for (std::size_t i = 0 ; i < ns.size() ; ++i)
        {      
            collision_detection::CollisionWorld::NamespaceObjectsConstPtr o = cworld->getObjects(ns[i]);
            const rviz::Color &color = colors[i % (sizeof(colors)/sizeof(rviz::Color))];
            for (std::size_t j = 0 ; j < o->shapes_.size() ; ++j)
            {
                const shapes::Shape *s = o->shapes_[j];
                const btTransform &p = o->shape_poses_[j];
                ogre_tools::Shape* ogre_shape = NULL;
                switch (s->type)
                {
                case shapes::SPHERE:
                    {
                        ogre_shape = new ogre_tools::Shape(ogre_tools::Shape::Sphere,
                                                           vis_manager_->getSceneManager(), scene_node_);
                        double d = 2.0 * static_cast<const shapes::Sphere*>(s)->radius;
                        ogre_shape->setScale(Ogre::Vector3(d, d, d));
                    }
                    break;
                case shapes::BOX:
                    {
                        ogre_shape = new ogre_tools::Shape(ogre_tools::Shape::Cube,
                                                           vis_manager_->getSceneManager(), scene_node_);
                        const double* sz = static_cast<const shapes::Box*>(s)->size;
                        ogre_shape->setScale(Ogre::Vector3(sz[0], sz[1], sz[2]));
                    }
                    break;
                case shapes::CYLINDER:
                    {
                        ogre_shape = new ogre_tools::Shape(ogre_tools::Shape::Cylinder,
                                                           vis_manager_->getSceneManager(), scene_node_);
                        double d = 2.0 * static_cast<const shapes::Cylinder*>(s)->radius;
                        double z = static_cast<const shapes::Cylinder*>(s)->length;
                        ogre_shape->setScale(Ogre::Vector3(d, d, z));
                    }
                    break;
                case shapes::MESH:
                    ROS_WARN("Mesh rendering not yet implemented");
                    break;
                default:
                    break;
                }
                if (ogre_shape)
                {
                    ogre_shape->setColor(color.r_, color.g_, color.b_, scene_alpha_);
                    Ogre::Vector3 position(p.getOrigin().x(), p.getOrigin().y(), p.getOrigin().z());
                    const btQuaternion &q = p.getRotation();
                    Ogre::Quaternion orientation(q.getW(), q.getX(), q.getY(), q.getZ());
                    ogre_shape->setPosition(position);
                    ogre_shape->setOrientation(orientation);
		    scene_shapes_.push_back(boost::shared_ptr<ogre_tools::Shape>(ogre_shape));
                }
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
      scene_monitor_->updateFixedTransforms();
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
  if (vis_manager_->getTFClient()->getLatestCommonTime(target_frame_, scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp, &err_string) != tf::NO_ERROR)
      return;
  
  tf::Stamped<tf::Pose> pose(btTransform::getIdentity(), stamp, scene_monitor_->getPlanningScene()->getPlanningFrame());

  if (vis_manager_->getTFClient()->canTransform(target_frame_, scene_monitor_->getPlanningScene()->getPlanningFrame(), stamp))
  {
    try
    {
      vis_manager_->getTFClient()->transformPose(target_frame_, pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), target_frame_.c_str() );
    }
  }

  Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  const btQuaternion &q = pose.getRotation();
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

void PlanningDisplay::targetFrameChanged()
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
