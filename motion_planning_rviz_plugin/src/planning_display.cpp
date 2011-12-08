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

#include <ogre_tools/axes.h>

#include <tf/transform_listener.h>
#include <planning_scene_ros/robot_model_loader.h>
#include <planning_models/conversions.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

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

PlanningDisplay::PlanningDisplay(const std::string& name, rviz::VisualizationManager* manager) :
  Display(name, manager), new_kinematic_path_(false), animating_path_(false), state_display_time_(0.05f)
{
  robot_ = new rviz::Robot(vis_manager_, "Planning Robot " + name_);

  setVisualVisible(false);
  setCollisionVisible(true);

  setLoopDisplay(false);
  setAlpha(0.6f);
}

PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();

  delete robot_;
}

void PlanningDisplay::initialize(const std::string& description_param, const std::string& kinematic_path_topic)
{
  setRobotDescription(description_param);
  setTrajectoryTopic(kinematic_path_topic);
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

void PlanningDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;

  robot_->setAlpha(alpha_);

  propertyChanged(alpha_property_);
}

void PlanningDisplay::setTrajectoryTopic(const std::string& topic)
{
  unsubscribe();
  unadvertise();
  kinematic_path_topic_ = topic;
  subscribe();
  advertise();

  propertyChanged(trajectory_topic_property_);
}

void PlanningDisplay::setStateDisplayTime(float time)
{
  state_display_time_ = time;

  propertyChanged(state_display_time_property_);

  causeRender();
}

void PlanningDisplay::setSceneVisible(bool visible)
{
    display_scene_ = visible;
    
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

bool PlanningDisplay::isSceneVisible()
{
  return display_scene_;
}

bool PlanningDisplay::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool PlanningDisplay::isCollisionVisible()
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

  planning_scene_ros::RobotModelLoader rml(description_param_);
  if (rml.getURDF() && rml.getSRDF())
  {
      planning_scene_.reset(new planning_scene::PlanningScene());
      planning_scene_->configure(rml.getURDF(), rml.getSRDF());
  }
}

void PlanningDisplay::onEnable()
{
  subscribe();
  advertise();
  load();
  robot_->setVisible(true);
}

void PlanningDisplay::onDisable()
{
  unsubscribe();
  unadvertise();
  robot_->setVisible(false);
}

void PlanningDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!kinematic_path_topic_.empty())
  {
    sub_ = update_nh_.subscribe(kinematic_path_topic_, 2, &PlanningDisplay::incomingJointPath, this);
  }

}

void PlanningDisplay::unsubscribe()
{
  sub_.shutdown();
}

void PlanningDisplay::advertise()
{
  if (!isEnabled())
  {
    return;
  }
}

void PlanningDisplay::unadvertise()
{
}


void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  if (!planning_scene_)
    return;

  if (!animating_path_ && !new_kinematic_path_ && loop_display_ && displaying_kinematic_path_message_)
  {
      new_kinematic_path_ = true;
      incoming_kinematic_path_message_ = displaying_kinematic_path_message_;
  }
  
  if (!animating_path_ && new_kinematic_path_)
  {
      displaying_kinematic_path_message_ = incoming_kinematic_path_message_;
      displaying_path_states_ = std::max(displaying_kinematic_path_message_->trajectory.joint_trajectory.points.size(),
					 displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.points.size());
      animating_path_ = true;
      new_kinematic_path_ = false;
      current_state_ = -1;
      current_state_time_ = state_display_time_ + 1.0f;
      
      start_state_.reset(new planning_models::KinematicState(planning_scene_->getCurrentState()));
      robotStateToKinematicState(displaying_kinematic_path_message_->robot_state, *start_state_);
      /// \todo should use TF to pass in transforms
      
      robot_->update(PlanningLinkUpdater(start_state_.get()));
  }

  if (animating_path_)
  {
    if (current_state_time_ > state_display_time_)
    {
      ++current_state_;

      calculateRobotPosition();

      if ((std::size_t) current_state_ < displaying_path_states_)
      {
	moveit_msgs::RobotState rs;
	planning_models::robotTrajectoryPointToRobotState(displaying_kinematic_path_message_->trajectory, current_state_, rs);
	/// \todo pass in TF

	planning_models::KinematicState state(*start_state_);
	robotStateToKinematicState(rs, state);
	robot_->update(PlanningLinkUpdater(&state));
        causeRender();
      }
      else
      {
        animating_path_ = false;
      }

      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void PlanningDisplay::calculateRobotPosition()
{
  if (!displaying_kinematic_path_message_)
  {
    return;
  }

  const ros::Time &stamp = 
      displaying_kinematic_path_message_->trajectory.joint_trajectory.points.empty() ? 
      displaying_kinematic_path_message_->robot_state.joint_state.header.stamp :
      displaying_kinematic_path_message_->trajectory.joint_trajectory.header.stamp;
  
  tf::Stamped<tf::Pose> pose(btTransform::getIdentity(), stamp, planning_scene_->getPlanningFrame());

  if (vis_manager_->getTFClient()->canTransform(target_frame_, planning_scene_->getPlanningFrame(), stamp))
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
}

void PlanningDisplay::incomingJointPath(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  incoming_kinematic_path_message_ = msg;
  new_kinematic_path_ = true;
}

void PlanningDisplay::targetFrameChanged()
{
  calculateRobotPosition();
}

void PlanningDisplay::createProperties()
{
  scene_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Scene Enabled", property_prefix_,
										   boost::bind(&PlanningDisplay::isSceneVisible, this),
										   boost::bind(&PlanningDisplay::setSceneVisible, this, _1), parent_category_, this);
  setPropertyHelpText(scene_enabled_property_, "Indicates whether planning scenes should be displayed");
  
  visual_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Visual Enabled", property_prefix_,
										    boost::bind(&PlanningDisplay::isVisualVisible, this),
										    boost::bind(&PlanningDisplay::setVisualVisible, this, _1), parent_category_, this);
  setPropertyHelpText(visual_enabled_property_, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed");
  
  collision_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Collision Enabled", property_prefix_,
										       boost::bind(&PlanningDisplay::isCollisionVisible, this),
										       boost::bind(&PlanningDisplay::setCollisionVisible, this, _1), parent_category_, this);
  setPropertyHelpText(collision_enabled_property_, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed");
  
  state_display_time_property_ = property_manager_->createProperty<rviz::FloatProperty> ("State Display Time", property_prefix_,
											 boost::bind(&PlanningDisplay::getStateDisplayTime, this),
											 boost::bind(&PlanningDisplay::setStateDisplayTime, this, _1),
											 parent_category_, this);
  setPropertyHelpText(state_display_time_property_, "The amount of wall-time to wait in between displaying states along a received trajectory path");

  rviz::FloatPropertyPtr float_prop = state_display_time_property_.lock();
  float_prop->setMin(0.0001);

  loop_display_property_ = property_manager_->createProperty<rviz::BoolProperty>("Loop Display", property_prefix_, boost::bind(&PlanningDisplay::getLoopDisplay, this),
                                                                                 boost::bind(&PlanningDisplay::setLoopDisplay, this, _1), parent_category_, this);
  setPropertyHelpText(loop_display_property_, "Indicates whether the last received path is to be animated in a loop");  

  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Alpha", property_prefix_, boost::bind(&PlanningDisplay::getAlpha, this),
									    boost::bind(&PlanningDisplay::setAlpha, this, _1), parent_category_, this);
  setPropertyHelpText(alpha_property_, "Specifies the alpha for the robot links");  

  robot_description_property_ = property_manager_->createProperty<rviz::StringProperty> ("Robot Description", property_prefix_,
											 boost::bind(&PlanningDisplay::getRobotDescription, this),
											 boost::bind(&PlanningDisplay::setRobotDescription, this, _1), parent_category_,
											 this);
  setPropertyHelpText(robot_description_property_, "The name of the ROS parameter where the URDF for the robot is loaded");  

  trajectory_topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Trajectory Topic", property_prefix_,
												boost::bind(&PlanningDisplay::getTrajectoryTopic, this),
												boost::bind(&PlanningDisplay::setTrajectoryTopic, this, _1), parent_category_, this);
  setPropertyHelpText(trajectory_topic_property_, "The topic on which the moveit_msgs::DisplayTrajectory messages are received");

  rviz::ROSTopicStringPropertyPtr topic_prop = trajectory_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>());

  robot_->setPropertyManager(property_manager_, parent_category_);
}

} // namespace motion_planning_rviz_plugin


