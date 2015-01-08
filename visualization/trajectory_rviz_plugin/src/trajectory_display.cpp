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

/* Author: Dave Coleman */

#include <moveit/trajectory_rviz_plugin/trajectory_display.h>

#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <rviz/robot/robot.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/color_property.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

namespace moveit_rviz_plugin
{

TrajectoryDisplay::TrajectoryDisplay() :
  Display(),
  animating_path_(false)
{
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                              this,
                              SLOT( changedRobotDescription() ), this );

  trajectory_topic_property_ =
    new rviz::RosTopicProperty("Trajectory Topic", "/move_group/display_planned_path",
                               ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),
                               "The topic on which the moveit_msgs::DisplayTrajectory messages are received",
                               this,
                               SLOT(changedTrajectoryTopic()), this);

  display_path_visual_enabled_property_ =
    new rviz::BoolProperty("Show Robot Visual", true, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                           this,
                           SLOT(changedDisplayPathVisualEnabled()), this);

  display_path_collision_enabled_property_ =
    new rviz::BoolProperty("Show Robot Collision", false, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
                           this,
                           SLOT(changedDisplayPathCollisionEnabled()), this);

  robot_path_alpha_property_ =
    new rviz::FloatProperty("Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                            this,
                            SLOT(changedRobotPathAlpha()), this);
  robot_path_alpha_property_->setMin(0.0);
  robot_path_alpha_property_->setMax(1.0);

  state_display_time_property_ =  new rviz::EditableEnumProperty("State Display Time", "0.05 s",
                                                                 "The amount of wall-time to wait in between displaying states along a received trajectory path",
                                                                 this,
                                                                 SLOT(changedStateDisplayTime()), this);
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");

  loop_display_property_ =
    new rviz::BoolProperty("Loop Animation", false, "Indicates whether the last received path is to be animated in a loop",
                           this,
                           SLOT(changedLoopDisplay()), this);

  trail_display_property_ =
    new rviz::BoolProperty("Show Trail", false, "Show a path trail",
                           this,
                           SLOT(changedShowTrail()), this);

  connect(this, SIGNAL(timeToShowNewTrail()), this, SLOT(changedShowTrail()));
}

TrajectoryDisplay::~TrajectoryDisplay()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();

  display_path_robot_.reset();
}

void TrajectoryDisplay::onInitialize()
{
  Display::onInitialize();

  // Load trajectory robot
  display_path_robot_.reset(new RobotStateVisualization(scene_node_, context_, "Planned Path", this));
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);
}

void TrajectoryDisplay::reset()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;

  display_path_robot_->clear();
  Display::reset();

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);

  loadRobotModel();
}

void TrajectoryDisplay::clearTrajectoryTrail()
{
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    delete trajectory_trail_[i];
  trajectory_trail_.clear();
}

void TrajectoryDisplay::loadRobotModel()
{
  if (!rdf_loader_)
    rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

  if (rdf_loader_->getURDF())
  {
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
    robot_model_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
    display_path_robot_->load(*robot_model_->getURDF());
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    robot_state_->setToDefaultValues();
    setStatus( rviz::StatusProperty::Ok, "RobotModel", "Robot model loaded successfully" );
  }
  else
    setStatus( rviz::StatusProperty::Error, "RobotModel", "Unable to load robot model" );
}

void TrajectoryDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void TrajectoryDisplay::changedLoopDisplay()
{
  display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
}

void TrajectoryDisplay::changedShowTrail()
{
  clearTrajectoryTrail();

  if (!trail_display_property_->getBool())
    return;
  robot_trajectory::RobotTrajectoryPtr t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;
  if (!t)
    return;

  trajectory_trail_.resize(t->getWayPointCount());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    rviz::Robot *r = new rviz::Robot(scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), NULL);
    r->load(*getRobotModel()->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool());
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    r->update(PlanningLinkUpdater(t->getWayPointPtr(i)));
    r->setVisible(isEnabled() && (!animating_path_ || i <= current_state_));
    trajectory_trail_[i] = r;
  }
}

void TrajectoryDisplay::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setAlpha(robot_path_alpha_property_->getFloat());
}

void TrajectoryDisplay::changedTrajectoryTopic()
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
  {
    trajectory_topic_sub_ = update_nh_.subscribe(trajectory_topic_property_->getStdString(), 2, &TrajectoryDisplay::incomingDisplayTrajectory, this);
  }
}

void TrajectoryDisplay::changedDisplayPathVisualEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
    display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
  }
}

void TrajectoryDisplay::changedStateDisplayTime()
{
}

void TrajectoryDisplay::changedDisplayPathCollisionEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  }
}

void TrajectoryDisplay::onEnable()
{
  Display::onEnable();
  loadRobotModel();

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(displaying_trajectory_message_ && animating_path_);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
    trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    trajectory_trail_[i]->setVisible(true);
  }

  changedTrajectoryTopic(); // load topic at startup if default used
}

void TrajectoryDisplay::onDisable()
{
  display_path_robot_->setVisible(false);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setVisible(false);
  displaying_trajectory_message_.reset();

  Display::onDisable();
}

float TrajectoryDisplay::getStateDisplayTime()
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

void TrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  // Check if starting new trajectory
  if (!animating_path_ && !trajectory_message_to_display_ && loop_display_property_->getBool() && displaying_trajectory_message_)
  {
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = std::numeric_limits<float>::infinity();
    display_path_robot_->setVisible(isEnabled());
  }

  if (!animating_path_ && trajectory_message_to_display_ && !trajectory_message_to_display_->empty())
  {
    displaying_trajectory_message_ = trajectory_message_to_display_;
    display_path_robot_->setVisible(isEnabled());
    trajectory_message_to_display_.reset();
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = std::numeric_limits<float>::infinity();
    display_path_robot_->update(displaying_trajectory_message_->getFirstWayPointPtr());
  }

  if (animating_path_)
  {
    float tm = getStateDisplayTime();
    if (tm < 0.0) // if we should use realtime
      tm = displaying_trajectory_message_->getWayPointDurationFromPrevious(current_state_ + 1);
    if (current_state_time_ > tm)
    {
      ++current_state_;
      if ((std::size_t) current_state_ < displaying_trajectory_message_->getWayPointCount())
      {
        display_path_robot_->update(displaying_trajectory_message_->getWayPointPtr(current_state_));
        for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
          trajectory_trail_[i]->setVisible(i <= current_state_);
      }
      else
      {
        animating_path_ = false;
        display_path_robot_->setVisible(loop_display_property_->getBool());
      }
      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void TrajectoryDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  if (!msg->model_id.empty() && msg->model_id != getRobotModel()->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
             msg->model_id.c_str(), getRobotModel()->getName().c_str());

  trajectory_message_to_display_.reset();

  robot_trajectory::RobotTrajectoryPtr t(new robot_trajectory::RobotTrajectory(getRobotModel(), ""));
  for (std::size_t i = 0 ; i < msg->trajectory.size() ; ++i)
  {
    if (t->empty())
    {
      t->setRobotTrajectoryMsg(*robot_state_, msg->trajectory_start, msg->trajectory[i]);
    }
    else
    {
      robot_trajectory::RobotTrajectory tmp(getRobotModel(), "");
      tmp.setRobotTrajectoryMsg(t->getLastWayPoint(), msg->trajectory[i]);
      t->append(tmp, 0.0);
    }
  }

  if (!t->empty())
  {
    trajectory_message_to_display_.swap(t);
  }
  if (trail_display_property_->getBool())
  {
    // incomingDisplayTrajectory() can be called from a non-GUI thread, so here we
    // use a signal/slot connection to invoke changedShowTrail() in the GUI thread.
    Q_EMIT timeToShowNewTrail();
  }
}


} // namespace moveit_rviz_plugin
