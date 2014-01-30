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

/* Author: Ioan Sucan, Dave Coleman, Adam Leeper, Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
MotionPlanningDisplay::MotionPlanningDisplay() :
  PlanningSceneDisplay(),
  text_to_display_(NULL),
  private_handle_("~"),
  animating_path_(false),
  frame_(NULL),
  frame_dock_(NULL),
  menu_handler_start_(new interactive_markers::MenuHandler),
  menu_handler_goal_(new interactive_markers::MenuHandler),
  int_marker_display_(NULL)
{
  // Category Groups
  plan_category_  = new rviz::Property("Planning Request", QVariant(), "", this);
  metrics_category_ = new rviz::Property("Planning Metrics", QVariant(), "", this);
  path_category_  = new rviz::Property("Planned Path", QVariant(), "", this);

  // Metrics category -----------------------------------------------------------------------------------------
  compute_weight_limit_property_ = new rviz::BoolProperty("Show Weight Limit", false, "Shows the weight limit at a particular pose for an end-effector",
                                                          metrics_category_,
                                                          SLOT(changedShowWeightLimit()), this);

  show_manipulability_index_property_ = new rviz::BoolProperty("Show Manipulability Index", false, "Shows the manipulability index for an end-effector",
                                                               metrics_category_,
                                                               SLOT(changedShowManipulabilityIndex()), this);

  show_manipulability_property_ = new rviz::BoolProperty("Show Manipulability", false, "Shows the manipulability for an end-effector",
                                                         metrics_category_,
                                                         SLOT(changedShowManipulability()), this);

  show_joint_torques_property_ = new rviz::BoolProperty("Show Joint Torques", false, "Shows the joint torques for a given configuration and payload",
                                                        metrics_category_,
                                                        SLOT(changedShowJointTorques()), this);

  metrics_set_payload_property_ =
    new rviz::FloatProperty("Payload", 1.0f, "Specify the payload at the end effector (kg)",
                            metrics_category_,
                            SLOT(changedMetricsSetPayload()), this);
  metrics_set_payload_property_->setMin(0.0);

  metrics_text_height_property_ =
    new rviz::FloatProperty("TextHeight", 0.08f, "Text height",
                            metrics_category_,
                            SLOT(changedMetricsTextHeight()), this);
  metrics_text_height_property_->setMin(0.001);

  // Planning request category -----------------------------------------------------------------------------------------

  planning_group_property_ = new rviz::EditableEnumProperty("Planning Group", "", "The name of the group of links to plan for (from the ones defined in the SRDF)",
                                                            plan_category_,
                                                            SLOT(changedPlanningGroup()), this);
  show_workspace_property_ = new rviz::BoolProperty("Show Workspace", false, "Shows the axis-aligned bounding box for the workspace allowed for planning",
                                                    plan_category_,
                                                    SLOT(changedWorkspace()), this);
  query_start_state_property_ = new rviz::BoolProperty("Query Start State", true, "Shows the start state for the motion planning query",
                                                       plan_category_,
                                                       SLOT(changedQueryStartState()), this);
  query_goal_state_property_ = new rviz::BoolProperty("Query Goal State", true, "Shows the goal state for the motion planning query",
                                                      plan_category_,
                                                      SLOT(changedQueryGoalState()), this);
  query_marker_scale_property_ = new rviz::FloatProperty("Interactive Marker Size", 0.0f, "Specifies scale of the interactive marker overlayed on the robot",
                                                         plan_category_,
                                                         SLOT(changedQueryMarkerScale()), this);
  query_marker_scale_property_->setMin(0.0f);

  query_start_color_property_ = new rviz::ColorProperty("Start State Color", QColor(0, 255, 0), "The highlight color for the start state",
                                                        plan_category_,
                                                        SLOT(changedQueryStartColor()), this);
  query_start_alpha_property_ = new rviz::FloatProperty("Start State Alpha", 1.0f, "Specifies the alpha for the robot links",
                                                        plan_category_,
                                                        SLOT(changedQueryStartAlpha()), this);
  query_start_alpha_property_->setMin(0.0);
  query_start_alpha_property_->setMax(1.0);


  query_goal_color_property_ = new rviz::ColorProperty("Goal State Color", QColor(250, 128, 0), "The highlight color for the goal state",
                                                       plan_category_,
                                                       SLOT(changedQueryGoalColor()), this);

  query_goal_alpha_property_ =
    new rviz::FloatProperty("Goal State Alpha", 1.0f, "Specifies the alpha for the robot links",
                            plan_category_,
                            SLOT(changedQueryGoalAlpha()), this);
  query_goal_alpha_property_->setMin(0.0);
  query_goal_alpha_property_->setMax(1.0);

  query_colliding_link_color_property_ = new rviz::ColorProperty("Colliding Link Color", QColor(255, 0, 0), "The highlight color for colliding links",
                                                                 plan_category_,
                                                                 SLOT(changedQueryCollidingLinkColor()), this);

  query_outside_joint_limits_link_color_property_ = new rviz::ColorProperty("Joint Violation Color", QColor(255, 0, 255),
                                                                            "The highlight color for child links of joints that are outside bounds",
                                                                            plan_category_,
                                                                            SLOT(changedQueryJointViolationColor()), this);
  // Path category ----------------------------------------------------------------------------------------------------

  trajectory_topic_property_ =
    new rviz::RosTopicProperty("Trajectory Topic", "/move_group/display_planned_path",
                               ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),
                               "The topic on which the moveit_msgs::DisplayTrajectory messages are received",
                               path_category_,
                               SLOT(changedTrajectoryTopic()), this);

  display_path_visual_enabled_property_ =
    new rviz::BoolProperty("Show Robot Visual", true, "Indicates whether the geometry of the robot as defined for visualisation purposes should be displayed",
                           path_category_,
                           SLOT(changedDisplayPathVisualEnabled()), this);

  display_path_collision_enabled_property_ =
    new rviz::BoolProperty("Show Robot Collision", false, "Indicates whether the geometry of the robot as defined for collision detection purposes should be displayed",
                           path_category_,
                           SLOT(changedDisplayPathCollisionEnabled()), this);

  robot_path_alpha_property_ =
    new rviz::FloatProperty("Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                            path_category_,
                            SLOT(changedRobotPathAlpha()), this);
  robot_path_alpha_property_->setMin(0.0);
  robot_path_alpha_property_->setMax(1.0);

  state_display_time_property_ =  new rviz::EditableEnumProperty("State Display Time", "0.05 s",
                                                                 "The amount of wall-time to wait in between displaying states along a received trajectory path",
                                                                 path_category_,
                                                                 SLOT(changedStateDisplayTime()), this);
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");

  loop_display_property_ =
    new rviz::BoolProperty("Loop Animation", false, "Indicates whether the last received path is to be animated in a loop",
                           path_category_,
                           SLOT(changedLoopDisplay()), this);

  trail_display_property_ =
    new rviz::BoolProperty("Show Trail", false, "Show a path trail",
                           path_category_,
                           SLOT(changedShowTrail()), this);

  background_process_.setJobUpdateEvent(boost::bind(&MotionPlanningDisplay::backgroundJobUpdate, this, _1, _2));

  connect(this, SIGNAL(timeToShowNewTrail()), this, SLOT(changedShowTrail()));
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
MotionPlanningDisplay::~MotionPlanningDisplay()
{
  background_process_.clearJobUpdateEvent();
  clearJobs();

  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();

  display_path_robot_.reset();
  query_robot_start_.reset();
  query_robot_goal_.reset();

  delete text_to_display_;
  delete int_marker_display_;
  delete frame_dock_;
}

void MotionPlanningDisplay::onInitialize()
{
  PlanningSceneDisplay::onInitialize();

  display_path_robot_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Planned Path", path_category_));
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);

  query_robot_start_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Planning Request Start", NULL));
  query_robot_start_->setCollisionVisible(false);
  query_robot_start_->setVisualVisible(true);
  query_robot_start_->setVisible(query_start_state_property_->getBool());
  std_msgs::ColorRGBA color; QColor qcolor = query_start_color_property_->getColor();
  color.r = qcolor.redF(); color.g = qcolor.greenF(); color.b = qcolor.blueF(); color.a = 1.0f;
  query_robot_start_->setDefaultAttachedObjectColor(color);

  query_robot_goal_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Planning Request Goal", NULL));
  query_robot_goal_->setCollisionVisible(false);
  query_robot_goal_->setVisualVisible(true);
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  qcolor = query_goal_color_property_->getColor();
  color.r = qcolor.redF(); color.g = qcolor.greenF(); color.b = qcolor.blueF();
  query_robot_goal_->setDefaultAttachedObjectColor(color);

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new MotionPlanningFrame(this, context_, window_context ? window_context->getParentWindow() : NULL);
  resetStatusTextColor();
  addStatusText("Initialized.");

  if (window_context)
    frame_dock_ = window_context->addPane("Motion Planning", frame_);

  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);

  text_display_scene_node_ = planning_scene_node_->createChildSceneNode();
  text_to_display_ = new rviz::MovableText("EMPTY");
  text_to_display_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_to_display_->setCharacterHeight(metrics_text_height_property_->getFloat());
  text_to_display_->showOnTop();
  text_to_display_->setVisible(false);
  text_display_for_start_ = false;
  text_display_scene_node_->attachObject(text_to_display_);

  planning_group_sub_ = node_handle_.subscribe("/rviz/moveit/select_planning_group", 1, &MotionPlanningDisplay::selectPlanningGroupCallback, this);

  if (context_ && context_->getWindowManager() && context_->getWindowManager()->getParentWindow())
  {
    QShortcut *im_reset_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_R), context_->getWindowManager()->getParentWindow());
    connect(im_reset_shortcut, SIGNAL(activated()), this, SLOT(resetInteractiveMarkers()));
  }
}

void MotionPlanningDisplay::selectPlanningGroupCallback(const std_msgs::StringConstPtr& msg)
{
  if (!getRobotModel() || !robot_interaction_)
    return;
  if (getRobotModel()->hasJointModelGroup(msg->data))
  {
    planning_group_property_->setStdString(msg->data);
    changedPlanningGroup();
  }
  else
  {
    ROS_ERROR("Group [%s] not found in the robot model.", msg->data.c_str());
  }
}
void MotionPlanningDisplay::reset()
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

  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);
}

void MotionPlanningDisplay::backgroundJobUpdate(moveit::tools::BackgroundProcessing::JobEvent , const std::string &)
{
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::updateBackgroundJobProgressBar, this));
}

void MotionPlanningDisplay::updateBackgroundJobProgressBar()
{
  if (!frame_)
    return;
  QProgressBar *p = frame_->ui_->background_job_progress;
  std::size_t n = background_process_.getJobCount();

  if (n == 0)
  {
    p->setValue(p->maximum());
    p->update();
    p->hide();
    p->setMaximum(0);
  }
  else
  {
    if (n == 1)
    {
      if (p->maximum() == 0)
        p->setValue(0);
      else
        p->setValue(p->maximum() - 1);
    }
    else
    {
      if (p->maximum() < n)
        p->setMaximum(n);
      else
        p->setValue(p->maximum() - n);
    }
    p->show();
    p->update();
  }
}

void MotionPlanningDisplay::changedShowWeightLimit()
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

void MotionPlanningDisplay::changedShowManipulabilityIndex()
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

void MotionPlanningDisplay::changedShowManipulability()
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

void MotionPlanningDisplay::changedShowJointTorques()
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

void MotionPlanningDisplay::changedMetricsSetPayload()
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

void MotionPlanningDisplay::changedMetricsTextHeight()
{
  text_to_display_->setCharacterHeight(metrics_text_height_property_->getFloat());
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

void MotionPlanningDisplay::clearTrajectoryTrail()
{
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    delete trajectory_trail_[i];
  trajectory_trail_.clear();
}

void MotionPlanningDisplay::changedLoopDisplay()
{
  display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
}

void MotionPlanningDisplay::changedShowTrail()
{
  clearTrajectoryTrail();

  if (!trail_display_property_->getBool() || !planning_scene_monitor_)
    return;
  robot_trajectory::RobotTrajectoryPtr t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;
  if (!t)
    return;

  trajectory_trail_.resize(t->getWayPointCount());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    rviz::Robot *r = new rviz::Robot(planning_scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), NULL);
    r->load(*getRobotModel()->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool());
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    r->update(PlanningLinkUpdater(t->getWayPointPtr(i)));
    r->setVisible(isEnabled() && (!animating_path_ || i <= current_state_));
    trajectory_trail_[i] = r;
  }
}

void MotionPlanningDisplay::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setAlpha(robot_path_alpha_property_->getFloat());
}

void MotionPlanningDisplay::renderWorkspaceBox()
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

void MotionPlanningDisplay::changedTrajectoryTopic()
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
  {
    trajectory_topic_sub_ = update_nh_.subscribe(trajectory_topic_property_->getStdString(), 2, &MotionPlanningDisplay::incomingDisplayTrajectory, this);
  }
}

void MotionPlanningDisplay::computeMetrics(bool start, const std::string &group, double payload)
{
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  if (eef.empty())
    return;
  boost::mutex::scoped_lock slock(update_metrics_lock_);

  robot_state::RobotStateConstPtr state = start ? getQueryStartState() : getQueryGoalState();
  for (std::size_t i = 0 ; i < eef.size() ; ++i)
    if (eef[i].parent_group == group)
      computeMetricsInternal(computed_metrics_[std::make_pair(start, group)], eef[i], *state, payload);
}

void MotionPlanningDisplay::computeMetricsInternal(std::map<std::string, double> &metrics, const robot_interaction::RobotInteraction::EndEffector &ee,
                                                   const robot_state::RobotState &state, double payload)
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
    state.copyJointGroupPositions(ee.parent_group, joint_values);
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
    if (position_only_ik_.find(ee.parent_group) == position_only_ik_.end())
      private_handle_.param(ee.parent_group + "/position_only_ik", position_only_ik_[ee.parent_group], false);

    double manipulability_index, manipulability;
    bool position_ik = position_only_ik_[ee.parent_group];
    if (kinematics_metrics_->getManipulabilityIndex(state, ee.parent_group, manipulability_index, position_ik))
      metrics["manipulability_index"] = manipulability_index;
    if (kinematics_metrics_->getManipulability(state, ee.parent_group, manipulability))
      metrics["manipulability"] = manipulability;
  }
}

namespace
{
inline void copyItemIfExists(const std::map<std::string, double> &source, std::map<std::string, double> &dest, const std::string &key)
{
  std::map<std::string, double>::const_iterator it = source.find(key);
  if (it != source.end())
    dest[key] = it->second;
}
}

void MotionPlanningDisplay::displayMetrics(bool start)
{
  if (!robot_interaction_ || !planning_scene_monitor_)
    return;

  static const Ogre::Quaternion orientation(1.0, 0.0, 0.0, 0.0);
  const std::vector<robot_interaction::RobotInteraction::EndEffector> &eef = robot_interaction_->getActiveEndEffectors();
  if (eef.empty())
    return;

  robot_state::RobotStateConstPtr state = start ? getQueryStartState() : getQueryGoalState();

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
      std::size_t nj = getRobotModel()->getJointModelGroup(eef[i].parent_group)->getJointModelNames().size();
      for(size_t j = 0 ; j < nj ; ++j)
      {
        std::stringstream stream;
        stream << "torque[" << j << "]";
        copyItemIfExists(metrics_table, text_table, stream.str());
      }
    }

    const robot_state::LinkModel *lm = NULL;
    const robot_model::JointModelGroup *jmg = getRobotModel()->getJointModelGroup(eef[i].parent_group);
    if (jmg)
      if (!jmg->getLinkModelNames().empty())
        lm = state->getLinkModel(jmg->getLinkModelNames().back());
    if (lm)
    {
      const Eigen::Vector3d &t = state->getGlobalLinkTransform(lm).translation();
      position[0] = t.x();
      position[1] = t.y();
      position[2] = t.z() + 0.2; // \todo this should be a param
    }
    if (start)
      displayTable(text_table, query_start_color_property_->getOgreColor(), position, orientation);
    else
      displayTable(text_table, query_goal_color_property_->getOgreColor(), position, orientation);
    text_display_for_start_ = start;
  }
}

void MotionPlanningDisplay::drawQueryStartState()
{
  if (!planning_scene_monitor_)
    return;

  if (query_start_state_property_->getBool())
  {
    if (isEnabled())
    {
      robot_state::RobotStateConstPtr state = getQueryStartState();
      
      // update link poses
      query_robot_start_->update(state);
      query_robot_start_->setVisible(true);
      
      // update link colors
      std::vector<std::string> collision_links;
      getPlanningSceneRO()->getCollidingLinks(collision_links, *state);
      status_links_start_.clear();
      for (std::size_t i = 0 ; i < collision_links.size() ; ++i)
        status_links_start_[collision_links[i]] = COLLISION_LINK;
      if (!collision_links.empty())
      {
        collision_detection::CollisionResult::ContactMap pairs;
        getPlanningSceneRO()->getCollidingPairs(pairs, *state);
        setStatusTextColor(query_start_color_property_->getColor());
        addStatusText("Start state colliding links:");
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = pairs.begin() ; it != pairs.end() ; ++it)
          addStatusText(it->first.first + " - " + it->first.second);
        addStatusText(".");
      }
      if (!getCurrentPlanningGroup().empty())
      {
        const robot_model::JointModelGroup *jmg = state->getJointModelGroup(getCurrentPlanningGroup());
        if (jmg)
        {
          std::vector<std::string> outside_bounds; 
          const std::vector<const robot_model::JointModel*> &jmodels = jmg->getActiveJointModels();
          for (std::size_t i = 0 ; i < jmodels.size() ; ++i)
            if (!state->satisfiesBounds(jmodels[i], jmodels[i]->getMaximumExtent() * 1e-2))
            {
              outside_bounds.push_back(jmodels[i]->getChildLinkModel()->getName());
              status_links_start_[outside_bounds.back()] = OUTSIDE_BOUNDS_LINK;
            }
          if (!outside_bounds.empty())
          {
            setStatusTextColor(query_start_color_property_->getColor());
            addStatusText("Links descending from joints that are outside bounds in start state:");
            addStatusText(outside_bounds);
          }
        }
      }
      updateLinkColors();
      // update metrics text
      displayMetrics(true);
    }
  }
  else
    query_robot_start_->setVisible(false);
  context_->queueRender();
}

void MotionPlanningDisplay::resetStatusTextColor()
{
  setStatusTextColor(Qt::darkGray);
}

void MotionPlanningDisplay::setStatusTextColor(const QColor &color)
{
  if (frame_)
    frame_->ui_->status_text->setTextColor(color);
}

void MotionPlanningDisplay::addStatusText(const std::string &text)
{
  if (frame_)
    frame_->ui_->status_text->append(QString::fromStdString(text));
}

void MotionPlanningDisplay::addStatusText(const std::vector<std::string> &text)
{
  for (std::size_t i = 0 ; i < text.size() ; ++i)
    addStatusText(text[i]);
}

void MotionPlanningDisplay::recomputeQueryStartStateMetrics()
{
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(true, group, metrics_set_payload_property_->getFloat());
}

void MotionPlanningDisplay::recomputeQueryGoalStateMetrics()
{
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(false, group, metrics_set_payload_property_->getFloat());
}

void MotionPlanningDisplay::changedQueryStartState()
{
  if (!planning_scene_monitor_)
    return;
  setStatusTextColor(query_start_color_property_->getColor());
  addStatusText("Changed start state");
  drawQueryStartState();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, true), "publishInteractiveMarkers");
}

void MotionPlanningDisplay::changedQueryGoalState()
{
  if (!planning_scene_monitor_)
    return;
  setStatusTextColor(query_goal_color_property_->getColor());
  addStatusText("Changed goal state");
  drawQueryGoalState();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, true), "publishInteractiveMarkers");
}

void MotionPlanningDisplay::drawQueryGoalState()
{
  if (!planning_scene_monitor_)
    return;
  if (query_goal_state_property_->getBool())
  {
    if (isEnabled())
    {
      robot_state::RobotStateConstPtr state = getQueryGoalState();
      
      // update link poses
      query_robot_goal_->update(state);
      query_robot_goal_->setVisible(true);

      // update link colors
      std::vector<std::string> collision_links;
      getPlanningSceneRO()->getCollidingLinks(collision_links, *state);
      status_links_goal_.clear();
      for (std::size_t i = 0 ; i < collision_links.size() ; ++i)
        status_links_goal_[collision_links[i]] = COLLISION_LINK;
      if (!collision_links.empty())
      {
        collision_detection::CollisionResult::ContactMap pairs;
        getPlanningSceneRO()->getCollidingPairs(pairs, *state);
        setStatusTextColor(query_goal_color_property_->getColor());
        addStatusText("Goal state colliding links:");
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = pairs.begin() ; it != pairs.end() ; ++it)
          addStatusText(it->first.first + " - " + it->first.second);
        addStatusText(".");
      }
      
      if (!getCurrentPlanningGroup().empty())
      {
        const robot_model::JointModelGroup *jmg = state->getJointModelGroup(getCurrentPlanningGroup());
        if (jmg)
        {
          const std::vector<const robot_state::JointModel*> &jmodels = jmg->getActiveJointModels();
          std::vector<std::string> outside_bounds;
          for (std::size_t i = 0 ; i < jmodels.size() ; ++i)
            if (!state->satisfiesBounds(jmodels[i], jmodels[i]->getMaximumExtent() * 1e-2))
            {
              outside_bounds.push_back(jmodels[i]->getChildLinkModel()->getName());
              status_links_goal_[outside_bounds.back()] = OUTSIDE_BOUNDS_LINK;
            }
          
          if (!outside_bounds.empty())
          {
            setStatusTextColor(query_goal_color_property_->getColor());
            addStatusText("Links descending from joints that are outside bounds in goal state:");
            addStatusText(outside_bounds);
          }
        }
      }
      updateLinkColors();

      // update metrics text
      displayMetrics(false);
    }
  }
  else
    query_robot_goal_->setVisible(false);
  context_->queueRender();
}

void MotionPlanningDisplay::resetInteractiveMarkers()
{
  query_start_state_->clearError();
  query_goal_state_->clearError();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, false), "publishInteractiveMarkers");
}

void MotionPlanningDisplay::publishInteractiveMarkers(bool pose_update)
{
  if (robot_interaction_)
  {
    if (pose_update &&
        robot_interaction_->showingMarkers(query_start_state_) == query_start_state_property_->getBool() &&
        robot_interaction_->showingMarkers(query_goal_state_) == query_goal_state_property_->getBool())
    {
      if (query_start_state_property_->getBool())
        robot_interaction_->updateInteractiveMarkers(query_start_state_);
      if (query_goal_state_property_->getBool())
        robot_interaction_->updateInteractiveMarkers(query_goal_state_);
    }
    else
    {
      robot_interaction_->clearInteractiveMarkers();
      if (query_start_state_property_->getBool())
        robot_interaction_->addInteractiveMarkers(query_start_state_, query_marker_scale_property_->getFloat());
      if (query_goal_state_property_->getBool())
        robot_interaction_->addInteractiveMarkers(query_goal_state_, query_marker_scale_property_->getFloat());
      robot_interaction_->publishInteractiveMarkers();
    }
  }
}

void MotionPlanningDisplay::changedQueryStartColor()
{
  std_msgs::ColorRGBA color;
  QColor qcolor = query_start_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = 1.0f;
  query_robot_start_->setDefaultAttachedObjectColor(color);
  changedQueryStartState();
}

void MotionPlanningDisplay::changedQueryStartAlpha()
{
  query_robot_start_->setAlpha(query_start_alpha_property_->getFloat());
  changedQueryStartState();
}

void MotionPlanningDisplay::changedQueryMarkerScale()
{
  if (!planning_scene_monitor_)
    return;

  if (isEnabled())
  {
    // Clear the interactive markers and re-add them:
    publishInteractiveMarkers(false);
  }
}

void MotionPlanningDisplay::changedQueryGoalColor()
{
  std_msgs::ColorRGBA color;
  QColor qcolor = query_goal_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = 1.0f;
  query_robot_goal_->setDefaultAttachedObjectColor(color);
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryGoalAlpha()
{
  query_robot_goal_->setAlpha(query_goal_alpha_property_->getFloat());
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryCollidingLinkColor()
{
  changedQueryStartState();
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryJointViolationColor()
{
  changedQueryStartState();
  changedQueryGoalState();
}

void MotionPlanningDisplay::scheduleDrawQueryStartState(robot_interaction::RobotInteraction::InteractionHandler *, bool error_state_changed)
{
  if (!planning_scene_monitor_)
    return;
  if (error_state_changed)
    addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, false), "publishInteractiveMarkers");
  recomputeQueryStartStateMetrics();
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::drawQueryStartState, this));
  context_->queueRender();
}

void MotionPlanningDisplay::scheduleDrawQueryGoalState(robot_interaction::RobotInteraction::InteractionHandler *, bool error_state_changed)
{
  if (!planning_scene_monitor_)
    return;
  if (error_state_changed)
    addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, false), "publishInteractiveMarkers");
  recomputeQueryGoalStateMetrics();
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::drawQueryGoalState, this));
  context_->queueRender();
}

void MotionPlanningDisplay::updateQueryStartState()
{
  recomputeQueryStartStateMetrics();
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryStartState, this));
  context_->queueRender();
}

void MotionPlanningDisplay::updateQueryGoalState()
{
  recomputeQueryGoalStateMetrics();
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryGoalState, this));
  context_->queueRender();
}

void MotionPlanningDisplay::setQueryStartState(const robot_state::RobotState &start)
{
  query_start_state_->setState(start);
  updateQueryStartState();
}

void MotionPlanningDisplay::setQueryGoalState(const robot_state::RobotState &goal)
{
  query_goal_state_->setState(goal);
  updateQueryGoalState();
}

void MotionPlanningDisplay::useApproximateIK(bool flag)
{
  if (query_start_state_)
  {
    kinematics::KinematicsQueryOptions o = query_start_state_->getKinematicsQueryOptions();
    o.return_approximate_solution = flag;
    query_start_state_->setKinematicsQueryOptions(o);
  }
  if (query_goal_state_)
  {
    kinematics::KinematicsQueryOptions o = query_goal_state_->getKinematicsQueryOptions();
    o.return_approximate_solution = flag;
    query_goal_state_->setKinematicsQueryOptions(o);
  }
}

bool MotionPlanningDisplay::isIKSolutionCollisionFree(robot_state::RobotState *state, const robot_model::JointModelGroup *group, const double *ik_solution) const
{
  if (frame_->ui_->collision_aware_ik->isChecked() && planning_scene_monitor_)
  {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    bool res = !getPlanningSceneRO()->isStateColliding(*state, group->getName());
    return res;
  }
  else
    return true;
}

void MotionPlanningDisplay::updateLinkColors()
{
  unsetAllColors(&query_robot_start_->getRobot());
  unsetAllColors(&query_robot_goal_->getRobot());
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
  {
    setGroupColor(&query_robot_start_->getRobot(), group, query_start_color_property_->getColor());
    setGroupColor(&query_robot_goal_->getRobot(), group, query_goal_color_property_->getColor());

    for (std::map<std::string, LinkDisplayStatus>::const_iterator it = status_links_start_.begin() ; it != status_links_start_.end() ; ++it)
      if (it->second == COLLISION_LINK)
        setLinkColor(&query_robot_start_->getRobot(), it->first, query_colliding_link_color_property_->getColor());
      else
        setLinkColor(&query_robot_start_->getRobot(), it->first, query_outside_joint_limits_link_color_property_->getColor());

    for (std::map<std::string, LinkDisplayStatus>::const_iterator it = status_links_goal_.begin() ; it != status_links_goal_.end() ; ++it)
      if (it->second == COLLISION_LINK)
        setLinkColor(&query_robot_goal_->getRobot(), it->first, query_colliding_link_color_property_->getColor());
      else
        setLinkColor(&query_robot_goal_->getRobot(), it->first, query_outside_joint_limits_link_color_property_->getColor());
  }
}

void MotionPlanningDisplay::changePlanningGroup(const std::string& group)
{
  if (!getRobotModel() || !robot_interaction_)
    return;

  if (getRobotModel()->hasJointModelGroup(group))
  {
    planning_group_property_->setStdString(group);
    changedPlanningGroup();
  }
  else 
    ROS_ERROR("Group [%s] not found in the robot model.", group.c_str());
}

void MotionPlanningDisplay::changedPlanningGroup()
{
  if (!getRobotModel() || !robot_interaction_)
    return;

  if (!planning_group_property_->getStdString().empty())
    if (!getRobotModel()->hasJointModelGroup(planning_group_property_->getStdString()))
    {
      planning_group_property_->setStdString("");
      return;
    }
  modified_groups_.insert(planning_group_property_->getStdString());
  
  if (robot_interaction_)
    robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());
  
  updateQueryStartState();
  updateQueryGoalState();
  updateLinkColors();

  if (frame_)
    frame_->changePlanningGroup();
  addBackgroundJob(boost::bind(&MotionPlanningDisplay::publishInteractiveMarkers, this, false), "publishInteractiveMarkers");
}

void MotionPlanningDisplay::changedWorkspace()
{
  renderWorkspaceBox();
}

std::string MotionPlanningDisplay::getCurrentPlanningGroup() const
{
  return planning_group_property_->getStdString();
}

void MotionPlanningDisplay::changedStateDisplayTime()
{
}

void MotionPlanningDisplay::changedDisplayPathVisualEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
    display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
  }
}

// ******************************************************************************************
// Collision Visible
// ******************************************************************************************

void MotionPlanningDisplay::changedDisplayPathCollisionEnabled()
{
  if (isEnabled())
  {
    display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    display_path_robot_->setVisible(isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
      trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  }
}

void MotionPlanningDisplay::setQueryStateHelper(bool use_start_state, const std::string &state_name)
{
  robot_state::RobotState state = use_start_state ? *getQueryStartState() : *getQueryGoalState();
  
  std::string v = "<" + state_name + ">";

  if (v == "<random>")
  {
    if (const robot_state::JointModelGroup *jmg = state.getJointModelGroup(getCurrentPlanningGroup()))
      state.setToRandomPositions(jmg);
  }
  else
    if (v == "<current>")
    {
      const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
      if (ps)
        state = ps->getCurrentState();
    }
    else
      if (v == "<same as goal>")
      {
        state = *getQueryGoalState();
      }
      else
        if (v == "<same as start>")
        {
          state = *getQueryStartState();
        }
        else
        {
          // maybe it is a named state
          if (const robot_model::JointModelGroup *jmg = state.getJointModelGroup(getCurrentPlanningGroup()))
            state.setToDefaultValues(jmg, state_name);
        }
  
  use_start_state ? setQueryStartState(state) : setQueryGoalState(state);
}

void MotionPlanningDisplay::populateMenuHandler(boost::shared_ptr<interactive_markers::MenuHandler>& mh)
{
  typedef interactive_markers::MenuHandler immh;
  std::vector<std::string> state_names;
  state_names.push_back("random");
  state_names.push_back("current");
  state_names.push_back("same as start");
  state_names.push_back("same as goal");

  // hacky way to distinguish between the start and goal handlers...
  bool is_start = (mh.get() == menu_handler_start_.get());

  // Commands for changing the state
  immh::EntryHandle menu_states = mh->insert(is_start ? "Set start state to" : "Set goal state to" ,
                                             immh::FeedbackCallback());
  for (int i = 0; i < state_names.size(); ++i)
  {
    // Don't add "same as start" to the start state handler, and vice versa.
    if ((state_names[i] == "same as start" && is_start) || (state_names[i] == "same as goal"  && !is_start))
      continue;
    mh->insert(menu_states, state_names[i],
               boost::bind(&MotionPlanningDisplay::setQueryStateHelper, this, is_start, state_names[i]));
  }
  
  //  // Group commands, which end up being the same for both interaction handlers
  //  const std::vector<std::string>& group_names = getRobotModel()->getJointModelGroupNames();
  //  immh::EntryHandle menu_groups = mh->insert("Planning Group", immh::FeedbackCallback());
  //  for (int i = 0; i < group_names.size(); ++i)
  //    mh->insert(menu_groups, group_names[i],
  //               boost::bind(&MotionPlanningDisplay::changePlanningGroup, this, group_names[i]));

}

void MotionPlanningDisplay::onRobotModelLoaded()
{
  PlanningSceneDisplay::onRobotModelLoaded();

  robot_interaction_.reset(new robot_interaction::RobotInteraction(getRobotModel(), "rviz_moveit_motion_planning_display"));
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction_->getServerTopic() + "/update"));
  display_path_robot_->load(*getRobotModel()->getURDF());
  query_robot_start_->load(*getRobotModel()->getURDF());
  query_robot_goal_->load(*getRobotModel()->getURDF());

  robot_state::RobotStatePtr ks(new robot_state::RobotState(getPlanningSceneRO()->getCurrentState()));
  query_start_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("start", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *getQueryStartState(), planning_scene_monitor_->getTFClient()));
  query_start_state_->setUpdateCallback(boost::bind(&MotionPlanningDisplay::scheduleDrawQueryStartState, this, _1, _2));
  query_goal_state_->setUpdateCallback(boost::bind(&MotionPlanningDisplay::scheduleDrawQueryGoalState, this, _1, _2));
  query_start_state_->setGroupStateValidityCallback(boost::bind(&MotionPlanningDisplay::isIKSolutionCollisionFree, this, _1, _2, _3));
  query_goal_state_->setGroupStateValidityCallback(boost::bind(&MotionPlanningDisplay::isIKSolutionCollisionFree, this, _1, _2, _3));

  // Interactive marker menus
  populateMenuHandler(menu_handler_start_);
  populateMenuHandler(menu_handler_goal_);
  query_start_state_->setMenuHandler(menu_handler_start_);
  query_goal_state_->setMenuHandler(menu_handler_goal_);

  if (!planning_group_property_->getStdString().empty())
    if (!getRobotModel()->hasJointModelGroup(planning_group_property_->getStdString()))
      planning_group_property_->setStdString("");

  const std::vector<std::string> &groups = getRobotModel()->getJointModelGroupNames();
  planning_group_property_->clearOptions();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    planning_group_property_->addOptionStd(groups[i]);
  planning_group_property_->sortOptions();
  if (!groups.empty() && planning_group_property_->getStdString().empty())
    planning_group_property_->setStdString(groups[0]);

  modified_groups_.clear();
  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(getRobotModel()));

  geometry_msgs::Vector3 gravity_vector;
  gravity_vector.x = 0.0;
  gravity_vector.y = 0.0;
  gravity_vector.z = 9.81;

  dynamics_solver_.clear();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    if (getRobotModel()->getJointModelGroup(groups[i])->isChain())
      dynamics_solver_[groups[i]].reset(new dynamics_solver::DynamicsSolver(getRobotModel(), groups[i], gravity_vector));
  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedPlanningGroup, this));
}

void MotionPlanningDisplay::updateStateExceptModified(robot_state::RobotState &dest, const robot_state::RobotState &src)
{
  robot_state::RobotState src_copy = src;
  for (std::set<std::string>::const_iterator it = modified_groups_.begin() ; it != modified_groups_.end() ; ++it)
  {
    const robot_model::JointModelGroup *jmg = dest.getJointModelGroup(*it);
    if (jmg)
    {
      std::vector<double> values_to_keep;
      dest.copyJointGroupPositions(jmg, values_to_keep);
      src_copy.setJointGroupPositions(jmg, values_to_keep);
    }
  }

  // overwrite the destination state
  dest = src_copy;
}

void MotionPlanningDisplay::onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  PlanningSceneDisplay::onSceneMonitorReceivedUpdate(update_type);
  robot_state::RobotState current_state = getPlanningSceneRO()->getCurrentState();
  std::string group = planning_group_property_->getStdString();

  if (query_start_state_property_->getBool() && !group.empty())
  {
    robot_state::RobotState start = *getQueryStartState();
    updateStateExceptModified(start, current_state);
    setQueryStartState(start);
  }

  if (query_goal_state_property_->getBool() && !group.empty())
  {
    robot_state::RobotState goal = *getQueryGoalState();
    updateStateExceptModified(goal, current_state);
    setQueryGoalState(goal);
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

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(displaying_trajectory_message_ && animating_path_);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
  {
    trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
    trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    trajectory_trail_[i]->setVisible(true);
  }

  text_to_display_->setVisible(false);

  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  frame_->enable();

  int_marker_display_->setEnabled(true);
  int_marker_display_->setFixedFrame(fixed_frame_);

  changedTrajectoryTopic(); // load topic at startup if default used
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void MotionPlanningDisplay::onDisable()
{
  if (robot_interaction_)
    robot_interaction_->clear();
  int_marker_display_->setEnabled(false);

  display_path_robot_->setVisible(false);
  for (std::size_t i = 0 ; i < trajectory_trail_.size() ; ++i)
    trajectory_trail_[i]->setVisible(false);
  displaying_trajectory_message_.reset();

  query_robot_start_->setVisible(false);
  query_robot_goal_->setVisible(false);
  frame_->disable();
  text_to_display_->setVisible(false);

  PlanningSceneDisplay::onDisable();
}

float MotionPlanningDisplay::getStateDisplayTime()
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
  if (int_marker_display_)
    int_marker_display_->update(wall_dt, ros_dt);
  if (frame_)
    frame_->updateSceneMarkers(wall_dt, ros_dt);

  PlanningSceneDisplay::update(wall_dt, ros_dt);
}

void MotionPlanningDisplay::updateInternal(float wall_dt, float ros_dt)
{
  PlanningSceneDisplay::updateInternal(wall_dt, ros_dt);

  if (!animating_path_ && !trajectory_message_to_display_ && loop_display_property_->getBool() && displaying_trajectory_message_)
  {
    animating_path_ = true;
    current_state_ = -1;
    current_state_time_ = std::numeric_limits<float>::infinity();
    display_path_robot_->setVisible(isEnabled());
  }

  if (!animating_path_ && trajectory_message_to_display_ && !trajectory_message_to_display_->empty())
  {
    planning_scene_monitor_->updateFrameTransforms();
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

  renderWorkspaceBox();
}

void MotionPlanningDisplay::load(const rviz::Config& config)
{
  PlanningSceneDisplay::load(config);
  if (frame_)
  {
    QString host;
    if (config.mapGetString("MoveIt_Warehouse_Host", &host))
      frame_->ui_->database_host->setText(host);
    int port;
    if (config.mapGetInt("MoveIt_Warehouse_Port", &port))
      frame_->ui_->database_port->setValue(port);
    float d;
    if (config.mapGetFloat("MoveIt_Planning_Time", &d))
      frame_->ui_->planning_time->setValue(d);
    if (config.mapGetFloat("MoveIt_Goal_Tolerance", &d))
      frame_->ui_->goal_tolerance->setValue(d);
    bool b;
    if (config.mapGetBool("MoveIt_Use_Constraint_Aware_IK", &b))
      frame_->ui_->collision_aware_ik->setChecked(b);
  }
}

void MotionPlanningDisplay::save(rviz::Config config) const
{
  PlanningSceneDisplay::save(config);
  if (frame_)
  {
    config.mapSetValue("MoveIt_Warehouse_Host", frame_->ui_->database_host->text());
    config.mapSetValue("MoveIt_Warehouse_Port", frame_->ui_->database_port->value());
    config.mapSetValue("MoveIt_Planning_Time", frame_->ui_->planning_time->value());
    config.mapSetValue("MoveIt_Goal_Tolerance", frame_->ui_->goal_tolerance->value());
    config.mapSetValue("MoveIt_Use_Constraint_Aware_IK", frame_->ui_->collision_aware_ik->isChecked());
  }
}

void MotionPlanningDisplay::incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  if (!planning_scene_monitor_)
  {
    return;
  }

  if (!msg->model_id.empty() && msg->model_id != getRobotModel()->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
             msg->model_id.c_str(), getRobotModel()->getName().c_str());

  trajectory_message_to_display_.reset();

  robot_trajectory::RobotTrajectoryPtr t(new robot_trajectory::RobotTrajectory(planning_scene_monitor_->getRobotModel(), ""));
  for (std::size_t i = 0 ; i < msg->trajectory.size() ; ++i)
  {
    if (t->empty())
    {
      const planning_scene_monitor::LockedPlanningSceneRO &ps = getPlanningSceneRO();
      t->setRobotTrajectoryMsg(ps->getCurrentState(), msg->trajectory_start, msg->trajectory[i]);
    }
    else
    {
      robot_trajectory::RobotTrajectory tmp(planning_scene_monitor_->getRobotModel(), "");
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

void MotionPlanningDisplay::fixedFrameChanged()
{
  PlanningSceneDisplay::fixedFrameChanged();
  if (int_marker_display_)
    int_marker_display_->setFixedFrame(fixed_frame_);
  changedPlanningGroup();
}


// Pick and place
void MotionPlanningDisplay::clearPlaceLocationsDisplay()
{
  for(std::size_t i=0; i < place_locations_display_.size(); ++i)
    place_locations_display_[i].reset();
  place_locations_display_.clear();
}

void MotionPlanningDisplay::visualizePlaceLocations(const std::vector<geometry_msgs::PoseStamped> &place_poses)
{
  clearPlaceLocationsDisplay();
  place_locations_display_.resize(place_poses.size());
  for(std::size_t i=0; i < place_poses.size(); ++i)
  {
    place_locations_display_[i].reset(new rviz::Shape(rviz::Shape::Sphere, context_->getSceneManager()));
    place_locations_display_[i]->setColor(1.0f, 0.0f, 0.0f, 0.3f);
    Ogre::Vector3 center(place_poses[i].pose.position.x,
                         place_poses[i].pose.position.y,
                         place_poses[i].pose.position.z);
    Ogre::Vector3 extents(0.02,0.02,0.02);
    place_locations_display_[i]->setScale(extents);
    place_locations_display_[i]->setPosition(center);
  }
}


} // namespace moveit_rviz_plugin
