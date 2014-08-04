/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2013, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Ioan Sucan, Adam Leeper */

#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_interaction/kinematic_options_map.h>
#include <moveit/transforms/transforms.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_interaction
{

static const float END_EFFECTOR_UNREACHABLE_COLOR[4] = { 1.0, 0.3, 0.3, 1.0 };
static const float END_EFFECTOR_REACHABLE_COLOR[4] = { 0.2, 1.0, 0.2, 1.0 };

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

RobotInteraction::RobotInteraction(const robot_model::RobotModelConstPtr &robot_model, const std::string &ns)
: robot_model_(robot_model)
, kinematic_options_map_(new KinematicOptionsMap)
{
  topic_ = ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC;
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(topic_);
  
  // spin a thread that will process feedback events
  run_processing_thread_ = true;
  processing_thread_.reset(new boost::thread(boost::bind(&RobotInteraction::processingThread, this)));
}

RobotInteraction::~RobotInteraction()
{
  run_processing_thread_ = false;
  new_feedback_condition_.notify_all();
  processing_thread_->join();

  clear();
  delete int_marker_server_;
}

void RobotInteraction::decideActiveComponents(const std::string &group)
{
  decideActiveComponents(group, InteractionStyle::SIX_DOF);
}

void RobotInteraction::decideActiveComponents(const std::string &group, InteractionStyle::InteractionStyle style)
{
  decideActiveEndEffectors(group, style);
  decideActiveJoints(group);
  if (active_eef_.empty() && active_vj_.empty() && active_generic_.empty())
    ROS_INFO_NAMED("robot_interaction",
                   "No active joints or end effectors found for group '%s'. "
                   "Make sure you have defined an end effector in your SRDF file and that "
                   "kinematics.yaml is loaded in this node's namespace.",
                   group.c_str());
}

void RobotInteraction::addActiveComponent(const InteractiveMarkerConstructorFn &construct,
                                          const ProcessFeedbackFn &process,
                                          const InteractiveMarkerUpdateFn &update,
                                          const std::string &name)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  GenericInteraction g;
  g.construct_marker = construct;
  g.update_pose = update;
  g.process_feedback = process;
  // compute the suffix that will be added to the generated markers
  g.marker_name_suffix = "_" + name + "_" + boost::lexical_cast<std::string>(active_generic_.size());
  active_generic_.push_back(g);
}

double RobotInteraction::computeGroupMarkerSize(const std::string &group)
{
  static const double DEFAULT_SCALE = 0.25;
  if (group.empty())
    return DEFAULT_SCALE;
  const robot_model::JointModelGroup *jmg = robot_model_->getJointModelGroup(group);
  if (!jmg)
    return 0.0;

  const std::vector<std::string> &links = jmg->getLinkModelNames();
  if (links.empty())
    return DEFAULT_SCALE;

  // compute the aabb of the links that make up the group
  const double inf = std::numeric_limits<double>::infinity();
  Eigen::Vector3d lo( inf,  inf,  inf);
  Eigen::Vector3d hi(-inf, -inf, -inf);
  robot_state::RobotState default_state(robot_model_);
  default_state.setToDefaultValues();

  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    const robot_model::LinkModel *lm = default_state.getLinkModel(links[i]);
    if (!lm)
      continue;
    const Eigen::Vector3d &ext = lm->getShapeExtentsAtOrigin();

    Eigen::Vector3d corner1 = ext/2.0;
    corner1 = default_state.getGlobalLinkTransform(lm) * corner1;
    Eigen::Vector3d corner2 = ext/-2.0;
    corner2 = default_state.getGlobalLinkTransform(lm) * corner2;
    lo = lo.cwiseMin(corner1);
    hi = hi.cwiseMax(corner2);
  }

  // slightly bigger than the size of the largest end effector dimension
  double s = std::max(std::max(hi.x() - lo.x(), hi.y() - lo.y()), hi.z() - lo.z());
  s *= 1.73205081; // sqrt(3)

  // if the scale is less than 5cm, set it to default
  if (s < 0.05)
    s = DEFAULT_SCALE;
  return s;
}

void RobotInteraction::decideActiveJoints(const std::string &group)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_vj_.clear();

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active joints for group '%s'", group.c_str());

  if (group.empty())
    return;

  const boost::shared_ptr<const srdf::Model> &srdf = robot_model_->getSRDF();
  const robot_model::JointModelGroup *jmg = robot_model_->getJointModelGroup(group);

  if (!jmg || !srdf)
    return;

  std::set<std::string> used;
  if (jmg->hasJointModel(robot_model_->getRootJointName()))
  {
    robot_state::RobotState default_state(robot_model_);
    default_state.setToDefaultValues();
    std::vector<double> aabb;
    default_state.computeAABB(aabb);

    const std::vector<srdf::Model::VirtualJoint> &vj = srdf->getVirtualJoints();
    for (std::size_t i = 0 ; i < vj.size() ; ++i)
      if (vj[i].name_ == robot_model_->getRootJointName())
      {
        if (vj[i].type_ == "planar" || vj[i].type_ == "floating")
        {
          JointInteraction v;
          v.connecting_link = vj[i].child_link_;
          v.parent_frame = vj[i].parent_frame_;
          if (!v.parent_frame.empty() && v.parent_frame[0] == '/')
            v.parent_frame = v.parent_frame.substr(1);
          v.joint_name = vj[i].name_;
          if (vj[i].type_ == "planar")
            v.dof = 3;
          else
            v.dof = 6;
          // take the max of the X, Y, Z extent
          v.size = std::max(std::max(aabb[1] - aabb[0], aabb[3] - aabb[2]), aabb[5] - aabb[4]);
          active_vj_.push_back(v);
          used.insert(v.joint_name);
        }
      }
  }

  const std::vector<const robot_model::JointModel*> &joints = jmg->getJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    if ((joints[i]->getType() == robot_model::JointModel::PLANAR ||
     joints[i]->getType() == robot_model::JointModel::FLOATING) &&
        used.find(joints[i]->getName()) == used.end())
    {
      JointInteraction v;
      v.connecting_link = joints[i]->getChildLinkModel()->getName();
      if (joints[i]->getParentLinkModel())
        v.parent_frame = joints[i]->getParentLinkModel()->getName();
      v.joint_name = joints[i]->getName();
      if (joints[i]->getType() == robot_model::JointModel::PLANAR)
        v.dof = 3;
      else
        v.dof = 6;
      // take the max of the X, Y, Z extent
      v.size = computeGroupMarkerSize(group);
      active_vj_.push_back(v);
    }
  }
}

void RobotInteraction::decideActiveEndEffectors(const std::string &group)
{
  decideActiveEndEffectors(group, InteractionStyle::SIX_DOF);
}

void RobotInteraction::decideActiveEndEffectors(const std::string &group, InteractionStyle::InteractionStyle style)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_eef_.clear();

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active end-effectors for group '%s'", group.c_str());

  if (group.empty())
    return;

  const boost::shared_ptr<const srdf::Model> &srdf = robot_model_->getSRDF();
  const robot_model::JointModelGroup *jmg = robot_model_->getJointModelGroup(group);

  if (!jmg || !srdf)
  {
    ROS_WARN_NAMED("robot_interaction", "Unable to decide active end effector: no joint model group or no srdf model");
    return;
  }

  const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
  const std::pair<robot_model::JointModelGroup::KinematicsSolver,
                  robot_model::JointModelGroup::KinematicsSolverMap> &smap = jmg->getGroupKinematics();

  // if we have an IK solver for the selected group, we check if there are any end effectors attached to this group
  if (smap.first)
  {
    if (eef.empty() && !jmg->getLinkModelNames().empty())
    {
      // No end effectors.  Use last link in group as the "end effector".
      EndEffectorInteraction ee;
      ee.parent_group = group;
      ee.parent_link = jmg->getLinkModelNames().back();
      ee.eef_group = group;
      ee.interaction = style;
      active_eef_.push_back(ee);
    }
    else
    {
      for (std::size_t i = 0 ; i < eef.size() ; ++i)
        if ((jmg->hasLinkModel(eef[i].parent_link_) ||
             jmg->getName() == eef[i].parent_group_) &&
            jmg->canSetStateFromIK(eef[i].parent_link_))
        {
          // We found an end-effector whose parent is the group.
          EndEffectorInteraction ee;
          ee.parent_group = group;
          ee.parent_link = eef[i].parent_link_;
          ee.eef_group = eef[i].component_group_;
          ee.interaction = style;
          active_eef_.push_back(ee);
        }
    }
  }
  else if (!smap.second.empty())
  {
    for (robot_model::JointModelGroup::KinematicsSolverMap::const_iterator it = smap.second.begin() ;
         it != smap.second.end() ;
         ++it)
    {
      for (std::size_t i = 0 ; i < eef.size() ; ++i)
      {
        if ((it->first->hasLinkModel(eef[i].parent_link_) ||
             jmg->getName() == eef[i].parent_group_) &&
            it->first->canSetStateFromIK(eef[i].parent_link_))
        {
          // We found an end-effector whose parent is a subgroup of the group.  (May be more than one)
          EndEffectorInteraction ee;
          ee.parent_group = it->first->getName();
          ee.parent_link = eef[i].parent_link_;
          ee.eef_group = eef[i].component_group_;
          ee.interaction = style;
          active_eef_.push_back(ee);
          break;
        }
      }
    }
  }

  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    // if we have a separate group for the eef, we compute the scale based on
    // it; otherwise, we use a default scale
    active_eef_[i].size = active_eef_[i].eef_group == active_eef_[i].parent_group ?
                            computeGroupMarkerSize("") :
                            computeGroupMarkerSize(active_eef_[i].eef_group);
    ROS_DEBUG_NAMED("robot_interaction",
                    "Found active end-effector '%s', of scale %lf",
                    active_eef_[i].eef_group.c_str(),
                    active_eef_[i].size);
  }
}

void RobotInteraction::clear()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_eef_.clear();
  active_vj_.clear();
  active_generic_.clear();
  clearInteractiveMarkersUnsafe();
  publishInteractiveMarkers();
}

void RobotInteraction::clearInteractiveMarkers()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  clearInteractiveMarkersUnsafe();
}

void RobotInteraction::clearInteractiveMarkersUnsafe()
{
  handlers_.clear();
  shown_markers_.clear();
  int_marker_move_subscribers_.clear();
  int_maker_move_topics_.clear();
  int_marker_names_.clear();
  int_marker_server_->clear();
}

void RobotInteraction::addEndEffectorMarkers(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const EndEffectorInteraction& eef,
      visualization_msgs::InteractiveMarker& im,
      bool position,
      bool orientation)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  addEndEffectorMarkers(handler, eef, pose, im, position, orientation);
}

void RobotInteraction::addEndEffectorMarkers(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const EndEffectorInteraction& eef,
      const geometry_msgs::Pose& im_to_eef,
      visualization_msgs::InteractiveMarker& im,
      bool position,
      bool orientation)
{
  if (eef.parent_group == eef.eef_group || !robot_model_->hasLinkModel(eef.parent_link))
    return;

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = false;
  if (position && orientation)
    m_control.interaction_mode = m_control.MOVE_ROTATE_3D;
  else if (orientation)
    m_control.interaction_mode = m_control.ROTATE_3D;
  else
    m_control.interaction_mode = m_control.MOVE_3D;

  std_msgs::ColorRGBA marker_color;
  const float *color = handler->inError(eef) ? END_EFFECTOR_UNREACHABLE_COLOR : END_EFFECTOR_REACHABLE_COLOR;
  marker_color.r = color[0];
  marker_color.g = color[1];
  marker_color.b = color[2];
  marker_color.a = color[3];

  robot_state::RobotStateConstPtr rstate = handler->getState();
  const std::vector<std::string> &link_names = rstate->getJointModelGroup(eef.eef_group)->getLinkModelNames();
  visualization_msgs::MarkerArray marker_array;
  rstate->getRobotMarkers(marker_array, link_names, marker_color, eef.eef_group, ros::Duration());
  tf::Pose tf_root_to_link;
  tf::poseEigenToTF(rstate->getGlobalLinkTransform(eef.parent_link), tf_root_to_link);
  // Release the ptr count on the kinematic state
  rstate.reset();

  for (std::size_t i = 0 ; i < marker_array.markers.size() ; ++i)
  {
    marker_array.markers[i].header = im.header;
    marker_array.markers[i].mesh_use_embedded_materials = true;
    // - - - - - - Do some math for the offset - - - - - -
    tf::Pose tf_root_to_im, tf_root_to_mesh, tf_im_to_eef;
    tf::poseMsgToTF(im.pose, tf_root_to_im);
    tf::poseMsgToTF(marker_array.markers[i].pose, tf_root_to_mesh);
    tf::poseMsgToTF(im_to_eef, tf_im_to_eef);
    tf::Pose tf_eef_to_mesh = tf_root_to_link.inverse() * tf_root_to_mesh;
    tf::Pose tf_im_to_mesh = tf_im_to_eef * tf_eef_to_mesh;
    tf::Pose tf_root_to_mesh_new = tf_root_to_im * tf_im_to_mesh;
    tf::poseTFToMsg(tf_root_to_mesh_new, marker_array.markers[i].pose);
    // - - - - - - - - - - - - - - - - - - - - - - - - - -
    m_control.markers.push_back(marker_array.markers[i]);
  }

  im.controls.push_back(m_control);
}

static inline std::string getMarkerName(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const EndEffectorInteraction &eef)
{
  return "EE:" + handler->getName() + "_" + eef.parent_link;
}

static inline std::string getMarkerName(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const JointInteraction &vj)
{
  return "JJ:" + handler->getName() + "_" + vj.connecting_link;
}

static inline std::string getMarkerName(
      const ::robot_interaction::InteractionHandlerPtr &handler, const GenericInteraction &g)
{
  return "GG:" + handler->getName() + "_" + g.marker_name_suffix;
}

void RobotInteraction::addInteractiveMarkers(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const double marker_scale)
{
  handler->setRobotInteraction(this);
  // If scale is left at default size of 0, scale will be based on end effector link size. a good value is between 0-1
  std::vector<visualization_msgs::InteractiveMarker> ims;
  ros::NodeHandle nh;
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    robot_state::RobotStateConstPtr s = handler->getState();
    
    for (std::size_t i = 0 ; i < active_generic_.size() ; ++i)
    {
      visualization_msgs::InteractiveMarker im;
      if (active_generic_[i].construct_marker(*s, im))
      {
        im.name = getMarkerName(handler, active_generic_[i]);
        shown_markers_[im.name] = i;
        ims.push_back(im);
        ROS_DEBUG_NAMED("robot_interaction",
                        "Publishing interactive marker %s (size = %lf)",
                        im.name.c_str(),
                        im.scale);
      }
    }
    ros::NodeHandle nh;
    
    for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
    {
      
      geometry_msgs::PoseStamped pose;
      geometry_msgs::Pose control_to_eef_tf;
      pose.header.frame_id = robot_model_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      computeMarkerPose(handler, active_eef_[i], *s, pose.pose, control_to_eef_tf);

      std::string marker_name = getMarkerName(handler, active_eef_[i]);
      shown_markers_[marker_name] = i;

      // Determine interactive maker size
      double mscale = marker_scale < std::numeric_limits<double>::epsilon() ? active_eef_[i].size : marker_scale;

      visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, mscale);
      if (handler && handler->getControlsVisible())
      {
        if (active_eef_[i].interaction & EEF_POSITION_ARROWS)
          addPositionControl(im, active_eef_[i].interaction & EEF_FIXED);
        if (active_eef_[i].interaction & EEF_ORIENTATION_CIRCLES)
          addOrientationControl(im, active_eef_[i].interaction & EEF_FIXED);
        if (active_eef_[i].interaction & (EEF_POSITION_SPHERE|EEF_ORIENTATION_SPHERE))
        {
          std_msgs::ColorRGBA color;
          color.r = 0;
          color.g = 1;
          color.b = 1;
          color.a = 0.5;
          addViewPlaneControl(im,
                              mscale * 0.25,
                              color,
                              active_eef_[i].interaction&EEF_POSITION_SPHERE,
                              active_eef_[i].interaction&EEF_ORIENTATION_SPHERE);
        }
      }
      if (handler &&
          handler->getMeshesVisible() &&
          (active_eef_[i].interaction & (EEF_POSITION_EEF|EEF_ORIENTATION_EEF)))
        addEndEffectorMarkers(handler,
                              active_eef_[i],
                              control_to_eef_tf,
                              im,
                              active_eef_[i].interaction & EEF_POSITION_EEF,
                              active_eef_[i].interaction & EEF_ORIENTATION_EEF);
      ims.push_back(im);
      registerMoveInteractiveMarkerTopic(
        marker_name,
        handler->getName() + "_" + active_eef_[i].parent_link);
      ROS_DEBUG_NAMED("robot_interaction",
                      "Publishing interactive marker %s (size = %lf)",
                      marker_name.c_str(),
                      mscale);
    }
    for (std::size_t i = 0 ; i < active_vj_.size() ; ++i)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = robot_model_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      tf::poseEigenToMsg(s->getGlobalLinkTransform(active_vj_[i].connecting_link), pose.pose);
      std::string marker_name = getMarkerName(handler, active_vj_[i]);
      shown_markers_[marker_name] = i;

      // Determine interactive maker size
      double mscale = marker_scale < std::numeric_limits<double>::epsilon() ? active_vj_[i].size : marker_scale;

      visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, mscale);
      if (handler && handler->getControlsVisible())
      {
        if (active_vj_[i].dof == 3) // planar joint
          addPlanarXYControl(im, false);
        else
          add6DOFControl(im, false);
      }
      ims.push_back(im);
      registerMoveInteractiveMarkerTopic(marker_name,  handler->getName() + "_" + active_vj_[i].connecting_link);
      ROS_DEBUG_NAMED("robot_interaction",
                      "Publishing interactive marker %s (size = %lf)",
                      marker_name.c_str(),
                      mscale);
    }
    handlers_[handler->getName()] = handler;
  }

  // we do this while marker_access_lock_ is unlocked because the interactive marker server locks
  // for most function calls, and maintains that lock while the feedback callback is running
  // that can cause a deadlock if we were to run the loop below while marker_access_lock_ is locked
  for (std::size_t i = 0 ; i < ims.size() ; ++i)
  {
    int_marker_server_->insert(ims[i]);
    int_marker_server_->setCallback(ims[i].name,
                                    boost::bind(&RobotInteraction::processInteractiveMarkerFeedback,
                                                this,
                                                _1));

    // Add menu handler to all markers that this interaction handler creates.
    if (boost::shared_ptr<interactive_markers::MenuHandler> mh = handler->getMenuHandler())
      mh->apply(*int_marker_server_, ims[i].name);
  }
}

void RobotInteraction::registerMoveInteractiveMarkerTopic(
  const std::string marker_name, const std::string& name)
{
  ros::NodeHandle nh;
  std::stringstream ss;
  ss << "/rviz/moveit/move_marker/";
  ss << name;
  int_maker_move_topics_.push_back(ss.str());
  int_marker_names_.push_back(marker_name);
}

void RobotInteraction::toggleMoveInteractiveMarkerTopic(bool enable)
{
  if (enable) {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    if (int_marker_move_subscribers_.size() != 0) {
      
    }
    else {
      ros::NodeHandle nh;
      for (size_t i = 0; i < int_maker_move_topics_.size(); i++) {
        std::string topic_name = int_maker_move_topics_[i];
        std::string marker_name = int_marker_names_[i];
        int_marker_move_subscribers_.push_back(
          nh.subscribe<geometry_msgs::PoseStamped>
          (topic_name, 1, boost::bind(&RobotInteraction::moveInteractiveMarker,
                                      this, marker_name, _1)));
      }
    }
  }
  else {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    int_marker_move_subscribers_.clear();
  }
}
  
void RobotInteraction::computeMarkerPose(
      const ::robot_interaction::InteractionHandlerPtr &handler,
      const EndEffectorInteraction &eef,
      const robot_state::RobotState &robot_state,
      geometry_msgs::Pose &pose,
      geometry_msgs::Pose &control_to_eef_tf) const
{
  // Need to allow for control pose offsets
  tf::Transform tf_root_to_link, tf_root_to_control;
  tf::poseEigenToTF(robot_state.getGlobalLinkTransform(eef.parent_link), tf_root_to_link);

  geometry_msgs::Pose msg_link_to_control;
  if (handler->getPoseOffset(eef, msg_link_to_control))
  {
    tf::Transform tf_link_to_control;
    tf::poseMsgToTF(msg_link_to_control, tf_link_to_control);

    tf_root_to_control = tf_root_to_link * tf_link_to_control;
    tf::poseTFToMsg(tf_link_to_control.inverse(), control_to_eef_tf);
  }
  else
  {
    tf_root_to_control = tf_root_to_link;
    control_to_eef_tf.orientation.x = 0.0;
    control_to_eef_tf.orientation.y = 0.0;
    control_to_eef_tf.orientation.z = 0.0;
    control_to_eef_tf.orientation.w = 1.0;
  }

  tf::poseTFToMsg(tf_root_to_control, pose);
}

void RobotInteraction::updateInteractiveMarkers(const ::robot_interaction::InteractionHandlerPtr &handler)
{
  handler->setRobotInteraction(this);
  std::map<std::string, geometry_msgs::Pose> pose_updates;
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

    robot_state::RobotStateConstPtr s = handler->getState();
    for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
    {
      std::string marker_name = getMarkerName(handler, active_eef_[i]);
      geometry_msgs::Pose control_to_eef_tf;
      computeMarkerPose(handler, active_eef_[i], *s, pose_updates[marker_name], control_to_eef_tf);
    }

    for (std::size_t i = 0 ; i < active_vj_.size() ; ++i)
    {
      std::string marker_name = getMarkerName(handler, active_vj_[i]);
      tf::poseEigenToMsg(s->getGlobalLinkTransform(active_vj_[i].connecting_link), pose_updates[marker_name]);
    }

    for (std::size_t i = 0 ; i < active_generic_.size() ; ++i)
    {
      std::string marker_name = getMarkerName(handler, active_generic_[i]);
      geometry_msgs::Pose pose;
      if (active_generic_[i].update_pose && active_generic_[i].update_pose(*s, pose))
        pose_updates[marker_name] = pose;
    }
  }
  for (std::map<std::string, geometry_msgs::Pose>::const_iterator it = pose_updates.begin() ;
       it != pose_updates.end() ;
       ++it)
    int_marker_server_->setPose(it->first, it->second);
  int_marker_server_->applyChanges();
}

void RobotInteraction::publishInteractiveMarkers()
{
  // the server locks internally, so we need not worry about locking
  int_marker_server_->applyChanges();
}

bool RobotInteraction::showingMarkers(const ::robot_interaction::InteractionHandlerPtr &handler)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
    if (shown_markers_.find(getMarkerName(handler, active_eef_[i])) == shown_markers_.end())
      return false;
  for (std::size_t i = 0 ; i < active_vj_.size() ; ++i)
    if (shown_markers_.find(getMarkerName(handler, active_vj_[i])) == shown_markers_.end())
      return false;
  for (std::size_t i = 0 ; i < active_generic_.size() ; ++i)
    if (shown_markers_.find(getMarkerName(handler, active_generic_[i])) == shown_markers_.end())
      return false;
  return true;
}

// TODO: can we get rid of this?  Only used in moveit_ros/benchmarks_gui/src/tab_states_and_goals.cpp right now.
bool RobotInteraction::updateState(
      robot_state::RobotState &state,
      const EndEffectorInteraction &eef,
      const geometry_msgs::Pose &pose,
      unsigned int attempts,
      double ik_timeout,
      const robot_state::GroupStateValidityCallbackFn &validity_callback,
      const kinematics::KinematicsQueryOptions &kinematics_query_options)
{
  if (state.setFromIK(state.getJointModelGroup(eef.parent_group), pose, eef.parent_link,
                      kinematics_query_options.lock_redundant_joints ? 1 : attempts,
                      ik_timeout, validity_callback, kinematics_query_options))
  {
    state.update();
    return true;
  }
  return false;
}

void RobotInteraction::moveInteractiveMarker(const std::string name, const geometry_msgs::PoseStampedConstPtr& msg)
{
  {
    
    std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(name);
    if (it != shown_markers_.end())
    {
      visualization_msgs::InteractiveMarkerFeedback::Ptr feedback (new visualization_msgs::InteractiveMarkerFeedback);
      feedback->header = msg->header;
      feedback->marker_name = name;
      feedback->pose = msg->pose;
      feedback->event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
      processInteractiveMarkerFeedback(feedback);
      {
        boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
        int_marker_server_->setPose(name, msg->pose, msg->header); // move the interactive marker
        int_marker_server_->applyChanges();
      }
    }
    else
    {
      return;
    }
  }
  
//  }
}

void RobotInteraction::processInteractiveMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // perform some validity checks
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
  {
    ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class)", feedback->marker_name.c_str());
    return;
  }

  std::size_t u = feedback->marker_name.find_first_of("_");
  if (u == std::string::npos || u < 4)
  {
    ROS_ERROR("Invalid marker name: '%s'",  feedback->marker_name.c_str());
    return;
  }

  feedback_map_[feedback->marker_name] = feedback;
  new_feedback_condition_.notify_all();
}

void RobotInteraction::processingThread()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

  while (run_processing_thread_ && ros::ok())
  {
    while (feedback_map_.empty() && run_processing_thread_ && ros::ok())
      new_feedback_condition_.wait(ulock);

    while (!feedback_map_.empty() && ros::ok())
    {
      visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback = feedback_map_.begin()->second;
      feedback_map_.erase(feedback_map_.begin());
      ROS_DEBUG_NAMED("robot_interaction",
                      "Processing feedback from map for marker [%s]",
                      feedback->marker_name.c_str());

      std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
      if (it == shown_markers_.end())
      {
        ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class) "
                  "(should never have ended up in the feedback_map!)",
                  feedback->marker_name.c_str());
        continue;
      }
      std::size_t u = feedback->marker_name.find_first_of("_");
      if (u == std::string::npos || u < 4)
      {
        ROS_ERROR("Invalid marker name: '%s' (should never have ended up in the feedback_map!)",
                  feedback->marker_name.c_str());
        continue;
      }
      std::string marker_class = feedback->marker_name.substr(0, 2);
      std::string handler_name = feedback->marker_name.substr(3, u - 3); // skip the ":"
      std::map<std::string, ::robot_interaction::InteractionHandlerPtr>::const_iterator jt =
                                                                handlers_.find(handler_name);
      if (jt == handlers_.end())
      {
        ROS_ERROR("Interactive Marker Handler '%s' is not known.", handler_name.c_str());
        continue;
      }

      // we put this in a try-catch because user specified callbacks may be triggered
      try
      {
        if (marker_class == "EE")
        {
          // make a copy of the data, so we do not lose it while we are unlocked
          EndEffectorInteraction eef = active_eef_[it->second];
          ::robot_interaction::InteractionHandlerPtr ih = jt->second;
          marker_access_lock_.unlock();
          try
          {
            ih->handleEndEffector(eef, feedback);
          }
          catch(std::runtime_error &ex)
          {
            ROS_ERROR("Exception caught while handling end-effector update: %s", ex.what());
          }
          catch(...)
          {
            ROS_ERROR("Exception caught while handling end-effector update");
          }
          marker_access_lock_.lock();
        }
        else
          if (marker_class == "JJ")
          {
            // make a copy of the data, so we do not lose it while we are unlocked
            JointInteraction vj = active_vj_[it->second];
            ::robot_interaction::InteractionHandlerPtr ih = jt->second;
            marker_access_lock_.unlock();
            try
            {
              ih->handleJoint(vj, feedback);
            }
            catch(std::runtime_error &ex)
            {
              ROS_ERROR("Exception caught while handling joint update: %s", ex.what());
            }
            catch(...)
            {
              ROS_ERROR("Exception caught while handling joint update");
            }
            marker_access_lock_.lock();
          }
          else
            if (marker_class == "GG")
            {
              ::robot_interaction::InteractionHandlerPtr ih = jt->second;
              GenericInteraction g = active_generic_[it->second];
              marker_access_lock_.unlock();
              try
              {
                ih->handleGeneric(g, feedback);
              }
              catch(std::runtime_error &ex)
              {
                ROS_ERROR("Exception caught while handling joint update: %s", ex.what());
              }
              catch(...)
              {
                ROS_ERROR("Exception caught while handling joint update");
              }
              marker_access_lock_.lock();
            }
            else
              ROS_ERROR("Unknown marker class ('%s') for marker '%s'",
                        marker_class.c_str(),
                        feedback->marker_name.c_str());
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("Exception caught while processing event: %s", ex.what());
      }
      catch (...)
      {
        ROS_ERROR("Exception caught while processing event");
      }
    }
  }
}

// DEPRECATED FUNCTIONALITY for backwards compatibility
void RobotInteraction::decideActiveComponents(const std::string &group, EndEffectorInteractionStyle style)
{
  decideActiveComponents(group, (InteractionStyle::InteractionStyle)(int)style);
}

// DEPRECATED FUNCTIONALITY for backwards compatibility
void RobotInteraction::decideActiveEndEffectors(const std::string &group, EndEffectorInteractionStyle style)
{
  decideActiveEndEffectors(group, (InteractionStyle::InteractionStyle)(int)style);
}

}
