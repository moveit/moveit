/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "moveit/robot_interaction/robot_interaction.h"
#include "moveit/robot_interaction/interactive_marker_helpers.h"
#include <planning_models/transforms.h>
#include <interactive_markers/interactive_marker_server.h>
#include <boost/lexical_cast.hpp>
#include <limits>

namespace robot_interaction
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

RobotInteraction::RobotInteraction(const planning_models::KinematicModelConstPtr &kmodel,
                                   const InteractionHandlerPtr &handler) :
  kmodel_(kmodel), handler_(handler)
{  
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(INTERACTIVE_MARKER_TOPIC);
}

RobotInteraction::~RobotInteraction(void)
{
  clear();
  delete int_marker_server_;
}

void RobotInteraction::decideActiveComponents(const std::string &group)
{
  decideActiveEndEffectors(group);
  decideActiveVirtualJoints(group);
}

double RobotInteraction::computeGroupScale(const std::string &group)
{
  static const double DEFAULT_SCALE = 0.2;
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  if (!jmg)
    return 0.0;
  
  const std::vector<std::string> &links = jmg->getLinkModelNames();
  if (links.empty())
    return DEFAULT_SCALE;
  
  std::vector<double> scale(3, 0.0);
  std::vector<double> low(3, std::numeric_limits<double>::infinity());
  std::vector<double> hi(3, -std::numeric_limits<double>::infinity());
  planning_models::KinematicState default_state(kmodel_);
  default_state.setToDefaultValues();
  
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    planning_models::KinematicState::LinkState *ls = default_state.getLinkState(links[i]);
    if (!ls)
      continue;
    const Eigen::Vector3d &ext = ls->getLinkModel()->getShapeExtentsAtOrigin();
    
    Eigen::Vector3d corner1 = ext/2.0;
    corner1 = ls->getGlobalLinkTransform() * corner1;
    Eigen::Vector3d corner2 = ext/-2.0;
    corner2 = ls->getGlobalLinkTransform() * corner2;
    for (int k = 0 ; k < 3 ; ++k)
    {
      if (low[k] > corner2[k])
        low[k] = corner2[k];
      if (hi[k] < corner1[k])
        hi[k] = corner1[k];
    }
  }
  
  for (int i = 0 ; i < 3 ; ++i)
    scale[i] = hi[i] - low[i];

  static const double sqrt_3 = 1.732050808;
  double s = std::max(std::max(scale[0], scale[1]), scale[2]) * sqrt_3;
  
  // if the scale is less than 1mm, set it to default
  if (s < 1e-3)
    s = DEFAULT_SCALE;  
  return s;
}

void RobotInteraction::decideActiveVirtualJoints(const std::string &group)
{ 
  active_vj_.clear();
  
  if (group.empty())
    return;
  
  const boost::shared_ptr<const srdf::Model> &srdf = kmodel_->getSRDF();
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  
  if (!jmg || !srdf)
    return;
  
  if (!jmg->hasJointModel(kmodel_->getRootJointName()))
    return;

  planning_models::KinematicState default_state(kmodel_);
  default_state.setToDefaultValues();
  std::vector<double> aabb;
  default_state.computeAABB(aabb);
  
  const std::vector<srdf::Model::VirtualJoint> &vj = srdf->getVirtualJoints();
  for (std::size_t i = 0 ; i < vj.size() ; ++i)
    if (vj[i].name_ == kmodel_->getRootJointName())
    {
      if (vj[i].type_ == "planar" || vj[i].type_ == "floating")
      {
        VirtualJoint v;
        v.connecting_link = vj[i].child_link_;
        v.joint_name = vj[i].name_;
        if (vj[i].type_ == "planar")
          v.dof = 3;
        else
          v.dof = 6;
        // take the max of the X extent and the Y extent
        v.scale = std::max(aabb[1] - aabb[0], aabb[3] - aabb[2]);
        active_vj_.push_back(v);
      }
    }
}

void RobotInteraction::decideActiveEndEffectors(const std::string &group)
{
  active_eef_.clear();

  if (group.empty())
    return;
  
  const boost::shared_ptr<const srdf::Model> &srdf = kmodel_->getSRDF();
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  
  if (!jmg || !srdf)
    return;

  const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
  const std::pair<planning_models::KinematicModel::SolverAllocatorFn, planning_models::KinematicModel::SolverAllocatorMapFn> &smap = jmg->getSolverAllocators();
  
  // if we have an IK solver for the selected group, we check if there are any end effectors attached to this group
  if (smap.first)
  {
    for (std::size_t i = 0 ; i < eef.size() ; ++i)
      if (jmg->hasLinkModel(eef[i].parent_link_) && jmg->canSetStateFromIK(eef[i].parent_link_))
      {
        // we found an end-effector for the selected group
        EndEffector ee;
        ee.group = group;
        ee.tip_link = eef[i].parent_link_;
        ee.eef_group = eef[i].component_group_;
        active_eef_.push_back(ee);
        break;
      }
  }
  else
    if (!smap.second.empty())
    {
      for (std::map<const planning_models::KinematicModel::JointModelGroup*, planning_models::KinematicModel::SolverAllocatorFn>::const_iterator it = smap.second.begin() ; 
           it != smap.second.end() ; ++it)
      {
        for (std::size_t i = 0 ; i < eef.size() ; ++i)
          if (it->first->hasLinkModel(eef[i].parent_link_) && it->first->canSetStateFromIK(eef[i].parent_link_))
          {
            // we found an end-effector for the selected group;
            EndEffector ee;
            ee.group = it->first->getName();
            ee.tip_link = eef[i].parent_link_;
            ee.eef_group = eef[i].component_group_;
            active_eef_.push_back(ee);
            break;
          }
      }
    }
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    active_eef_[i].scale = computeGroupScale(active_eef_[i].eef_group);
    ROS_DEBUG("Found active end-effector '%s', of scale %lf", active_eef_[i].eef_group.c_str(), active_eef_[i].scale);
  }
}

void RobotInteraction::clear(void)
{
  active_eef_.clear();
  active_vj_.clear();
  clearInteractiveMarkers();
  publishInteractiveMarkers();
}

void RobotInteraction::clearInteractiveMarkers(void)
{
  shown_markers_.clear();
  int_marker_server_->clear();
}

void RobotInteraction::addInteractiveMarkers(const planning_models::KinematicState &state, int id)
{ 
  //  ros::WallTime start = ros::WallTime::now();
  
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = kmodel_->getModelFrame();
    pose.header.stamp = ros::Time::now();
    
    const planning_models::KinematicState::LinkState *ls = state.getLinkState(active_eef_[i].tip_link);
    planning_models::msgFromPose(ls->getGlobalLinkTransform(), pose.pose);
    
    std::string marker_name = "IK" + boost::lexical_cast<std::string>(id) + "_" + active_eef_[i].tip_link;
    shown_markers_[marker_name] = i;
    visualization_msgs::InteractiveMarker im = make6DOFMarker(marker_name, pose, active_eef_[i].scale);
    if (handler_ && handler_->inError(active_eef_[i], id))
      addErrorMarker(im);
    int_marker_server_->insert(im);
    int_marker_server_->setCallback(im.name, boost::bind(&RobotInteraction::processInteractiveMarkerFeedback, this, _1));
    ROS_DEBUG("Publishing interactive marker %s (scale = %lf)", marker_name.c_str(), active_eef_[i].scale);
  }
  
  for (std::size_t i = 0 ; i < active_vj_.size() ; ++i)
    if (active_vj_[i].dof == 3)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = kmodel_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      
      const planning_models::KinematicState::LinkState *ls = state.getLinkState(active_vj_[i].connecting_link);
      planning_models::msgFromPose(ls->getGlobalLinkTransform(), pose.pose);
      
      std::string marker_name = "XY" + boost::lexical_cast<std::string>(id) + "_" + active_vj_[i].connecting_link;
      shown_markers_[marker_name] = i;
      visualization_msgs::InteractiveMarker im = make3DOFMarker(marker_name, pose, active_vj_[i].scale);
      
      int_marker_server_->insert(im);
      int_marker_server_->setCallback(im.name, boost::bind(&RobotInteraction::processInteractiveMarkerFeedback, this, _1));
      ROS_DEBUG("Publishing interactive marker %s (scale = %lf)", marker_name.c_str(), active_vj_[i].scale);
    }
  
  //  ROS_INFO("Spent %0.5lf s to publish markers", (ros::WallTime::now() - start).toSec());
}

void RobotInteraction::publishInteractiveMarkers(void)
{
  int_marker_server_->applyChanges();
}

bool RobotInteraction::updateState(planning_models::KinematicState &state, const VirtualJoint &vj, const geometry_msgs::Pose &pose)
{
  Eigen::Quaterniond q;
  if (!planning_models::quatFromMsg(pose.orientation, q))
    return false;
  std::map<std::string, double> vals;
  vals[ vj.joint_name + "/x"] = pose.position.x;
  vals[ vj.joint_name + "/y"] = pose.position.y;
  Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);
  vals[ vj.joint_name + "/theta"] = xyz[2];
  state.getJointState(vj.joint_name)->setVariableValues(vals);
  state.updateLinkTransforms();  
  return true;
}

bool RobotInteraction::updateState(planning_models::KinematicState &state, const EndEffector &eef, const geometry_msgs::Pose &pose)
{ 
  static const double IK_TIMEOUT = 0.1;
  return state.getJointStateGroup(eef.group)->setFromIK(pose, eef.tip_link, IK_TIMEOUT);
}

void RobotInteraction::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (!handler_)
    return;
  
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
  {
    ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class)", feedback->marker_name.c_str());
    return;
  }
  
  std::size_t u = feedback->marker_name.find_first_of("_");
  if (u == std::string::npos || u < 3)
  {
    ROS_ERROR("Invalid marker name: '%s'",  feedback->marker_name.c_str());
    return;
  }
  
  std::string marker_class = feedback->marker_name.substr(0, 2);
  int marker_id = 0; 

  try
  {
    marker_id = boost::lexical_cast<int>(feedback->marker_name.substr(2, u - 2));
  }
  catch(boost::bad_lexical_cast &ex)
  {
    ROS_ERROR("Invalid marker name ('%s') : %s", feedback->marker_name.c_str(), ex.what());
    return;
  }
  
  if (marker_class == "IK")
    handler_->handleEndEffector(active_eef_[it->second], marker_id, feedback);
  else
    if (marker_class == "XY")
      handler_->handleVirtualJoint(active_vj_[it->second], marker_id, feedback);
    else
      ROS_ERROR("Unknown marker class ('%s') for marker '%s'", marker_class.c_str(), feedback->marker_name.c_str());  
}

}
