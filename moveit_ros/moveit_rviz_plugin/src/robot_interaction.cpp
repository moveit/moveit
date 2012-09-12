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

#include "moveit_rviz_plugin/robot_interaction.h"
#include "moveit_rviz_plugin/planning_display.h"
#include "moveit_rviz_plugin/interactive_marker_helpers.h"

#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <limits>

namespace moveit_rviz_plugin
{

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "planning_display_interactive_marker_topic";

RobotInteraction::RobotInteraction(PlanningDisplay *planning_display, rviz::DisplayContext* context) :
  planning_display_(planning_display), context_(context)
{  
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(INTERACTIVE_MARKER_TOPIC);
}

RobotInteraction::~RobotInteraction(void)
{
  delete int_marker_server_;
  
}

void RobotInteraction::decideActiveComponents(void)
{
  decideActiveEndEffectors();
  decideActiveVirtualJoints();
}

double RobotInteraction::computeGroupScale(const std::string &group)
{
  if (!planning_display_->getPlanningSceneMonitor())
    return 0.0;
  const planning_models::KinematicModelConstPtr &kmodel = planning_display_->getPlanningSceneMonitor()->getKinematicModel();
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel->getJointModelGroup(group);
  if (!jmg)
    return 0.0;
  
  const std::vector<std::string> &links = jmg->getLinkModelNames();
  if (links.empty())
    return 0.0;
  
  std::vector<double> scale(3, 0.0);
  std::vector<double> low(3, std::numeric_limits<double>::infinity());
  std::vector<double> hi(3, -std::numeric_limits<double>::infinity());
  planning_models::KinematicState default_state(kmodel);
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
  return std::max(std::max(scale[0], scale[1]), scale[2]) * sqrt_3;
}

void RobotInteraction::decideActiveVirtualJoints(void)
{ 
  active_vj_.clear();  
}

void RobotInteraction::decideActiveEndEffectors(void)
{
  active_eef_.clear();
  
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  const planning_models::KinematicModelConstPtr &kmodel = planning_display_->getPlanningSceneMonitor()->getKinematicModel();
  if (!kmodel)
    return;

  std::string group = planning_display_->getCurrentPlanningGroup();
  if (group.empty())
    return;

  const boost::shared_ptr<const srdf::Model> &srdf = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getSrdfModel();
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel->getJointModelGroup(group);
  
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
  invalid_start_state_.clear();
  invalid_goal_state_.clear();
  
  active_eef_.clear();
  active_vj_.clear();
  shown_markers_.clear();
  int_marker_server_->clear();  
  int_marker_server_->applyChanges();
}

void RobotInteraction::publishInteractiveMarkers(void)
{ 
  //  ros::WallTime start = ros::WallTime::now();

  shown_markers_.clear();
  int_marker_server_->clear();
  
  std::vector<bool> start_goal(2);
  start_goal[0] = planning_display_->subProp("Planning Request")->subProp("Query Start State")->getValue().toBool();
  start_goal[1] = planning_display_->subProp("Planning Request")->subProp("Query Goal State")->getValue().toBool();
  
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
    for (int s = 0 ; s < 2 ; ++s)
      if (start_goal[s])
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();
        pose.header.stamp = ros::Time::now();
        
        const planning_models::KinematicState::LinkState *ls = 
          s == 0 ?
          planning_display_->getQueryStartState()->getLinkState(active_eef_[i].tip_link) :
          planning_display_->getQueryGoalState()->getLinkState(active_eef_[i].tip_link);
        planning_models::msgFromPose(ls->getGlobalLinkTransform(), pose.pose);
        
        std::string marker_name = "IK_" + boost::lexical_cast<std::string>(s) + "_" + active_eef_[i].tip_link;
        shown_markers_[marker_name] = i;
        visualization_msgs::InteractiveMarker im = make6DOFMarker(marker_name, pose, active_eef_[i].scale);
        if (s == 0)
        {
          if (invalid_start_state_.find(active_eef_[i].tip_link) != invalid_start_state_.end())
            addErrorMarker(im);
        }
        else
        {
          if (invalid_goal_state_.find(active_eef_[i].tip_link) != invalid_goal_state_.end())
            addErrorMarker(im);
        }
        int_marker_server_->insert(im);
        int_marker_server_->setCallback(im.name, boost::bind(&RobotInteraction::processInteractiveMarkerFeedback, this, _1));
        ROS_DEBUG("Publishing interactive marker %s", marker_name.c_str());
      }
  int_marker_server_->applyChanges();
  //  ROS_INFO("Spend %0.5lf s to publish markers", (ros::WallTime::now() - start).toSec());
}

void RobotInteraction::computeProcessInteractiveMarkerFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{   
  //  ros::WallTime start = ros::WallTime::now();

  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
    return;

  if (!planning_display_->getPlanningSceneMonitor())
    return;

  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    invalid_start_state_.clear();
    invalid_goal_state_.clear();
  }
  
  std::string planning_frame = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();

  static const double IK_TIMEOUT = 0.1;
  
  if (feedback->marker_name[1] == 'K')
  {
    bool start = feedback->marker_name[3] == '0'; // start state marker
    const EndEffector &ee = active_eef_[it->second];
    geometry_msgs::Pose target_pose = feedback->pose;
    if (feedback->header.frame_id != planning_frame)
    {
      geometry_msgs::PoseStamped tpose;
      tpose.header = feedback->header;
      tpose.pose = feedback->pose;
      try
      {
        context_->getTFClient()->transformPose(planning_frame, tpose, tpose);
        target_pose = tpose.pose;
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", tpose.header.frame_id.c_str(), planning_frame.c_str());
      }
    }
    if (start)
    {
      if (planning_display_->getQueryStartState()->getJointStateGroup(ee.group)->setFromIK(target_pose, ee.tip_link, IK_TIMEOUT))
      {
        //        planning_display_->addBackgroundJob(boost::bind(&RobotInteraction::computeMetrics, this, true, ee.group));
        computeMetricsInternal(computed_metrics_[std::make_pair(true, ee.group)], ee, *planning_display_->getQueryStartState(), planning_display_->getPayload());
        invalid_start_state_.erase(ee.tip_link);
      }
      else
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
          invalid_start_state_.insert(ee.tip_link);
      planning_display_->updateQueryStartState();
    }
    else
    {
      if (planning_display_->getQueryGoalState()->getJointStateGroup(ee.group)->setFromIK(target_pose, ee.tip_link, IK_TIMEOUT))
      {  
        //        planning_display_->addBackgroundJob(boost::bind(&RobotInteraction::computeMetrics, this, false, ee.group));
        computeMetricsInternal(computed_metrics_[std::make_pair(false, ee.group)], ee, *planning_display_->getQueryGoalState(),planning_display_->getPayload());
        invalid_goal_state_.erase(ee.tip_link);
      }
      else  
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
          invalid_goal_state_.insert(ee.tip_link);
      planning_display_->updateQueryGoalState();
    }
  } 
  //  ROS_INFO("Spend %0.5lf s to process IM feedback", (ros::WallTime::now() - start).toSec());
}

void RobotInteraction::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  planning_display_->addBackgroundJob(boost::bind(&RobotInteraction::computeProcessInteractiveMarkerFeedback, this, feedback));
}

void RobotInteraction::computeMetrics(double payload)
{
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    computeMetricsInternal(computed_metrics_[std::make_pair(true, active_eef_[i].group)], active_eef_[i], *planning_display_->getQueryStartState(), payload);
    computeMetricsInternal(computed_metrics_[std::make_pair(false, active_eef_[i].group)], active_eef_[i], *planning_display_->getQueryGoalState(), payload);
  }
}  

void RobotInteraction::computeMetrics(bool start, const std::string &group, double payload)
{
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
    if (active_eef_[i].group == group)
      computeMetricsInternal(computed_metrics_[std::make_pair(start, group)], active_eef_[i],
                             start ? *planning_display_->getQueryStartState() : *planning_display_->getQueryGoalState(), payload);
  if (start)
    planning_display_->updateQueryStartState();
  else
    planning_display_->updateQueryGoalState();
}

void RobotInteraction::computeMetricsInternal(std::map<std::string, double> &metrics, const EndEffector &ee, const planning_models::KinematicState &state, double payload)
{ 
  metrics.clear();
  
  // Max payload
  if (planning_display_->getDynamicsSolver(ee.group))
  {
    double max_payload;
    unsigned int saturated_joint;
    std::vector<double> joint_values;
    state.getJointStateGroup(ee.group)->getGroupStateValues(joint_values);
    if(planning_display_->getDynamicsSolver(ee.group)->getMaxPayload(joint_values, max_payload, saturated_joint))
    {
      metrics["max_payload"] = max_payload;      
      metrics["saturated_joint"] = saturated_joint;      
    } 
    std::vector<double> joint_torques;
    joint_torques.resize(joint_values.size());    
    if(planning_display_->getDynamicsSolver(ee.group)->getPayloadTorques(joint_values, payload, joint_torques))
    {
      for(unsigned int i=0; i < joint_torques.size(); ++i)
      {
        std::stringstream stream;
        stream << "torque[" << i << "]";        
        metrics[stream.str()] = joint_torques[i];
      }      
    }
    
  } 
  
  if (planning_display_->getKinematicsMetrics())
  {
    double manipulability_index, condition_number;
    if(planning_display_->getKinematicsMetrics()->getManipulabilityIndex(state, ee.group, manipulability_index))
      metrics["manipulability_index"] = manipulability_index;
    if(planning_display_->getKinematicsMetrics()->getConditionNumber(state, ee.group, condition_number))
      metrics["condition_number"] = condition_number;
  }
}

}
