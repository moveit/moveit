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

#include "moveit_rviz_plugin/planning_markers.h"
#include "moveit_rviz_plugin/planning_display.h"

#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>

namespace moveit_rviz_plugin
{

static visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name,
                                                            const geometry_msgs::PoseStamped &stamped, 
                                                            double scale,
                                                            bool fixed = false);

PlanningMarkers::PlanningMarkers(PlanningDisplay *planning_display, rviz::DisplayContext* context) :
  planning_display_(planning_display), context_(context)
{  
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer("planning_display_interactive_marker_topic");
}

PlanningMarkers::~PlanningMarkers(void)
{
  delete int_marker_server_;
  
}

void PlanningMarkers::computeMarkerScale(IKMarker &ik_marker)
{
  ik_marker.scale = 0.0;

  if (!planning_display_->getPlanningSceneMonitor())
    return;
  const planning_models::KinematicModelConstPtr &kmodel = planning_display_->getPlanningSceneMonitor()->getKinematicModel();
  const planning_models::KinematicModel::JointModelGroup *jmg = kmodel->getJointModelGroup(ik_marker.eef_group);
  if (!jmg)
    return;
  
  const std::vector<std::string> &links = jmg->getLinkModelNames();
  if (links.empty())
    return;
  
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

  ik_marker.scale = std::max(std::max(scale[0], scale[1]), scale[2]);
}

void PlanningMarkers::decideInteractiveMarkers(void)
{
  ik_markers_.clear();

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
        // we found an end-effector for the selected group; we need to add a 6DOF marker
        IKMarker im;
        im.group = group;
        im.tip_link = eef[i].parent_link_;
        im.eef_group = eef[i].component_group_;
        ik_markers_.push_back(im);
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
            // we found an end-effector for the selected group; we need to add a 6DOF marker
            IKMarker im;
            im.group = it->first->getName();
            im.tip_link = eef[i].parent_link_;
            im.eef_group = eef[i].component_group_;
            ik_markers_.push_back(im);
            break;
          }
      }
    }
  for (std::size_t i = 0 ; i < ik_markers_.size() ; ++i)
    computeMarkerScale(ik_markers_[i]);
}

void PlanningMarkers::clear(void)
{
  ik_markers_.clear();
  shown_markers_.clear();
  int_marker_server_->clear();  
  int_marker_server_->applyChanges();
}

void PlanningMarkers::publishInteractiveMarkers(void)
{
  shown_markers_.clear();
  int_marker_server_->clear();
  
  std::vector<bool> start_goal(2);
  start_goal[0] = planning_display_->subProp("Planning Request")->subProp("Query Start State")->getValue().toBool();
  start_goal[1] = planning_display_->subProp("Planning Request")->subProp("Query Goal State")->getValue().toBool();
  
  for (std::size_t i = 0 ; i < ik_markers_.size() ; ++i)
    for (int s = 0 ; s < 2 ; ++s)
      if (start_goal[s])
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();
        pose.header.stamp = ros::Time::now();
        
        const planning_models::KinematicState::LinkState *ls = 
          s == 0 ?
          planning_display_->getQueryStartState()->getLinkState(ik_markers_[i].tip_link) :
          planning_display_->getQueryGoalState()->getLinkState(ik_markers_[i].tip_link);
        
        Eigen::Affine3d transf = ls->getGlobalLinkTransform(); 
        planning_models::msgFromPose(transf, pose.pose);
        
        std::string marker_name = "IK_" + boost::lexical_cast<std::string>(s) + "_" + ik_markers_[i].tip_link;
        shown_markers_[marker_name] = i;
        visualization_msgs::InteractiveMarker im = make6DOFMarker(marker_name, pose, ik_markers_[i].scale);
        int_marker_server_->insert(im);
        int_marker_server_->setCallback(im.name, boost::bind(&PlanningMarkers::processInteractiveMarkerFeedback, this, _1));
      }
  int_marker_server_->applyChanges();
}

void PlanningMarkers::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{ 
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
    return;
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  std::string planning_frame = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();

  static const double IK_TIMEOUT = 0.1;
  
  if (feedback->marker_name[1] == 'K')
  {
    bool start = feedback->marker_name[3] == '0'; // start state marker
    const IKMarker &im = ik_markers_[it->second];
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
      planning_display_->getQueryStartState()->getJointStateGroup(im.group)->setFromIK(target_pose, im.tip_link, IK_TIMEOUT);
      planning_display_->updateQueryStartState();
      double max_payload;      
      std::vector<double> joint_values;      
      planning_display_->getQueryStartState()->getJointStateGroup(im.group)->getGroupStateValues(joint_values);      
      /*      if(planning_display_->getDynamicsSolver(im.group))
      {
        if(planning_display_->getDynamicsSolver(im.group)->getMaxPayload(joint_values,max_payload))
        {
          // Do nothing for now
        }    
      } */     
    }
    else
    {
      planning_display_->getQueryGoalState()->getJointStateGroup(im.group)->setFromIK(target_pose, im.tip_link, IK_TIMEOUT);
      planning_display_->updateQueryGoalState();
    }
  }
}

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     double scale,
                                                     bool fixed)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  
  visualization_msgs::InteractiveMarkerControl control;
  
  if (fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  return int_marker;
}

}
