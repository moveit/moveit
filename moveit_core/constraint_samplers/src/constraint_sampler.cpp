/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "constraint_samplers/constraint_sampler.h"
#include "constraint_samplers/default_constraint_samplers.h"
#include "constraint_samplers/union_constraint_sampler.h"
#include <ros/console.h>

constraint_samplers::ConstraintSampler::ConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name)
{
  jmg_ = scene->getKinematicModel()->getJointModelGroup(group_name);
  scene_ = scene;
  if (!jmg_)
    ROS_FATAL("A JointModelGroup should have been specified for the constraint sampler");
}

void constraint_samplers::ConstraintSampler::visualizeDistribution(const planning_models::KinematicState &reference_state, const std::string &link_name, unsigned int attempts,
                                                                   unsigned int count, visualization_msgs::MarkerArray &markers)
{
  planning_models::KinematicState ks(reference_state); 
  std_msgs::ColorRGBA color;
  color.r = 1.0f;
  color.g = 0.0f;
  color.b = 0.0f;
  color.a = 1.0f;
  for (unsigned int i = 0 ; i < count ; ++i)
  {
    if (!sample(ks.getJointStateGroup(getJointModelGroup()->getName()), ks, attempts))
      continue;
    const planning_models::KinematicState::LinkState *ls = ks.getLinkState(link_name);
    if (ls)
    {
      const Eigen::Vector3d &pos = ls->getGlobalLinkTransform().translation();
      visualization_msgs::Marker mk;
      mk.header.stamp = ros::Time::now();
      mk.header.frame_id = jmg_->getParentModel()->getModelFrame();
      mk.ns = "constraint_samples";
      mk.id = i;
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.position.x = pos.x();
      mk.pose.position.y = pos.y();
      mk.pose.position.z = pos.z();
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
      mk.color = color;
      mk.lifetime = ros::Duration(30.0);
      markers.markers.push_back(mk);
    }
  }
}
