/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <pick_place_planner/manipulation_group.h>

namespace pick_place_planner
{

ManipulationGroup::ManipulationGroup(const planning_models::KinematicModelConstPtr &kinematic_model,
                                     const std::string &group_name)
{
  group_name_ = group_name;
  arm_names_ = kinematic_model->getJointModelGroup(group_name)->getSubgroupNames();
  if(arm_names_.empty())
  {
    arm_names_.push_back(group_name_);
  }
  
  for(unsigned int i=0; i < arm_names_.size(); ++i)
  {    
    arm_link_names_map_[arm_names_[i]] = kinematic_model->getJointModelGroup(arm_names_[i])->getLinkModelNames();
    end_effector_names_map_[arm_names_[i]] = kinematic_model->getJointModelGroup(arm_names_[i])->getAttachedEndEffectorGroupName();    
    end_effector_link_names_map_[arm_names_[i]] = kinematic_model->getJointModelGroup(end_effector_names_map_[arm_names_[i]])->getLinkModelNames();        
    end_effector_link_names_.insert(end_effector_link_names_.end(),end_effector_link_names_map_.find(arm_names_[i])->second.begin(),end_effector_link_names_map_.find(arm_names_[i])->second.end());    
  }  
  kinematic_model_ = kinematic_model;  
}

moveit_msgs::AttachedCollisionObject ManipulationGroup::getAttachedBodyMsg(const std::string &body_name) const
{
  moveit_msgs::AttachedCollisionObject attached_object = getAttachedBodyMsg(body_name, arm_names_.front());
  return attached_object;
}

moveit_msgs::AttachedCollisionObject ManipulationGroup::getAttachedBodyMsg(const std::string &body_name, 
                                                                           const std::string &arm_name) const
{
  moveit_msgs::AttachedCollisionObject attached_object;
  if(arm_link_names_map_.find(arm_name) == arm_link_names_map_.end())
  {
    ROS_ERROR("Could not find %s in arm link names map",arm_name.c_str());
    return attached_object;    
  }  
  attached_object.link_name = arm_link_names_map_.find(arm_name)->second.back();
  attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
  attached_object.object.id = body_name;
  attached_object.touch_links = end_effector_link_names_;  
  return attached_object;  
}


}
