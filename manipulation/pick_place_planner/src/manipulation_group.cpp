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

ManipulationGroup::ManipulationGroup(const kinematic_model::KinematicModelConstPtr &kinematic_model,
                                     const std::string &group_name)
{
    kinematic_model_ = kinematic_model;
    group_name_ = group_name;

    /* assume that group_name is either an arm or a group which contains arms */
    std::vector<std::string> arm_names = kinematic_model->getJointModelGroup(group_name)->getSubgroupNames();
    if(arm_names.empty())
    {
        arm_names.push_back(group_name_);
    }

    end_effector_names_.resize(arm_names.size());
    for(unsigned int arm_i = 0; arm_i < arm_names.size(); ++arm_i)
    {
        end_effector_names_[arm_i] = kinematic_model->getJointModelGroup(arm_names[arm_i])->getAttachedEndEffectorGroupName();
    }
}

moveit_msgs::AttachedCollisionObject ManipulationGroup::getAttachedBodyMsg(const std::string &body_name) const
{
    moveit_msgs::AttachedCollisionObject attached_object = getAttachedBodyMsg(body_name, end_effector_names_.front());
    return attached_object;
}

moveit_msgs::AttachedCollisionObject ManipulationGroup::getAttachedBodyMsg(const std::string &body_name, 
                                                                           const std::string &end_effector_name) const
{
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = kinematic_model_->getJointModelGroup(end_effector_name)->getEndEffectorParentGroup().second;

    attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
    attached_object.object.id = body_name;
    attached_object.touch_links = kinematic_model_->getJointModelGroup(end_effector_name)->getLinkModelNames();
    return attached_object;
}


}
