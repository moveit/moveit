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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/robot_state/link_state.h>
#include <moveit/robot_state/robot_state.h>

robot_state::LinkState::LinkState(RobotState *state, const robot_model::LinkModel* lm) :
  robot_state_(state), link_model_(lm), parent_joint_state_(NULL), parent_link_state_(NULL)
{
  global_link_transform_.setIdentity();
  global_collision_body_transform_.setIdentity();
}

robot_state::LinkState::~LinkState()
{
}

void robot_state::LinkState::computeTransformForward(const Eigen::Affine3d& parent_transform)
{
  global_link_transform_ = parent_transform;

  // do fwd transforms
  const std::vector<robot_model::JointModel*> &child_jmodels = link_model_->getChildJointModels();
  for (std::size_t i = 0 ; i < child_jmodels.size() ; ++i)
    robot_state_->getLinkState(child_jmodels[i]->getChildLinkModel()->getName())->computeTransformForward(this);

  computeGeometryTransforms();
}

void robot_state::LinkState::computeTransformForward(const LinkState *parent_link)
{
  global_link_transform_ = parent_link->global_link_transform_ * link_model_->getJointOriginTransform() * parent_joint_state_->getVariableTransform();

  // do fwd transforms
  const std::vector<robot_model::JointModel*> &child_jmodels = link_model_->getChildJointModels();
  for (std::size_t i = 0 ; i < child_jmodels.size() ; ++i)
    robot_state_->getLinkState(child_jmodels[i]->getChildLinkModel()->getName())->computeTransformForward(this);

  computeGeometryTransforms();
}

void robot_state::LinkState::computeTransformBackward(const LinkState *child_link)
{
  global_link_transform_ = child_link->global_link_transform_ * (child_link->link_model_->getJointOriginTransform() * child_link->parent_joint_state_->getVariableTransform()).inverse();
  if (parent_link_state_)
    parent_link_state_->computeTransformBackward(this);
  else
    parent_joint_state_->setVariableValues(global_link_transform_);

  // do fwd transforms
  const std::vector<robot_model::JointModel*> &child_jmodels = link_model_->getChildJointModels();
  for (std::size_t i = 0 ; i < child_jmodels.size() ; ++i)
  {
    LinkState *child = robot_state_->getLinkState(child_jmodels[i]->getChildLinkModel()->getName());
    if (child != child_link)
      child->computeTransformForward(this);
  }

  computeGeometryTransforms();
}

void robot_state::LinkState::computeTransformBackward(const Eigen::Affine3d& child_transform)
{
  global_link_transform_ = child_transform;
  if (parent_link_state_)
    parent_link_state_->computeTransformBackward(this);
  else
    parent_joint_state_->setVariableValues(global_link_transform_);
  // do fwd transforms
  const std::vector<robot_model::JointModel*> &child_jmodels = link_model_->getChildJointModels();
  for (std::size_t i = 0 ; i < child_jmodels.size() ; ++i)
    robot_state_->getLinkState(child_jmodels[i]->getChildLinkModel()->getName())->computeTransformForward(this);

  computeGeometryTransforms();
}

void robot_state::LinkState::computeGeometryTransforms()
{
  global_collision_body_transform_ = global_link_transform_ * link_model_->getCollisionOriginTransform();
  updateAttachedBodies();
}

void robot_state::LinkState::computeTransform()
{
  global_link_transform_ = (parent_link_state_ ? parent_link_state_->global_link_transform_ : robot_state_->getRootTransform())
    * link_model_->getJointOriginTransform() * parent_joint_state_->getVariableTransform();
  computeGeometryTransforms();
}

void robot_state::LinkState::updateAttachedBodies()
{
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    it->second->computeTransform(global_link_transform_);
}

bool robot_state::LinkState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const robot_state::AttachedBody* robot_state::LinkState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    logError("Attached body '%s' not found on link '%s'", id.c_str(), getName().c_str());
    return NULL;
  }
  else
    return it->second;
}

void robot_state::LinkState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}
