/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>

moveit::core::RobotState::RobotState(const RobotModelConstPtr &robot_model, AllocComponents alloc_components)
  : robot_model_(robot_model)
  , called_new_for_(ALLOC_POSITION)
  , position_(new double[robot_model->getVariableCount() * 
			 (1 + (alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? 1 : 0) + 
			  (alloc_components & ALLOC_ACCELERATION ? 1 : 0))])
  , velocity_(alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? position_ + robot_model->getVariableCount() : NULL)
  , acceleration_(alloc_components & ALLOC_ACCELERATION ? velocity_ + robot_model->getVariableCount() : NULL)
  , variable_joint_transforms_(NULL)
  , global_link_transforms_(NULL)
  , global_collision_body_transforms_(NULL)
  , dirty_joint_transforms_(NULL)
  , dirty_link_transforms_(NULL)
  , dirty_collision_body_transforms_(NULL)
  , rng_(NULL)
{
  if (alloc_components & ALLOC_TRANSFORMS)
    allocTransforms();
  if (velocity_)
    memset(velocity_, 0, sizeof(double) * robot_model_->getVariableCount());
  if (acceleration_)
    memset(acceleration_, 0, sizeof(double) * robot_model_->getVariableCount());
}

moveit::core::RobotState::RobotState(const RobotState &other)
  : position_(NULL)
  , velocity_(NULL)
  , acceleration_(NULL)
  , variable_joint_transforms_(NULL)
  , global_link_transforms_(NULL)
  , global_collision_body_transforms_(NULL)
  , rng_(NULL)
{
  copyFrom(other);
}

moveit::core::RobotState::~RobotState()
{
  delete[] position_;
  if (called_new_for_ & ALLOC_VELOCITY)
    delete[] velocity_;
  if (called_new_for_ & ALLOC_ACCELERATION)
    delete[] acceleration_;
  if (rng_)
    delete rng_;
}

moveit::core::RobotState& moveit::core::RobotState::operator=(const RobotState &other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void moveit::core::RobotState::allocVelocity()
{
  if (!velocity_)
  {
    called_new_for_ |= ALLOC_VELOCITY;
    velocity_ = new double[robot_model_->getVariableCount()];
    memset(velocity_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void moveit::core::RobotState::allocAcceleration()
{
  if (!acceleration_)
  {
    if (velocity_)
    {
      called_new_for_ |= ALLOC_ACCELERATION;
      acceleration_ = new double[robot_model_->getVariableCount()];
      memset(acceleration_, 0, sizeof(double) * robot_model_->getVariableCount());
    }
    else
    {
      called_new_for_ |= ALLOC_VELOCITY;
      velocity_ = new double[2 * robot_model_->getVariableCount()];
      memset(velocity_, 0, sizeof(double) * 2 * robot_model_->getVariableCount());
      acceleration_ = velocity_ + robot_model_->getVariableCount();
    }
  }
}

void moveit::core::RobotState::allocTransforms()
{
  if (!variable_joint_transforms_)
  {
    transforms_.resize(robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() + robot_model_->getLinkGeometryCount(),
                       Eigen::Affine3d::Identity());
    if (transforms_.size() > 0)
    {
      variable_joint_transforms_ = &transforms_[0];      
      global_link_transforms_ = variable_joint_transforms_ + robot_model_->getJointModelCount();
      global_collision_body_transforms_ = global_link_transforms_ + robot_model_->getLinkModelCount();
    }
    dirty_joint_transforms_ = robot_model_->getRootJoint();
  }
}

void moveit::core::RobotState::copyFrom(const RobotState &other)
{
  robot_model_ = other.robot_model_;
  
  // if the other state has allocated memory for transforms, we may have to copy data
  if (other.variable_joint_transforms_)
  {
    // if this state does not have memory allocated for transforms yet, allocate it
    if (!variable_joint_transforms_)
      allocTransforms();
    
    // if the other state is fully computed, it is worth copying the data
    if (other.dirty_joint_transforms_ == NULL)
    {
      transforms_ = other.transforms_;
      dirty_joint_transforms_ = NULL;
      dirty_collision_body_transforms_ = other.dirty_collision_body_transforms_;
      dirty_link_transforms_ = other.dirty_link_transforms_;
    }
    else
    {
      // otherwise, we will just assume everything is dirty and re-compute when needed
      dirty_joint_transforms_ = robot_model_->getRootJoint();
      dirty_collision_body_transforms_ = NULL;
      dirty_link_transforms_ = NULL;
    }
  }
  else
  {
    // no transforms to copy, so everything will become dirty if/when they get allocated.
    dirty_joint_transforms_ = NULL;
    dirty_collision_body_transforms_ = NULL;
    dirty_link_transforms_ = NULL;
  }
  
  // if we have previously allocated some memory, 
  if (position_)
  {
    // see if more memory needs to be allocated, minimizing calls to new
    if (other.acceleration_)
      allocAcceleration();
    else
      if (other.velocity_)
        allocVelocity();
    // copy the data. we use 3 calls to memcpy to avoid problems of non-contiguous blocks at source & destination
    std::size_t c = robot_model_->getVariableCount() * sizeof(double);
    memcpy(position_, other.position_, c);
    if (other.velocity_)
      memcpy(velocity_, other.velocity_, c);
    if (other.acceleration_)
      memcpy(acceleration_, other.acceleration_, c);      
  }
  else
  {
    // we allocate all the memory we need in one block
    called_new_for_ = ALLOC_POSITION;
    std::size_t vc = robot_model_->getVariableCount();
    position_ = new double[(1 + (other.velocity_ ? 1 : 0) + (other.acceleration_ ? 1 : 0)) * vc];
    std::size_t c = vc * sizeof(double);
    
    // copy the data. we use 3 calls to memcpy to avoid problems of non-contiguous blocks at source & destination
    memcpy(position_, other.position_, c);
    if (other.velocity_)
    {
      velocity_ = position_ + vc;
      memcpy(velocity_, other.velocity_, c);
    }
    if (other.acceleration_)
    {
      acceleration_ = velocity_ + vc;
      memcpy(acceleration_, other.acceleration_, c);
    }
  }
  
  // copy attached bodies
  clearAttachedBodies();
  for (std::map<std::string, AttachedBody*>::const_iterator it = other.attached_body_map_.begin() ; it != other.attached_body_map_.end() ; ++it)
    attachBody(it->second->getName(), it->second->getShapes(), it->second->getFixedTransforms(),
               it->second->getTouchLinks(), it->second->getAttachedLinkName(), it->second->getDetachPosture());
}

void moveit::core::RobotState::setToRandomPositions()
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  robot_model_->getVariableRandomValues(rng, position_);
  dirty_joint_transforms_ = robot_model_->getRootJoint();
  // mimic values are correctly set in RobotModel
}

void moveit::core::RobotState::setToRandomPositions(const JointModelGroup *group)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  const std::vector<const JointModel*> &joints = group->getActiveJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
    joints[i]->getVariableRandomValues(rng, position_ + joints[i]->getFirstVariableIndex());
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyJointTransforms(group->getCommonRoot());
}

void moveit::core::RobotState::setToRandomPositionsNearBy(const JointModelGroup *group, const RobotState &near, const std::vector<double> &distances)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  const std::vector<const JointModel*> &joints = group->getActiveJointModels();
  assert(distances.size() == joints.size());
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    const int idx = joints[i]->getFirstVariableIndex();
    joints[i]->getVariableRandomValuesNearBy(rng, position_ + joints[i]->getFirstVariableIndex(), near.position_ + idx, distances[i]);
  }
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyJointTransforms(group->getCommonRoot());
}

void moveit::core::RobotState::setToRandomPositionsNearBy(const JointModelGroup *group, const RobotState &near, double distance)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  const std::vector<const JointModel*> &joints = group->getActiveJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    const int idx = joints[i]->getFirstVariableIndex();
    joints[i]->getVariableRandomValuesNearBy(rng, position_ + joints[i]->getFirstVariableIndex(), near.position_ + idx, distance);
  }
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyJointTransforms(group->getCommonRoot());
}

void moveit::core::RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultValues(position_); // mimic values are updated
  if (velocity_)
    memset(velocity_, 0, sizeof(double) * robot_model_->getVariableCount());
  if (acceleration_)
    memset(acceleration_, 0, sizeof(double) * robot_model_->getVariableCount());
  dirty_joint_transforms_ = robot_model_->getRootJoint();
}

void moveit::core::RobotState::setVariablePositions(const std::map<std::string, double> &variable_map)
{
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
  {
    int index = robot_model_->getVariableIndex(it->first);
    position_[index] = it->second;
    updateMimicPosition(index);
    dirtyJointTransforms(index);
  }
}

void moveit::core::RobotState::getMissingKeys(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string> &nm = robot_model_->getVariableNames();
  for (std::size_t i = 0 ; i < nm.size() ; ++i)
    if (variable_map.find(nm[i]) == variable_map.end())
      if (robot_model_->getJointOfVariable(nm[i])->getMimic() == NULL)
        missing_variables.push_back(nm[i]);
}

void moveit::core::RobotState::setVariablePositions(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position)
{
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
  { 
    int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];  
    updateMimicPosition(index);
    dirtyJointTransforms(index);
  }
}

void moveit::core::RobotState::setVariableVelocities(const std::map<std::string, double> &variable_map)
{
  allocVelocity();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
    velocity_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void moveit::core::RobotState::setVariableVelocities(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariableVelocities(const std::vector<std::string>& variable_names, const std::vector<double>& variable_velocity)
{
  assert(variable_names.size() == variable_velocity.size());
  allocVelocity();
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];  
}

void moveit::core::RobotState::setVariableAccelerations(const std::map<std::string, double> &variable_map)
{
  allocAcceleration();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
    acceleration_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void moveit::core::RobotState::setVariableAccelerations(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariableAccelerations(const std::vector<std::string>& variable_names, const std::vector<double>& variable_acceleration)
{
  assert(variable_names.size() == variable_acceleration.size());
  allocAcceleration();
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
    acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];  
}

void moveit::core::RobotState::setJointGroupPositions(const JointModelGroup *group, const double *gstate)
{
  const std::vector<int> &il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(position_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0 ; i < il.size() ; ++i)
      position_[il[i]] = gstate[i];
  }
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyJointTransforms(group->getCommonRoot());
}

void moveit::core::RobotState::setJointGroupPositions(const JointModelGroup *group, const Eigen::VectorXd& values)
{
  const std::vector<int> &il = group->getVariableIndexList();
  for (std::size_t i = 0 ; i < il.size() ; ++i)
    position_[il[i]] = values(i);
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyJointTransforms(group->getCommonRoot());
}

void moveit::core::RobotState::copyJointGroupPositions(const JointModelGroup *group, double *gstate) const
{
  const std::vector<int> &il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, position_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0 ; i < il.size() ; ++i)
      gstate[i] = position_[il[i]];
}

void moveit::core::RobotState::copyJointGroupPositions(const JointModelGroup *group, Eigen::VectorXd& values) const
{
  const std::vector<int> &il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0 ; i < il.size() ; ++i)
    values(i) = position_[il[i]];  
}

void moveit::core::RobotState::updateCollisionBodyTransforms()
{
  if (dirty_joint_transforms_ != NULL)
  {
    updateJointTransforms();
    updateLinkTransforms();
  }
  else
    if (dirty_link_transforms_ != NULL)
      updateLinkTransforms();
  
  if (dirty_collision_body_transforms_ != NULL)
  {
    const std::vector<const LinkModel*> &links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = NULL;
    for (std::size_t i = 0 ; i < links.size() ; ++i)
    {
      const EigenSTL::vector_Affine3d &ot = links[i]->getCollisionOriginTransforms();
      int index_co = links[i]->getFirstCollisionBodyTransformIndex();
      int index_l = links[i]->getLinkIndex();
      for (std::size_t j = 0 ; j < ot.size() ; ++j)
        global_collision_body_transforms_[index_co + j] = global_link_transforms_[index_l] * ot[j];
    }
  }
}

void moveit::core::RobotState::updateLinkTransforms()
{
  if (dirty_joint_transforms_ != NULL)
    updateJointTransforms(); // resets dirty_joint_transforms_, makes sure memory is allocated for transforms
  if (dirty_link_transforms_ != NULL)
  {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_)
      dirty_collision_body_transforms_ = robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    else
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    dirty_link_transforms_ = NULL;
  }
}

void moveit::core::RobotState::updateLinkTransformsInternal(const JointModel *start)
{
  const std::vector<const LinkModel*> &links = start->getDescendantLinkModels();
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    const LinkModel *parent = links[i]->getParentJointModel()->getParentLinkModel();
    int index_j = links[i]->getParentJointModel()->getJointIndex();
    int index_l = links[i]->getLinkIndex();
    if (parent)
      global_link_transforms_[index_l] = global_link_transforms_[parent->getLinkIndex()] * links[i]->getJointOriginTransform() * variable_joint_transforms_[index_j];
    else
      global_link_transforms_[index_l] = links[i]->getJointOriginTransform() * variable_joint_transforms_[index_j];
  }
  
  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

void moveit::core::RobotState::updateJointTransforms()
{
  if (dirty_joint_transforms_ != NULL)
  {
    const std::vector<const JointModel*> &joint_models = dirty_joint_transforms_->getDescendantJointModels();
    if (dirty_link_transforms_)
      dirty_link_transforms_ = robot_model_->getCommonRoot(dirty_joint_transforms_, dirty_link_transforms_);
    else
      dirty_link_transforms_ = dirty_joint_transforms_;
    if (!variable_joint_transforms_)
      allocTransforms();
    int index = dirty_joint_transforms_->getJointIndex();
    int vindex = dirty_joint_transforms_->getFirstVariableIndex();
    dirty_joint_transforms_->computeTransform(position_ + vindex, variable_joint_transforms_[index]);
    dirty_joint_transforms_ = NULL;
    
    for (std::size_t i = 0 ; i < joint_models.size() ; ++i)
    {
      index = joint_models[i]->getJointIndex();
      vindex = joint_models[i]->getFirstVariableIndex();
      joint_models[i]->computeTransform(position_ + vindex, variable_joint_transforms_[index]);
    }
  }
}

void moveit::core::RobotState::updateStateWithLinkAt(const LinkModel *link, const Eigen::Affine3d& transform, bool backward)
{
  updateLinkTransforms(); // no link transforms must be dirty, otherwise the transform we set will be overwritten
  
  // update the fact that collision body transforms are out of date
  if (dirty_collision_body_transforms_)
    dirty_collision_body_transforms_ = robot_model_->getCommonRoot(dirty_collision_body_transforms_, link->getParentJointModel());
  else
    dirty_collision_body_transforms_ = link->getParentJointModel();
  
  global_link_transforms_[link->getLinkIndex()] = transform;

  // update link transforms for descendant links only (leaving the transform for the current link untouched)
  const std::vector<const JointModel*> &cj = link->getChildJointModels();
  for (std::size_t i = 0 ; i < cj.size() ; ++i)
    updateLinkTransformsInternal(cj[i]);
  
  // if we also need to go backward
  if (backward)
  {
    const LinkModel *parent_link = link;
    const LinkModel *child_link;
    while (parent_link->getParentJointModel()->getParentLinkModel())
    {
      child_link = parent_link;
      parent_link = parent_link->getParentJointModel()->getParentLinkModel();

      // update the transform of the parent
      global_link_transforms_[parent_link->getLinkIndex()] = global_link_transforms_[child_link->getLinkIndex()] *
        (child_link->getJointOriginTransform() * variable_joint_transforms_[child_link->getParentJointModel()->getJointIndex()]).inverse();

      // update link transforms for descendant links only (leaving the transform for the current link untouched)
      // with the exception of the child link we are coming backwards from
      const std::vector<const JointModel*> &cj = parent_link->getChildJointModels();
      for (std::size_t i = 0 ; i < cj.size() ; ++i)
        if (cj[i] != child_link->getParentJointModel())
          updateLinkTransformsInternal(cj[i]);
    }
    // update the root joint of the model to match (as best as possible given #DOF) the transfor we wish to obtain for the root link.
    // but I am disabling this code, since I do not think this function should modify variable values.
    //    parent_link->getParentJointModel()->computeVariableValues(global_link_transforms_[parent_link->getLinkIndex()],
    //                                                              position_ + parent_link->getParentJointModel()->getFirstVariableIndex());
  }
  
  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

bool moveit::core::RobotState::satisfiesBounds(double margin) const
{
  const std::vector<const JointModel*> &jm = robot_model_->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;
}

bool moveit::core::RobotState::satisfiesBounds(const JointModelGroup *group, double margin) const
{
  const std::vector<const JointModel*> &jm = group->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;  
}

void moveit::core::RobotState::enforceBounds()
{
  const std::vector<const JointModel*> &jm = robot_model_->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    enforceBounds(jm[i]);
}

void moveit::core::RobotState::enforceBounds(const JointModelGroup *joint_group)
{
  const std::vector<const JointModel*> &jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    enforceBounds(jm[i]);
}

double moveit::core::RobotState::distance(const RobotState &other, const JointModelGroup *joint_group) const
{
  double d = 0.0;
  const std::vector<const JointModel*> &jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    d += jm[i]->getDistanceFactor() * jm[i]->distance(position_ + idx, other.position_ + idx);
  }
  return d;  
}

void moveit::core::RobotState::interpolate(const RobotState &to, double t, RobotState &state, const JointModelGroup *joint_group) const
{
  const std::vector<const JointModel*> &jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    jm[i]->interpolate(position_ + idx, to.position_ + idx, t, state.position_ + idx);
  }
  const std::vector<const JointModel*> &mimic = joint_group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    state.updateMimicJoint(mimic[i]);
  state.dirtyJointTransforms(joint_group->getCommonRoot());
}

void moveit::core::RobotState::setAttachedBodyUpdateCallback(const AttachedBodyCallback &callback)
{
  attached_body_update_callback_ = callback;
}

bool moveit::core::RobotState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const moveit::core::AttachedBody* moveit::core::RobotState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    logError("Attached body '%s' not found", id.c_str());
    return NULL;
  }
  else
    return it->second;
}

void moveit::core::RobotState::attachBody(AttachedBody *attached_body)
{
  attached_body_map_[attached_body->getName()] = attached_body;
  attached_body->computeTransform(getGlobalLinkTransform(attached_body->getAttachedLink()));
  if (attached_body_update_callback_)
    attached_body_update_callback_(attached_body, true);
}

void moveit::core::RobotState::attachBody(const std::string &id,
                                          const std::vector<shapes::ShapeConstPtr> &shapes,
                                          const EigenSTL::vector_Affine3d &attach_trans,
                                          const std::set<std::string> &touch_links,
                                          const std::string &link,
                                          const sensor_msgs::JointState &detach_posture)
{
  const LinkModel *l = robot_model_->getLinkModel(link);
  AttachedBody *ab = new AttachedBody(l, id, shapes, attach_trans, touch_links, detach_posture);
  attached_body_map_[id] = ab;
  ab->computeTransform(getGlobalLinkTransform(l));
  if (attached_body_update_callback_)
    attached_body_update_callback_(ab, true);
}

void moveit::core::RobotState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

void moveit::core::RobotState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies, const JointModelGroup *group) const
{
  attached_bodies.clear();
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    if (group->hasLinkModel(it->second->getAttachedLinkName()))
      attached_bodies.push_back(it->second);
}

void moveit::core::RobotState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies, const LinkModel *lm) const
{
  attached_bodies.clear();
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    if (it->second->getAttachedLink() == lm)
      attached_bodies.push_back(it->second);
}

void moveit::core::RobotState::clearAttachedBodies()
{
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
  }
  attached_body_map_.clear();
}

void moveit::core::RobotState::clearAttachedBodies(const LinkModel *link)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.begin();
  while (it != attached_body_map_.end())
  {
    if (it->second->getAttachedLink() != link)
    {
      ++it;
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    std::map<std::string, AttachedBody*>::iterator del = it++;
    attached_body_map_.erase(del);
  }
}

void moveit::core::RobotState::clearAttachedBodies(const JointModelGroup *group)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.begin();
  while (it != attached_body_map_.end())
  {
    if (!group->hasLinkModel(it->second->getAttachedLinkName()))
    {
      ++it;
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    std::map<std::string, AttachedBody*>::iterator del = it++;
    attached_body_map_.erase(del);
  }
}

bool moveit::core::RobotState::clearAttachedBody(const std::string &id)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    attached_body_map_.erase(it);
    return true;
  }
  else
    return false;
}

const Eigen::Affine3d& moveit::core::RobotState::getFrameTransform(const std::string &id)
{
  updateLinkTransforms();
  return const_cast<RobotState*>(this)->getFrameTransform(id);
}

const Eigen::Affine3d& moveit::core::RobotState::getFrameTransform(const std::string &id) const
{
  if (!id.empty() && id[0] == '/')
    return getFrameTransform(id.substr(1));
  static const Eigen::Affine3d identity_transform = Eigen::Affine3d::Identity();
  if (id.size() + 1 == robot_model_->getModelFrame().size() && '/' + id == robot_model_->getModelFrame())
    return identity_transform;
  if (robot_model_->hasLinkModel(id))
  {
    const LinkModel *lm = robot_model_->getLinkModel(id);
    return global_link_transforms_[lm->getLinkIndex()];
  }
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    logError("Transform from frame '%s' to frame '%s' is not known ('%s' should be a link name or an attached body id).",
             id.c_str(), robot_model_->getModelFrame().c_str(), id.c_str());
    return identity_transform;
  }
  const EigenSTL::vector_Affine3d &tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    logError("Attached body '%s' has no geometry associated to it. No transform to return.", id.c_str());
    return identity_transform;
  }
  if (tf.size() > 1)
    logWarn("There are multiple geometries associated to attached body '%s'. Returning the transform for the first one.", id.c_str());
  return tf[0];
}

bool moveit::core::RobotState::knowsFrameTransform(const std::string &id) const
{
  if (!id.empty() && id[0] == '/')
    return knowsFrameTransform(id.substr(1));
  if (robot_model_->hasLinkModel(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && it->second->getGlobalCollisionBodyTransforms().size() == 1;
}

void moveit::core::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr,
                                               const std::vector<std::string> &link_names,
                                               const std_msgs::ColorRGBA& color,
                                               const std::string& ns,
                                               const ros::Duration& dur,
                                               bool include_attached) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names, include_attached);
  unsigned int id = cur_num;
  for (std::size_t i = cur_num ; i < arr.markers.size() ; ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void moveit::core::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string> &link_names, bool include_attached) const
{
  ros::Time tm = ros::Time::now();
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    logDebug("Trying to get marker for link '%s'", link_names[i].c_str());
    const LinkModel* lm = robot_model_->getLinkModel(link_names[i]);
    if (!lm)
      continue;
    if (include_attached)
      for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
        if (it->second->getAttachedLink() == lm)
        {
          for (std::size_t j = 0 ; j < it->second->getShapes().size() ; ++j)
          {
            visualization_msgs::Marker att_mark;
            att_mark.header.frame_id = robot_model_->getModelFrame();
            att_mark.header.stamp = tm;
            if (shapes::constructMarkerFromShape(it->second->getShapes()[j].get(), att_mark))
            {
              // if the object is invisible (0 volume) we skip it
              if (fabs(att_mark.scale.x * att_mark.scale.y * att_mark.scale.z) < std::numeric_limits<float>::epsilon())
                continue;
              tf::poseEigenToMsg(it->second->getGlobalCollisionBodyTransforms()[j], att_mark.pose);
              arr.markers.push_back(att_mark);
            }
          }
        }
    
    if (lm->getShapes().empty())
      continue;

    for (std::size_t j = 0 ; j < lm->getShapes().size() ; ++j)
    {
      visualization_msgs::Marker mark;
      mark.header.frame_id = robot_model_->getModelFrame();
      mark.header.stamp = tm;
      
      // we prefer using the visual mesh, if a mesh is available and we have one body to render
      const std::string& mesh_resource = lm->getVisualMeshFilename();
      if (mesh_resource.empty() || lm->getShapes().size() > 1)
      {
        if (!shapes::constructMarkerFromShape(lm->getShapes()[j].get(), mark))
          continue;
        // if the object is invisible (0 volume) we skip it
        if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
          continue;
      }
      else
      {
        mark.type = mark.MESH_RESOURCE;
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = mesh_resource;
        const Eigen::Vector3d &mesh_scale = lm->getVisualMeshScale();
        
        mark.scale.x = mesh_scale[0];
        mark.scale.y = mesh_scale[1];
        mark.scale.z = mesh_scale[2];
      }
      tf::poseEigenToMsg(global_collision_body_transforms_[lm->getFirstCollisionBodyTransformIndex() + j], mark.pose);
      arr.markers.push_back(mark);
    }
  }
}

Eigen::MatrixXd moveit::core::RobotState::getJacobian(const JointModelGroup *group, const Eigen::Vector3d &reference_point_position) const
{
  Eigen::MatrixXd result;
  if (!getJacobian(group, group->getLinkModels().back(), reference_point_position, result, false))
    throw Exception("Unable to compute Jacobian");
  return result;
}

bool moveit::core::RobotState::getJacobian(const JointModelGroup *group, const LinkModel *link, const Eigen::Vector3d &reference_point_position,
                                           Eigen::MatrixXd& jacobian, bool use_quaternion_representation) const
{
  if (!group->isChain())
  {
    logError("The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    logError("Link name '%s' does not exist in the chain '%s' or is not a child for this chain", link->getName().c_str(), group->getName().c_str());
    return false;
  }
  
  const robot_model::JointModel* root_joint_model = group->getJointRoots()[0];
  const robot_model::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
  Eigen::Affine3d reference_transform = root_link_model ? getGlobalLinkTransform(root_link_model).inverse() : Eigen::Affine3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  Eigen::Affine3d link_transform = reference_transform * getGlobalLinkTransform(link);
  Eigen::Vector3d point_transform = link_transform * reference_point_position;

  /*
  logDebug("Point from reference origin expressed in world coordinates: %f %f %f",
           point_transform.x(),
           point_transform.y(),
           point_transform.z());
  */

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while (link)
  {
    /*
    logDebug("Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());
    logDebug("Joint: %s",link_state->getParentJointState()->getName().c_str());
    */
    const JointModel *pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      if (pjm->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform * getGlobalLinkTransform(link);
        joint_axis = joint_transform.rotation() * static_cast<const robot_model::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3,1>(3,joint_index) = jacobian.block<3,1>(3,joint_index) + joint_axis;
      }
      else
        if (pjm->getType() == robot_model::JointModel::PRISMATIC)
        {
          joint_transform = reference_transform * getGlobalLinkTransform(link);
          joint_axis = joint_transform * static_cast<const robot_model::PrismaticJointModel*>(pjm)->getAxis();
          jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + joint_axis;
        }
        else
          if (pjm->getType() == robot_model::JointModel::PLANAR)
          {
            joint_transform = reference_transform * getGlobalLinkTransform(link);
            joint_axis = joint_transform * Eigen::Vector3d(1.0,0.0,0.0);
            jacobian.block<3,1>(0,joint_index) = jacobian.block<3,1>(0,joint_index) + joint_axis;
            joint_axis = joint_transform*Eigen::Vector3d(0.0,1.0,0.0);
            jacobian.block<3,1>(0,joint_index+1) = jacobian.block<3,1>(0,joint_index+1) + joint_axis;
            joint_axis = joint_transform*Eigen::Vector3d(0.0,0.0,1.0);
            jacobian.block<3,1>(0,joint_index+2) = jacobian.block<3,1>(0,joint_index+2) + joint_axis.cross(point_transform - joint_transform.translation());
            jacobian.block<3,1>(3,joint_index+2) = jacobian.block<3,1>(3,joint_index+2) + joint_axis;
          }
          else
            logError("Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation) 
  { // Quaternion representation
    // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
    // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond q(link_transform.rotation());
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::MatrixXd quaternion_update_matrix(4,3);
    quaternion_update_matrix << -x, -y, -z,
                                 w, -z,  y,
                                 z,  w, -x,
                                -y,  x,  w;
    jacobian.block(3,0,4,columns) = 0.5*quaternion_update_matrix*jacobian.block(3,0, 3, columns);
  }
  return true;
}

bool moveit::core::RobotState::setFromDiffIK(const JointModelGroup *jmg, const Eigen::VectorXd &twist, const std::string &tip,
                                             double dt, const GroupStateValidityCallbackFn &constraint)
{
  Eigen::VectorXd qdot;
  computeVariableVelocity(jmg, qdot, twist, getLinkModel(tip));
  return integrateVariableVelocity(jmg, qdot, dt, constraint);
}

bool moveit::core::RobotState::setFromDiffIK(const JointModelGroup *jmg, const geometry_msgs::Twist &twist, const std::string &tip,
                                             double dt, const GroupStateValidityCallbackFn &constraint)
{
  Eigen::Matrix<double, 6, 1> t;
  tf::twistMsgToEigen(twist, t);
  return setFromDiffIK(jmg, t, tip, dt, constraint);
}

void moveit::core::RobotState::computeVariableVelocity(const JointModelGroup *jmg, Eigen::VectorXd &qdot,
                                                       const Eigen::VectorXd &twist, const LinkModel *tip) const
{
  //Get the Jacobian of the group at the current configuration
  Eigen::MatrixXd J(6, jmg->getVariableCount());
  Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
  getJacobian(jmg, tip, reference_point, J, false);
  
  //Rotate the jacobian to the end-effector frame
  Eigen::Affine3d eMb = getGlobalLinkTransform(tip).inverse();
  Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
  eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  J = eWb * J;

  //Do the Jacobian moore-penrose pseudo-inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd U = svdOfJ.matrixU();
  const Eigen::MatrixXd V = svdOfJ.matrixV();
  const Eigen::VectorXd S = svdOfJ.singularValues();
  
  Eigen::VectorXd Sinv = S;
  static const double pinvtoler = std::numeric_limits<float>::epsilon();
  double maxsv = 0.0 ;
  for (std::size_t i = 0; i < S.rows(); ++i)
    if (fabs(S(i)) > maxsv) maxsv = fabs(S(i));
  for (std::size_t i = 0; i < S.rows(); ++i)
  {
    //Those singular values smaller than a percentage of the maximum singular value are removed
    if (fabs(S(i)) > maxsv * pinvtoler)
      Sinv(i) = 1.0 / S(i);
    else Sinv(i) = 0.0;
  }
  Eigen::MatrixXd Jinv = (V * Sinv.asDiagonal() * U.transpose());
  
  // Compute joint velocity
  qdot = Jinv * twist;
}

bool moveit::core::RobotState::integrateVariableVelocity(const JointModelGroup *jmg, const Eigen::VectorXd &qdot, double dt,
                                                         const GroupStateValidityCallbackFn &constraint)
{
  Eigen::VectorXd q(jmg->getVariableCount());
  copyJointGroupPositions(jmg, q);
  q = q + dt * qdot;
  setJointGroupPositions(jmg, q);
  enforceBounds(jmg);
  
  if (constraint)
  {
    std::vector<double> values;
    copyJointGroupPositions(jmg, values);
    return constraint(jmg, &values[0]);
  }
  else
    return true;
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const geometry_msgs::Pose &pose,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint,
                                         const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  return setFromIK(jmg, pose, solver->getTipFrame(), attempts, timeout, constraint, options);
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const geometry_msgs::Pose &pose, const std::string &tip,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint,
                                         const kinematics::KinematicsQueryOptions &options)
{
  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, mat, tip, consistency_limits, attempts, timeout, constraint, options);
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const Eigen::Affine3d &pose,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint,
                                         const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose, solver->getTipFrame(), consistency_limits, attempts, timeout, constraint, options);
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const Eigen::Affine3d &pose_in, const std::string &tip_in,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint,
                                         const kinematics::KinematicsQueryOptions &options)
{
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose_in, tip_in, consistency_limits, attempts, timeout, constraint, options);
}

namespace moveit
{
namespace core
{
namespace
{
bool ikCallbackFnAdapter(const JointModelGroup *group, const GroupStateValidityCallbackFn &constraint,
                         const geometry_msgs::Pose &, const std::vector<double> &ik_sol, moveit_msgs::MoveItErrorCodes &error_code)
{
  const std::vector<unsigned int> &bij = group->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0 ; i < bij.size() ; ++i)
    solution[i] = ik_sol[bij[i]];
  if (constraint(group, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}
}
}
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const Eigen::Affine3d &pose_in, const std::string &tip_in,
                                         const std::vector<double> &consistency_limits, unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    logError("No kinematics solver instantiated for this group");
    return false;
  }

  Eigen::Affine3d pose = pose_in;
  std::string tip = tip_in;

  // bring the pose to the frame of the IK solver
  const std::string &ik_frame = solver->getBaseFrame();
  if (!Transforms::sameFrame(ik_frame, robot_model_->getModelFrame()))
  {
    const LinkModel *lm = getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!lm)
      return false;
    pose = getGlobalLinkTransform(lm).inverse() * pose;
  }

  // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
  std::string tip_frame = solver->getTipFrame();

  // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more often that we need to
  if (!tip_frame.empty() && tip_frame[0] == '/')
    tip_frame = tip_frame.substr(1);
  
  if (tip != tip_frame)
  {
    if (hasAttachedBody(tip))
    {
      const AttachedBody *ab = getAttachedBody(tip);
      const EigenSTL::vector_Affine3d &ab_trans = ab->getFixedTransforms();
      if (ab_trans.size() != 1)
      {
        logError("Cannot use an attached body with multiple geometries as a reference frame.");
        return false;
      }
      tip = ab->getAttachedLinkName();
      pose = pose * ab_trans[0].inverse();
    }
    if (tip != tip_frame)
    {
      const robot_model::LinkModel *lm = getLinkModel(tip);
      if (!lm)
        return false;
      const robot_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (robot_model::LinkModel::AssociatedFixedTransformMap::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
        if (Transforms::sameFrame(it->first->getName(), tip_frame))
        {
          tip = tip_frame;
          pose = pose * it->second;
          break;
        }
    }
  }
  
  if (tip != tip_frame)
  {
    logError("Cannot compute IK for tip reference frame '%s'", tip.c_str());
    return false;
  }

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = jmg->getDefaultIKTimeout();

  if (attempts == 0)
    attempts = jmg->getDefaultIKAttempts();
  
  const std::vector<unsigned int> &bij = jmg->getKinematicsSolverJointBijection();
  Eigen::Quaterniond quat(pose.rotation());
  Eigen::Vector3d point(pose.translation());
  geometry_msgs::Pose ik_query;
  ik_query.position.x = point.x();
  ik_query.position.y = point.y();
  ik_query.position.z = point.z();
  ik_query.orientation.x = quat.x();
  ik_query.orientation.y = quat.y();
  ik_query.orientation.z = quat.z();
  ik_query.orientation.w = quat.w();

  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&ikCallbackFnAdapter, jmg, constraint, _1, _2, _3);

  bool first_seed = true;
  std::vector<double> initial_values;
  copyJointGroupPositions(jmg, initial_values);
  for (unsigned int st = 0 ; st < attempts ; ++st)
  {
    std::vector<double> seed(bij.size());

    // the first seed is the initial state
    if (first_seed)
    {
      first_seed = false;
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        seed[bij[i]] = initial_values[i];
    }
    else
    {
      // sample a random seed
      random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
      std::vector<double> random_values;
      jmg->getVariableRandomValues(rng, random_values);
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        seed[bij[i]] = random_values[i];
      
      if (options.lock_redundant_joints)
      {
        std::vector<unsigned int> red_joints;
        solver->getRedundantJoints(red_joints);
        for(std::size_t i = 0 ; i < red_joints.size(); ++i)
          seed[bij[red_joints[i]]] = initial_values[red_joints[i]];
      }
    }
    
    // compute the IK solution
    std::vector<double> ik_sol;
    moveit_msgs::MoveItErrorCodes error;
    if (ik_callback_fn ?
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error, options) :
        solver->searchPositionIK(ik_query, seed, timeout, consistency_limits, ik_sol, error, options))
    {
      std::vector<double> solution(bij.size());
      for (std::size_t i = 0 ; i < bij.size() ; ++i)
        solution[i] = ik_sol[bij[i]];
      setJointGroupPositions(jmg, solution);
      return true;
    }
  }
  return false;
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const EigenSTL::vector_Affine3d &poses_in, const std::vector<std::string> &tips_in,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  static const std::vector<std::vector<double> > consistency_limits;
  return setFromIK(jmg, poses_in, tips_in, consistency_limits, attempts, timeout, constraint, options);
}

bool moveit::core::RobotState::setFromIK(const JointModelGroup *jmg, const EigenSTL::vector_Affine3d &poses_in, const std::vector<std::string> &tips_in,
                                         const std::vector<std::vector<double> > &consistency_limits,
                                         unsigned int attempts, double timeout,
                                         const GroupStateValidityCallbackFn &constraint, const kinematics::KinematicsQueryOptions &options)
{
  if (poses_in.size() == 1 && tips_in.size() == 1 && consistency_limits.size() <= 1)
  {
    if (consistency_limits.empty())
      return setFromIK(jmg, poses_in[0], tips_in[0], attempts, timeout, constraint, options);
    else
      return setFromIK(jmg, poses_in[0], tips_in[0], consistency_limits[0], attempts, timeout, constraint, options);
  }

  const std::vector<std::string>& sub_group_names = jmg->getSubgroupNames();
  std::vector<const JointModelGroup*> sub_groups(sub_group_names.size());
  for (std::size_t i = 0 ; i < sub_group_names.size() ; ++i)
    sub_groups[i] = robot_model_->getJointModelGroup(sub_group_names[i]);
  
  if (poses_in.size() != sub_group_names.size())
  {
    logError("Number of poses must be the same as number of sub-groups");
    return false;
  }

  if (tips_in.size() != sub_group_names.size())
  {
    logError("Number of tip names must be the same as number of sub-groups");
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != sub_group_names.size())
  {
    logError("Number of consistency limit vectors must be the same as number of sub-groups");
    return false;
  }
  
  for (std::size_t i = 0 ; i < consistency_limits.size() ; ++i)
  {
    if (consistency_limits[i].size() != sub_groups[i]->getVariableCount())
    {
      logError("Number of joints in consistency_limits is %u but it should be should be %u", (unsigned int)i, sub_groups[i]->getVariableCount());
      return false;
    }
  }

  std::vector<kinematics::KinematicsBaseConstPtr> solvers;
  for (std::size_t i = 0; i < poses_in.size() ; ++i)
  {
    kinematics::KinematicsBaseConstPtr solver = sub_groups[i]->getSolverInstance();
    if (!solver)
    {
      logError("Could not find solver for group '%s'", sub_group_names[i].c_str());
      return false;
    }
    solvers.push_back(solver);
  }
  
  EigenSTL::vector_Affine3d transformed_poses = poses_in;
  std::vector<std::string> tip_names = tips_in;

  for (std::size_t i = 0 ; i < poses_in.size() ; ++i)
  {
    Eigen::Affine3d &pose = transformed_poses[i];
    std::string &tip = tip_names[i];

    // bring the pose to the frame of the IK solver
    const std::string &ik_frame = solvers[i]->getBaseFrame();
    if (!Transforms::sameFrame(ik_frame, robot_model_->getModelFrame()))
    {
      const LinkModel *lm = getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
      if (!lm)
        return false;
      pose = getGlobalLinkTransform(lm).inverse() * pose;
    }

    // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
    std::string tip_frame = solvers[i]->getTipFrame();

    // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more often that we need to
    if (!tip_frame.empty() && tip_frame[0] == '/')
      tip_frame = tip_frame.substr(1);

    if (tip != tip_frame)
    {
      if (hasAttachedBody(tip))
      {
        const AttachedBody *ab = getAttachedBody(tip);
        const EigenSTL::vector_Affine3d &ab_trans = ab->getFixedTransforms();
        if (ab_trans.size() != 1)
        {
          logError("Cannot use an attached body with multiple geometries as a reference frame.");
          return false;
        }
        tip = ab->getAttachedLinkName();
        pose = pose * ab_trans[0].inverse();
      }
      if (tip != tip_frame)
      {
        const robot_model::LinkModel *lm = getLinkModel(tip);
        if (!lm)
          return false;
        const robot_model::LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
        for (robot_model::LinkModel::AssociatedFixedTransformMap::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
          if (it->first->getName() == tip_frame)
          {
            tip = tip_frame;
            pose = pose * it->second;
            break;
          }
      }
    }

    if (tip != tip_frame)
    {
      logError("Cannot compute IK for tip reference frame '%s'", tip.c_str());
      return false;
    }
  }

  std::vector<geometry_msgs::Pose> ik_queries(poses_in.size());
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&ikCallbackFnAdapter, jmg, constraint, _1, _2, _3);

  for (std::size_t i = 0; i < transformed_poses.size() ; ++i)
  {
    Eigen::Quaterniond quat(transformed_poses[i].rotation());
    Eigen::Vector3d point(transformed_poses[i].translation());
    ik_queries[i].position.x = point.x();
    ik_queries[i].position.y = point.y();
    ik_queries[i].position.z = point.z();
    ik_queries[i].orientation.x = quat.x();
    ik_queries[i].orientation.y = quat.y();
    ik_queries[i].orientation.z = quat.z();
    ik_queries[i].orientation.w = quat.w();
  }

  if (attempts == 0)
    attempts = jmg->getDefaultIKAttempts();

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = jmg->getDefaultIKTimeout();

  bool first_seed = true;
  for (unsigned int st = 0 ; st < attempts ; ++st)
  {
    bool found_solution = true;
    for (std::size_t sg = 0; sg < sub_groups.size(); ++sg)
    {
      const std::vector<unsigned int>& bij = sub_groups[sg]->getKinematicsSolverJointBijection();
      std::vector<double> seed(bij.size());
       // the first seed is the initial state
      if (first_seed)
      {
        std::vector<double> initial_values;
        copyJointGroupPositions(sub_groups[sg], initial_values);
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          seed[bij[i]] = initial_values[i];
      }
      else
      {
        // sample a random seed
        random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
        std::vector<double> random_values;
        sub_groups[sg]->getVariableRandomValues(rng, random_values);
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          seed[bij[i]] = random_values[i];
      }

      // compute the IK solution
      std::vector<double> ik_sol;
      moveit_msgs::MoveItErrorCodes error;
      const std::vector<double> &climits = consistency_limits.empty() ? std::vector<double>() : consistency_limits[sg];
      if (solvers[sg]->searchPositionIK(ik_queries[sg], seed, timeout, climits, ik_sol, error))
      {
        std::vector<double> solution(bij.size());
        for (std::size_t i = 0 ; i < bij.size() ; ++i)
          solution[i] = ik_sol[bij[i]];
        setJointGroupPositions(sub_groups[sg], solution);
      }
      else
      {
        found_solution = false;
        break;
      }
      logDebug("IK attempt: %d of %d", st, attempts);
    }
    if (found_solution)
    {
      std::vector<double> full_solution;
      copyJointGroupPositions(jmg, full_solution);
      if (constraint ? constraint(jmg, &full_solution[0]) : true)
      {
        logDebug("Found IK solution");
        return true;
      }
    }
  }
  
  return false;
}

namespace
{
static inline void updateAABB(const Eigen::Affine3d &t, const Eigen::Vector3d &e, std::vector<double> &aabb)
{
  Eigen::Vector3d v = e / 2.0;
  Eigen::Vector3d c2 = t * v;
  v = -v;
  Eigen::Vector3d c1 = t * v;
  if (aabb.empty())
  {
    aabb.resize(6);
    aabb[0] = c1.x();
    aabb[2] = c1.y();
    aabb[4] = c1.z();
    aabb[1] = c2.x();
    aabb[3] = c2.y();
    aabb[5] = c2.z();
  }
  else
  {
    if (aabb[0] > c1.x())
      aabb[0] = c1.x();
    if (aabb[2] > c1.y())
      aabb[2] = c1.y();
    if (aabb[4] > c1.z())
      aabb[4] = c1.z();
    if (aabb[1] < c2.x())
      aabb[1] = c2.x();
    if (aabb[3] < c2.y())
      aabb[3] = c2.y();
    if (aabb[5] < c2.z())
      aabb[5] = c2.z();
  }
}
}

void robot_state::RobotState::computeAABB(std::vector<double> &aabb) const
{
  aabb.clear();
  std::vector<const LinkModel*> links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    const Eigen::Affine3d &t = getGlobalLinkTransform(links[i]);
    const Eigen::Vector3d &e = links[i]->getShapeExtentsAtOrigin();
    updateAABB(t, e, aabb);
  }
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
  {
    const EigenSTL::vector_Affine3d &ts = it->second->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr> &ss = it->second->getShapes();
    for (std::size_t i = 0 ; i < ts.size() ; ++i)
    {
      Eigen::Vector3d e = shapes::computeShapeExtents(ss[i].get());
      updateAABB(ts[i], e, aabb);
    }
  }
  if (aabb.empty())
    aabb.resize(6, 0.0);
}


void moveit::core::RobotState::printStateInfo(std::ostream &out) const
{
  out << "Robot State @" << this << std::endl;
  out << "  * Memory is available for" << (position_ ? " position" : "")
      << (velocity_ ? " velocity" : "") <<  (acceleration_ ? " acceleration" : "")
      << (variable_joint_transforms_ ? " transforms" : "") << std::endl;
  out << "  * Called new[] for" << (position_ ? " position" : "")
      << (called_new_for_ & ALLOC_VELOCITY ? " velocity" : "")
      << (called_new_for_ & ALLOC_ACCELERATION ? " acceleration" : "")
      << (transforms_.size() > 0 ? " transforms" : "") << std::endl;
  
  std::size_t n = robot_model_->getVariableCount();
  if (position_)
  {
    out << "  * Position: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << position_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Position: NULL" << std::endl;
  
  if (velocity_)
  {
    out << "  * Velocity: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << velocity_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Velocity: NULL" << std::endl;

  if (acceleration_)
  {
    out << "  * Acceleration: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << acceleration_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Acceleration: NULL" << std::endl;
  
  out << "  * Dirty FK: " << (dirty_joint_transforms_ ? dirty_joint_transforms_->getName() : "NULL") << std::endl;
  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL") << std::endl;
  out << "  * Dirty Collision Body Transforms: " << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;
  
  printTransforms(out);
}

void moveit::core::RobotState::printTransform(const Eigen::Affine3d &transform, std::ostream &out) const
{
  Eigen::Quaterniond q(transform.rotation());  
  out << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", " << transform.translation().z() << "], Q.xyzw = ["
      << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" << std::endl;
}

void moveit::core::RobotState::printTransforms(std::ostream &out) const
{
  if (!variable_joint_transforms_)
  {
    out << "No transforms computed" << std::endl;
    return;
  }
  
  out << "Joint transforms:" << std::endl;
  const std::vector<const JointModel*> &jm = robot_model_->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    out << "  " << jm[i]->getName() << ": ";
    printTransform(variable_joint_transforms_[jm[i]->getJointIndex()], out);
  }
  
  out << "Link poses:" << std::endl;
  const std::vector<const LinkModel*> &lm = robot_model_->getLinkModels();
  for (std::size_t i = 0 ; i < lm.size() ; ++i)
  {
    out << "  " << lm[i]->getName() << ": ";
    printTransform(global_link_transforms_[lm[i]->getLinkIndex()], out);
  }
}

std::string moveit::core::RobotState::getStateTreeString(const std::string& prefix) const
{
  std::stringstream ss;
  ss << "ROBOT: " << robot_model_->getName() << std::endl;
  getStateTreeJointString(ss, robot_model_->getRootJoint(), "   ", true);
  return ss.str();
}

namespace
{
void getPoseString(std::ostream& ss, const Eigen::Affine3d& pose, const std::string& pfx)
{
  ss.precision(3);
  for (int y = 0 ; y < 4 ; ++y)
  {
    ss << pfx;
    for (int x = 0 ; x < 4 ; ++x)
    {
      ss << std::setw(8) << pose(y, x) << " ";
    }
    ss << std::endl;
  }
}
}

void moveit::core::RobotState::getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const
{
  std::string pfx = pfx0 + "+--";
  
  ss << pfx << "Joint: " << jm->getName() << std::endl;

  pfx = pfx0 + (last ? "   " : "|  ");

  for (std::size_t i = 0 ; i < jm->getVariableCount(); ++i)
  {
    ss.precision(3);
    ss << pfx << jm->getVariableNames()[i] << std::setw(12) << position_[jm->getFirstVariableIndex() + i] << std::endl;
  }

  const LinkModel* lm = jm->getChildLinkModel();

  ss << pfx << "Link: " << lm->getName() << std::endl;
  getPoseString(ss, lm->getJointOriginTransform(), pfx + "joint_origin:");
  if (variable_joint_transforms_)
  {
    getPoseString(ss, variable_joint_transforms_[jm->getJointIndex()], pfx + "joint_variable:");
    getPoseString(ss, global_link_transforms_[lm->getLinkIndex()], pfx + "link_global:");
  }
  
  for (std::vector<const JointModel*>::const_iterator it = lm->getChildJointModels().begin() ; it != lm->getChildJointModels().end() ; ++it)
    getStateTreeJointString(ss, *it, pfx, it + 1 == lm->getChildJointModels().end());
}


std::ostream& moveit::core::operator<<(std::ostream &out, const RobotState &s)
{
  s.printStateInfo(out);
  return out;  
}
