/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Ioan Sucan, Sachin Chitta, Acorn Pooley, Mario Prats, Dave Coleman */

#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/backtrace/backtrace.h>
#include <moveit/profiler/profiler.h>
#include <boost/bind.hpp>
#include <moveit/robot_model/aabb.h>

namespace moveit
{
namespace core
{
/** \brief It is recommended that there are at least 10 steps per trajectory
 * for testing jump thresholds with computeCartesianPath. With less than 10 steps
 * it is difficult to choose a jump_threshold parameter that effectively separates
 * valid paths from paths with large joint space jumps. */
static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;

const std::string LOGNAME = "robot_state";

RobotState::RobotState(const RobotModelConstPtr& robot_model)
  : robot_model_(robot_model)
  , has_velocity_(false)
  , has_acceleration_(false)
  , has_effort_(false)
  , dirty_link_transforms_(robot_model_->getRootJoint())
  , dirty_collision_body_transforms_(nullptr)
  , rng_(nullptr)
{
  allocMemory();

  // all transforms are dirty initially
  const int nr_doubles_for_dirty_joint_transforms =
      1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
  memset(dirty_joint_transforms_, 1, sizeof(double) * nr_doubles_for_dirty_joint_transforms);
}

RobotState::RobotState(const RobotState& other) : rng_(nullptr)
{
  robot_model_ = other.robot_model_;
  allocMemory();
  copyFrom(other);
}

RobotState::~RobotState()
{
  clearAttachedBodies();
  free(memory_);
  if (rng_)
    delete rng_;
}

void RobotState::allocMemory()
{
  // memory for the dirty joint transforms
  const int nr_doubles_for_dirty_joint_transforms =
      1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
  const size_t bytes =
      sizeof(Eigen::Affine3d) * (robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() +
                                 robot_model_->getLinkGeometryCount()) +
      sizeof(double) * (robot_model_->getVariableCount() * 3 + nr_doubles_for_dirty_joint_transforms) + 15;
  memory_ = malloc(bytes);

  // make the memory for transforms align at 16 bytes
  variable_joint_transforms_ = reinterpret_cast<Eigen::Affine3d*>(((uintptr_t)memory_ + 15) & ~(uintptr_t)0x0F);
  global_link_transforms_ = variable_joint_transforms_ + robot_model_->getJointModelCount();
  global_collision_body_transforms_ = global_link_transforms_ + robot_model_->getLinkModelCount();
  dirty_joint_transforms_ =
      reinterpret_cast<unsigned char*>(global_collision_body_transforms_ + robot_model_->getLinkGeometryCount());
  position_ = reinterpret_cast<double*>(dirty_joint_transforms_) + nr_doubles_for_dirty_joint_transforms;
  velocity_ = position_ + robot_model_->getVariableCount();
  // acceleration and effort share the memory (not both can be specified)
  effort_ = acceleration_ = velocity_ + robot_model_->getVariableCount();
}

RobotState& RobotState::operator=(const RobotState& other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void RobotState::copyFrom(const RobotState& other)
{
  has_velocity_ = other.has_velocity_;
  has_acceleration_ = other.has_acceleration_;
  has_effort_ = other.has_effort_;

  dirty_collision_body_transforms_ = other.dirty_collision_body_transforms_;
  dirty_link_transforms_ = other.dirty_link_transforms_;

  if (dirty_link_transforms_ == robot_model_->getRootJoint())
  {
    // everything is dirty; no point in copying transforms; copy positions, potentially velocity & acceleration
    memcpy(position_, other.position_, robot_model_->getVariableCount() * sizeof(double) *
                                           (1 + ((has_velocity_ || has_acceleration_ || has_effort_) ? 1 : 0) +
                                            ((has_acceleration_ || has_effort_) ? 1 : 0)));

    // mark all transforms as dirty
    const int nr_doubles_for_dirty_joint_transforms =
        1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
    memset(dirty_joint_transforms_, 1, sizeof(double) * nr_doubles_for_dirty_joint_transforms);
  }
  else
  {
    // copy all the memory; maybe avoid copying velocity and acceleration if possible
    const int nr_doubles_for_dirty_joint_transforms =
        1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
    const size_t bytes =
        sizeof(Eigen::Affine3d) * (robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() +
                                   robot_model_->getLinkGeometryCount()) +
        sizeof(double) *
            (robot_model_->getVariableCount() * (1 + ((has_velocity_ || has_acceleration_ || has_effort_) ? 1 : 0) +
                                                 ((has_acceleration_ || has_effort_) ? 1 : 0)) +
             nr_doubles_for_dirty_joint_transforms);
    memcpy(variable_joint_transforms_, other.variable_joint_transforms_, bytes);
  }

  // copy attached bodies
  clearAttachedBodies();
  for (std::map<std::string, AttachedBody*>::const_iterator it = other.attached_body_map_.begin();
       it != other.attached_body_map_.end(); ++it)
    attachBody(it->second->getName(), it->second->getShapes(), it->second->getFixedTransforms(),
               it->second->getTouchLinks(), it->second->getAttachedLinkName(), it->second->getDetachPosture());
}

bool RobotState::checkJointTransforms(const JointModel* joint) const
{
  if (dirtyJointTransform(joint))
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty joint transforms for joint '%s'", joint->getName().c_str());
    return false;
  }
  return true;
}

bool RobotState::checkLinkTransforms() const
{
  if (dirtyLinkTransforms())
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty link transforms");
    return false;
  }
  return true;
}

bool RobotState::checkCollisionTransforms() const
{
  if (dirtyCollisionBodyTransforms())
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty collision body transforms");
    return false;
  }
  return true;
}

void RobotState::markVelocity()
{
  if (!has_velocity_)
  {
    has_velocity_ = true;
    memset(velocity_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::markAcceleration()
{
  if (!has_acceleration_)
  {
    has_acceleration_ = true;
    has_effort_ = false;
    memset(acceleration_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::markEffort()
{
  if (!has_effort_)
  {
    has_acceleration_ = false;
    has_effort_ = true;
    memset(effort_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::setToRandomPositions()
{
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  robot_model_->getVariableRandomPositions(rng, position_);
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
  // mimic values are correctly set in RobotModel
}

void RobotState::setToRandomPositions(const JointModelGroup* group)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  setToRandomPositions(group, rng);
}
void RobotState::setToRandomPositions(const JointModelGroup* group, random_numbers::RandomNumberGenerator& rng)
{
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
    joints[i]->getVariableRandomPositions(rng, position_ + joints[i]->getFirstVariableIndex());
  updateMimicJoints(group);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& near,
                                            const std::vector<double>& distances)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  assert(distances.size() == joints.size());
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    const int idx = joints[i]->getFirstVariableIndex();
    joints[i]->getVariableRandomPositionsNearBy(rng, position_ + joints[i]->getFirstVariableIndex(),
                                                near.position_ + idx, distances[i]);
  }
  updateMimicJoints(group);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& near, double distance)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    const int idx = joints[i]->getFirstVariableIndex();
    joints[i]->getVariableRandomPositionsNearBy(rng, position_ + joints[i]->getFirstVariableIndex(),
                                                near.position_ + idx, distance);
  }
  updateMimicJoints(group);
}

bool RobotState::setToDefaultValues(const JointModelGroup* group, const std::string& name)
{
  std::map<std::string, double> m;
  bool r = group->getVariableDefaultPositions(name, m);  // mimic values are updated
  setVariablePositions(m);
  return r;
}

void RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultPositions(position_);  // mimic values are updated
  // set velocity & acceleration to 0
  memset(velocity_, 0, sizeof(double) * 2 * robot_model_->getVariableCount());
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const double* position)
{
  // assume everything is in order in terms of array lengths (for efficiency reasons)
  memcpy(position_, position, robot_model_->getVariableCount() * sizeof(double));

  // the full state includes mimic joint values, so no need to update mimic here

  // Since all joint values have potentially changed, we will need to recompute all transforms
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map)
{
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end(); it != end;
       ++it)
  {
    const int index = robot_model_->getVariableIndex(it->first);
    position_[index] = it->second;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::getMissingKeys(const std::map<std::string, double>& variable_map,
                                std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (std::size_t i = 0; i < nm.size(); ++i)
    if (variable_map.find(nm[i]) == variable_map.end())
      if (robot_model_->getJointOfVariable(nm[i])->getMimic() == nullptr)
        missing_variables.push_back(nm[i]);
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map,
                                      std::vector<std::string>& missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariablePositions(const std::vector<std::string>& variable_names,
                                      const std::vector<double>& variable_position)
{
  for (std::size_t i = 0; i < variable_names.size(); ++i)
  {
    const int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map)
{
  markVelocity();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end(); it != end;
       ++it)
    velocity_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map,
                                       std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableVelocities(const std::vector<std::string>& variable_names,
                                       const std::vector<double>& variable_velocity)
{
  markVelocity();
  assert(variable_names.size() == variable_velocity.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map)
{
  markAcceleration();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end(); it != end;
       ++it)
    acceleration_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map,
                                          std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableAccelerations(const std::vector<std::string>& variable_names,
                                          const std::vector<double>& variable_acceleration)
{
  markAcceleration();
  assert(variable_names.size() == variable_acceleration.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map)
{
  markEffort();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end(); it != end;
       ++it)
    acceleration_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map,
                                   std::vector<std::string>& missing_variables)
{
  setVariableEffort(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableEffort(const std::vector<std::string>& variable_names,
                                   const std::vector<double>& variable_effort)
{
  markEffort();
  assert(variable_names.size() == variable_effort.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    effort_[robot_model_->getVariableIndex(variable_names[i])] = variable_effort[i];
}

void RobotState::invertVelocity()
{
  if (has_velocity_)
  {
    for (size_t i = 0; i < robot_model_->getVariableCount(); ++i)
      velocity_[i] *= -1;
  }
}

void RobotState::setJointEfforts(const JointModel* joint, const double* effort)
{
  if (has_acceleration_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to set joint efforts because array is being used for accelerations");
    return;
  }
  has_effort_ = true;

  memcpy(effort_ + joint->getFirstVariableIndex(), effort, joint->getVariableCount() * sizeof(double));
}

void RobotState::setJointGroupPositions(const JointModelGroup* group, const double* gstate)
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(position_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      position_[il[i]] = gstate[i];
  }
  updateMimicJoints(group);
}

void RobotState::setJointGroupPositions(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    position_[il[i]] = values(i);
  updateMimicJoints(group);
}

void RobotState::copyJointGroupPositions(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, position_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = position_[il[i]];
}

void RobotState::copyJointGroupPositions(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = position_[il[i]];
}

void RobotState::setJointGroupVelocities(const JointModelGroup* group, const double* gstate)
{
  markVelocity();
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(velocity_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      velocity_[il[i]] = gstate[i];
  }
}

void RobotState::setJointGroupVelocities(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  markVelocity();
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    velocity_[il[i]] = values(i);
}

void RobotState::copyJointGroupVelocities(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, velocity_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = velocity_[il[i]];
}

void RobotState::copyJointGroupVelocities(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = velocity_[il[i]];
}

void RobotState::setJointGroupAccelerations(const JointModelGroup* group, const double* gstate)
{
  markAcceleration();
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(acceleration_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      acceleration_[il[i]] = gstate[i];
  }
}

void RobotState::setJointGroupAccelerations(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  markAcceleration();
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    acceleration_[il[i]] = values(i);
}

void RobotState::copyJointGroupAccelerations(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, acceleration_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = acceleration_[il[i]];
}

void RobotState::copyJointGroupAccelerations(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = acceleration_[il[i]];
}

void RobotState::update(bool force)
{
  // make sure we do everything from scratch if needed
  if (force)
  {
    memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
    dirty_link_transforms_ = robot_model_->getRootJoint();
  }

  // this actually triggers all needed updates
  updateCollisionBodyTransforms();
}

void RobotState::updateCollisionBodyTransforms()
{
  if (dirty_link_transforms_ != nullptr)
    updateLinkTransforms();

  if (dirty_collision_body_transforms_ != nullptr)
  {
    const std::vector<const LinkModel*>& links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = nullptr;

    for (std::size_t i = 0; i < links.size(); ++i)
    {
      const EigenSTL::vector_Affine3d& ot = links[i]->getCollisionOriginTransforms();
      const std::vector<int>& ot_id = links[i]->areCollisionOriginTransformsIdentity();
      const int index_co = links[i]->getFirstCollisionBodyTransformIndex();
      const int index_l = links[i]->getLinkIndex();
      for (std::size_t j = 0; j < ot.size(); ++j)
        global_collision_body_transforms_[index_co + j].matrix().noalias() =
            ot_id[j] ? global_link_transforms_[index_l].matrix() :
                       global_link_transforms_[index_l].matrix() * ot[j].matrix();
    }
  }
}

void RobotState::updateLinkTransforms()
{
  if (dirty_link_transforms_ != nullptr)
  {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_)
      dirty_collision_body_transforms_ =
          robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    else
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    dirty_link_transforms_ = nullptr;
  }
}

void RobotState::updateLinkTransformsInternal(const JointModel* start)
{
  for (const LinkModel* link : start->getDescendantLinkModels())
  {
    const LinkModel* parent = link->getParentLinkModel();
    if (parent)  // root JointModel will not have a parent
    {
      if (link->parentJointIsFixed())
        global_link_transforms_[link->getLinkIndex()].matrix().noalias() =
            global_link_transforms_[parent->getLinkIndex()].matrix() * link->getJointOriginTransform().matrix();
      else
      {
        if (link->jointOriginTransformIsIdentity())
          global_link_transforms_[link->getLinkIndex()].matrix().noalias() =
              global_link_transforms_[parent->getLinkIndex()].matrix() *
              getJointTransform(link->getParentJointModel()).matrix();
        else
          global_link_transforms_[link->getLinkIndex()].matrix().noalias() =
              global_link_transforms_[parent->getLinkIndex()].matrix() * link->getJointOriginTransform().matrix() *
              getJointTransform(link->getParentJointModel()).matrix();
      }
    }
    else
    {
      if (link->jointOriginTransformIsIdentity())
        global_link_transforms_[link->getLinkIndex()] = getJointTransform(link->getParentJointModel());
      else
        global_link_transforms_[link->getLinkIndex()].matrix().noalias() =
            link->getJointOriginTransform().matrix() * getJointTransform(link->getParentJointModel()).matrix();
    }
  }

  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

void RobotState::updateStateWithLinkAt(const LinkModel* link, const Eigen::Affine3d& transform, bool backward)
{
  updateLinkTransforms();  // no link transforms must be dirty, otherwise the transform we set will be overwritten

  // update the fact that collision body transforms are out of date
  if (dirty_collision_body_transforms_)
    dirty_collision_body_transforms_ =
        robot_model_->getCommonRoot(dirty_collision_body_transforms_, link->getParentJointModel());
  else
    dirty_collision_body_transforms_ = link->getParentJointModel();

  global_link_transforms_[link->getLinkIndex()] = transform;

  // update link transforms for descendant links only (leaving the transform for the current link untouched)
  const std::vector<const JointModel*>& cj = link->getChildJointModels();
  for (std::size_t i = 0; i < cj.size(); ++i)
    updateLinkTransformsInternal(cj[i]);

  // if we also need to go backward
  if (backward)
  {
    const LinkModel* parent_link = link;
    const LinkModel* child_link;
    while (parent_link->getParentJointModel()->getParentLinkModel())
    {
      child_link = parent_link;
      parent_link = parent_link->getParentJointModel()->getParentLinkModel();

      // update the transform of the parent
      global_link_transforms_[parent_link->getLinkIndex()] =
          global_link_transforms_[child_link->getLinkIndex()] *
          (child_link->getJointOriginTransform() *
           variable_joint_transforms_[child_link->getParentJointModel()->getJointIndex()])
              .inverse(Eigen::Isometry);

      // update link transforms for descendant links only (leaving the transform for the current link untouched)
      // with the exception of the child link we are coming backwards from
      const std::vector<const JointModel*>& cj = parent_link->getChildJointModels();
      for (std::size_t i = 0; i < cj.size(); ++i)
        if (cj[i] != child_link->getParentJointModel())
          updateLinkTransformsInternal(cj[i]);
    }
    // all collision body transforms are invalid now
    dirty_collision_body_transforms_ = parent_link->getParentJointModel();
  }

  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

bool RobotState::satisfiesBounds(double margin) const
{
  const std::vector<const JointModel*>& jm = robot_model_->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;
}

bool RobotState::satisfiesBounds(const JointModelGroup* group, double margin) const
{
  const std::vector<const JointModel*>& jm = group->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;
}

void RobotState::enforceBounds()
{
  const std::vector<const JointModel*>& jm = robot_model_->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
    enforceBounds(jm[i]);
}

void RobotState::enforceBounds(const JointModelGroup* joint_group)
{
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
    enforceBounds(jm[i]);
}

std::pair<double, const JointModel*> RobotState::getMinDistanceToPositionBounds() const
{
  return getMinDistanceToPositionBounds(robot_model_->getActiveJointModels());
}

std::pair<double, const JointModel*> RobotState::getMinDistanceToPositionBounds(const JointModelGroup* group) const
{
  return getMinDistanceToPositionBounds(group->getActiveJointModels());
}

std::pair<double, const JointModel*>
RobotState::getMinDistanceToPositionBounds(const std::vector<const JointModel*>& joints) const
{
  double distance = std::numeric_limits<double>::max();
  const JointModel* index = nullptr;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    if (joints[i]->getType() == JointModel::PLANAR || joints[i]->getType() == JointModel::FLOATING)
      continue;
    if (joints[i]->getType() == JointModel::REVOLUTE)
      if (static_cast<const RevoluteJointModel*>(joints[i])->isContinuous())
        continue;

    const double* joint_values = getJointPositions(joints[i]);
    const JointModel::Bounds& bounds = joints[i]->getVariableBounds();
    std::vector<double> lower_bounds(bounds.size()), upper_bounds(bounds.size());
    for (std::size_t j = 0; j < bounds.size(); ++j)
    {
      lower_bounds[j] = bounds[j].min_position_;
      upper_bounds[j] = bounds[j].max_position_;
    }
    double new_distance = joints[i]->distance(joint_values, &lower_bounds[0]);
    if (new_distance < distance)
    {
      index = joints[i];
      distance = new_distance;
    }
    new_distance = joints[i]->distance(joint_values, &upper_bounds[0]);
    if (new_distance < distance)
    {
      index = joints[i];
      distance = new_distance;
    }
  }
  return std::make_pair(distance, index);
}

bool RobotState::isValidVelocityMove(const RobotState& other, const JointModelGroup* group, double dt) const
{
  const std::vector<const JointModel*>& jm = group->getActiveJointModels();
  for (std::size_t joint_id = 0; joint_id < jm.size(); ++joint_id)
  {
    const int idx = jm[joint_id]->getFirstVariableIndex();
    const std::vector<VariableBounds>& bounds = jm[joint_id]->getVariableBounds();

    // Check velocity for each joint variable
    for (std::size_t var_id = 0; var_id < jm[joint_id]->getVariableCount(); ++var_id)
    {
      const double dtheta = std::abs(*(position_ + idx + var_id) - *(other.getVariablePositions() + idx + var_id));

      if (dtheta > dt * bounds[var_id].max_velocity_)
        return false;
    }
  }

  return true;
}

double RobotState::distance(const RobotState& other, const JointModelGroup* joint_group) const
{
  double d = 0.0;
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    d += jm[i]->getDistanceFactor() * jm[i]->distance(position_ + idx, other.position_ + idx);
  }
  return d;
}

void RobotState::interpolate(const RobotState& to, double t, RobotState& state) const
{
  robot_model_->interpolate(getVariablePositions(), to.getVariablePositions(), t, state.getVariablePositions());

  memset(state.dirty_joint_transforms_, 1, state.robot_model_->getJointModelCount() * sizeof(unsigned char));
  state.dirty_link_transforms_ = state.robot_model_->getRootJoint();
}

void RobotState::interpolate(const RobotState& to, double t, RobotState& state,
                             const JointModelGroup* joint_group) const
{
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    jm[i]->interpolate(position_ + idx, to.position_ + idx, t, state.position_ + idx);
  }
  state.updateMimicJoints(joint_group);
}

void RobotState::setAttachedBodyUpdateCallback(const AttachedBodyCallback& callback)
{
  attached_body_update_callback_ = callback;
}

bool RobotState::hasAttachedBody(const std::string& id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const AttachedBody* RobotState::getAttachedBody(const std::string& id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Attached body '%s' not found", id.c_str());
    return nullptr;
  }
  else
    return it->second;
}

void RobotState::attachBody(AttachedBody* attached_body)
{
  attached_body_map_[attached_body->getName()] = attached_body;
  attached_body->computeTransform(getGlobalLinkTransform(attached_body->getAttachedLink()));
  if (attached_body_update_callback_)
    attached_body_update_callback_(attached_body, true);
}

void RobotState::attachBody(const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Affine3d& attach_trans, const std::set<std::string>& touch_links,
                            const std::string& link, const trajectory_msgs::JointTrajectory& detach_posture)
{
  const LinkModel* l = robot_model_->getLinkModel(link);
  AttachedBody* ab = new AttachedBody(l, id, shapes, attach_trans, touch_links, detach_posture);
  attached_body_map_[id] = ab;
  ab->computeTransform(getGlobalLinkTransform(l));
  if (attached_body_update_callback_)
    attached_body_update_callback_(ab, true);
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
    attached_bodies.push_back(it->second);
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies,
                                   const JointModelGroup* group) const
{
  attached_bodies.clear();
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
    if (group->hasLinkModel(it->second->getAttachedLinkName()))
      attached_bodies.push_back(it->second);
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const LinkModel* lm) const
{
  attached_bodies.clear();
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
    if (it->second->getAttachedLink() == lm)
      attached_bodies.push_back(it->second);
}

void RobotState::clearAttachedBodies()
{
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
  }
  attached_body_map_.clear();
}

void RobotState::clearAttachedBodies(const LinkModel* link)
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

void RobotState::clearAttachedBodies(const JointModelGroup* group)
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

bool RobotState::clearAttachedBody(const std::string& id)
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

const Eigen::Affine3d& RobotState::getFrameTransform(const std::string& id)
{
  updateLinkTransforms();
  return static_cast<const RobotState*>(this)->getFrameTransform(id);
}

const Eigen::Affine3d& RobotState::getFrameTransform(const std::string& id) const
{
  if (!id.empty() && id[0] == '/')
    return getFrameTransform(id.substr(1));
  BOOST_VERIFY(checkLinkTransforms());

  static const Eigen::Affine3d identity_transform = Eigen::Affine3d::Identity();
  if (id.size() + 1 == robot_model_->getModelFrame().size() && '/' + id == robot_model_->getModelFrame())
    return identity_transform;
  if (robot_model_->hasLinkModel(id))
  {
    const LinkModel* lm = robot_model_->getLinkModel(id);
    return global_link_transforms_[lm->getLinkIndex()];
  }
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Transform from frame '%s' to frame '%s' is not known "
                             "('%s' should be a link name or an attached body id).",
                    id.c_str(), robot_model_->getModelFrame().c_str(), id.c_str());
    return identity_transform;
  }
  const EigenSTL::vector_Affine3d& tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Attached body '%s' has no geometry associated to it. No transform to return.",
                    id.c_str());
    return identity_transform;
  }
  if (tf.size() > 1)
    ROS_DEBUG_NAMED(LOGNAME, "There are multiple geometries associated to attached body '%s'. "
                             "Returning the transform for the first one.",
                    id.c_str());
  return tf[0];
}

bool RobotState::knowsFrameTransform(const std::string& id) const
{
  if (!id.empty() && id[0] == '/')
    return knowsFrameTransform(id.substr(1));
  if (robot_model_->hasLinkModel(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && !it->second->getGlobalCollisionBodyTransforms().empty();
}

void RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                                 const std_msgs::ColorRGBA& color, const std::string& ns, const ros::Duration& dur,
                                 bool include_attached) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names, include_attached);
  unsigned int id = cur_num;
  for (std::size_t i = cur_num; i < arr.markers.size(); ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                                 bool include_attached) const
{
  ros::Time tm = ros::Time::now();
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Trying to get marker for link '%s'", link_names[i].c_str());
    const LinkModel* lm = robot_model_->getLinkModel(link_names[i]);
    if (!lm)
      continue;
    if (include_attached)
      for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
           it != attached_body_map_.end(); ++it)
        if (it->second->getAttachedLink() == lm)
        {
          for (std::size_t j = 0; j < it->second->getShapes().size(); ++j)
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

    for (std::size_t j = 0; j < lm->getShapes().size(); ++j)
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
        tf::poseEigenToMsg(global_collision_body_transforms_[lm->getFirstCollisionBodyTransformIndex() + j], mark.pose);
      }
      else
      {
        mark.type = mark.MESH_RESOURCE;
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = mesh_resource;
        const Eigen::Vector3d& mesh_scale = lm->getVisualMeshScale();

        mark.scale.x = mesh_scale[0];
        mark.scale.y = mesh_scale[1];
        mark.scale.z = mesh_scale[2];
        tf::poseEigenToMsg(global_link_transforms_[lm->getLinkIndex()] * lm->getVisualMeshOrigin(), mark.pose);
      }

      arr.markers.push_back(mark);
    }
  }
}

Eigen::MatrixXd RobotState::getJacobian(const JointModelGroup* group,
                                        const Eigen::Vector3d& reference_point_position) const
{
  Eigen::MatrixXd result;
  if (!getJacobian(group, group->getLinkModels().back(), reference_point_position, result, false))
    throw Exception("Unable to compute Jacobian");
  return result;
}

bool RobotState::getJacobian(const JointModelGroup* group, const LinkModel* link,
                             const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                             bool use_quaternion_representation) const
{
  BOOST_VERIFY(checkLinkTransforms());

  if (!group->isChain())
  {
    ROS_ERROR_NAMED(LOGNAME, "The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
                    link->getName().c_str(), group->getName().c_str());
    return false;
  }

  const robot_model::JointModel* root_joint_model = group->getJointModels()[0];  // group->getJointRoots()[0];
  const robot_model::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
  Eigen::Affine3d reference_transform =
      root_link_model ? getGlobalLinkTransform(root_link_model).inverse(Eigen::Isometry) : Eigen::Affine3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  Eigen::Affine3d link_transform = reference_transform * getGlobalLinkTransform(link);
  Eigen::Vector3d point_transform = link_transform * reference_point_position;

  /*
  ROS_DEBUG_NAMED(LOGNAME, "Point from reference origin expressed in world coordinates: %f %f %f",
           point_transform.x(),
           point_transform.y(),
           point_transform.z());
  */

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while (link)
  {
    /*
    ROS_DEBUG_NAMED(LOGNAME, "Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());
    ROS_DEBUG_NAMED(LOGNAME, "Joint: %s",link_state->getParentJointState()->getName().c_str());
    */
    const JointModel* pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      if (not group->hasJointModel(pjm->getName()))
      {
        link = pjm->getParentLinkModel();
        continue;
      }
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      if (pjm->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform * getGlobalLinkTransform(link);
        joint_axis = joint_transform.linear() * static_cast<const robot_model::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) =
            jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
      {
        joint_transform = reference_transform * getGlobalLinkTransform(link);
        joint_axis = joint_transform.rotation() * static_cast<const robot_model::PrismaticJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PLANAR)
      {
        joint_transform = reference_transform * getGlobalLinkTransform(link);
        joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
        jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
        jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                   joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
      }
      else
        ROS_ERROR_NAMED(LOGNAME, "Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation)
  {  // Quaternion representation
    // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
    // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond q(link_transform.linear());
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::MatrixXd quaternion_update_matrix(4, 3);
    quaternion_update_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    jacobian.block(3, 0, 4, columns) = 0.5 * quaternion_update_matrix * jacobian.block(3, 0, 3, columns);
  }
  return true;
}

bool RobotState::setFromDiffIK(const JointModelGroup* jmg, const Eigen::VectorXd& twist, const std::string& tip,
                               double dt, const GroupStateValidityCallbackFn& constraint)
{
  Eigen::VectorXd qdot;
  computeVariableVelocity(jmg, qdot, twist, getLinkModel(tip));
  return integrateVariableVelocity(jmg, qdot, dt, constraint);
}

bool RobotState::setFromDiffIK(const JointModelGroup* jmg, const geometry_msgs::Twist& twist, const std::string& tip,
                               double dt, const GroupStateValidityCallbackFn& constraint)
{
  Eigen::Matrix<double, 6, 1> t;
  tf::twistMsgToEigen(twist, t);
  return setFromDiffIK(jmg, t, tip, dt, constraint);
}

void RobotState::computeVariableVelocity(const JointModelGroup* jmg, Eigen::VectorXd& qdot,
                                         const Eigen::VectorXd& twist, const LinkModel* tip) const
{
  // Get the Jacobian of the group at the current configuration
  Eigen::MatrixXd J(6, jmg->getVariableCount());
  Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
  getJacobian(jmg, tip, reference_point, J, false);

  // Rotate the jacobian to the end-effector frame
  Eigen::Affine3d eMb = getGlobalLinkTransform(tip).inverse(Eigen::Isometry);
  Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
  eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
  J = eWb * J;

  // Do the Jacobian moore-penrose pseudo-inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd U = svdOfJ.matrixU();
  const Eigen::MatrixXd V = svdOfJ.matrixV();
  const Eigen::VectorXd S = svdOfJ.singularValues();

  Eigen::VectorXd Sinv = S;
  static const double pinvtoler = std::numeric_limits<float>::epsilon();
  double maxsv = 0.0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
    if (fabs(S(i)) > maxsv)
      maxsv = fabs(S(i));
  for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
  {
    // Those singular values smaller than a percentage of the maximum singular value are removed
    if (fabs(S(i)) > maxsv * pinvtoler)
      Sinv(i) = 1.0 / S(i);
    else
      Sinv(i) = 0.0;
  }
  Eigen::MatrixXd Jinv = (V * Sinv.asDiagonal() * U.transpose());

  // Compute joint velocity
  qdot = Jinv * twist;
}

bool RobotState::integrateVariableVelocity(const JointModelGroup* jmg, const Eigen::VectorXd& qdot, double dt,
                                           const GroupStateValidityCallbackFn& constraint)
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
    return constraint(this, jmg, &values[0]);
  }
  else
    return true;
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const geometry_msgs::Pose& pose, unsigned int attempts,
                           double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
    return false;
  }
  return setFromIK(jmg, pose, solver->getTipFrame(), attempts, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const geometry_msgs::Pose& pose, const std::string& tip,
                           unsigned int attempts, double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, mat, tip, consistency_limits, attempts, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Affine3d& pose, unsigned int attempts,
                           double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
    return false;
  }
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose, solver->getTipFrame(), consistency_limits, attempts, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Affine3d& pose_in, const std::string& tip_in,
                           unsigned int attempts, double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose_in, tip_in, consistency_limits, attempts, timeout, constraint, options);
}

namespace
{
bool ikCallbackFnAdapter(RobotState* state, const JointModelGroup* group,
                         const GroupStateValidityCallbackFn& constraint, const geometry_msgs::Pose& /*unused*/,
                         const std::vector<double>& ik_sol, moveit_msgs::MoveItErrorCodes& error_code)
{
  const std::vector<unsigned int>& bij = group->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[bij[i]] = ik_sol[i];
  if (constraint(state, group, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return true;
}
}

bool RobotState::setToIKSolverFrame(Eigen::Affine3d& pose, const kinematics::KinematicsBaseConstPtr& solver)
{
  return setToIKSolverFrame(pose, solver->getBaseFrame());
}

bool RobotState::setToIKSolverFrame(Eigen::Affine3d& pose, const std::string& ik_frame)
{
  // Bring the pose to the frame of the IK solver
  if (!Transforms::sameFrame(ik_frame, robot_model_->getModelFrame()))
  {
    const LinkModel* lm = getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!lm)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "IK frame '" << ik_frame << "' does not exist.");
      return false;
    }
    pose = getGlobalLinkTransform(lm).inverse(Eigen::Isometry) * pose;
  }
  return true;
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Affine3d& pose_in, const std::string& tip_in,
                           const std::vector<double>& consistency_limits_in, unsigned int attempts, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  // Convert from single pose and tip to vectors
  EigenSTL::vector_Affine3d poses;
  poses.push_back(pose_in);

  std::vector<std::string> tips;
  tips.push_back(tip_in);

  std::vector<std::vector<double> > consistency_limits;
  consistency_limits.push_back(consistency_limits_in);

  return setFromIK(jmg, poses, tips, consistency_limits, attempts, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const EigenSTL::vector_Affine3d& poses_in,
                           const std::vector<std::string>& tips_in, unsigned int attempts, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  static const std::vector<std::vector<double> > consistency_limits;
  return setFromIK(jmg, poses_in, tips_in, consistency_limits, attempts, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const EigenSTL::vector_Affine3d& poses_in,
                           const std::vector<std::string>& tips_in,
                           const std::vector<std::vector<double> >& consistency_limit_sets, unsigned int attempts,
                           double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  // Error check
  if (poses_in.size() != tips_in.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of poses must be the same as number of tips");
    return false;
  }

  // Load solver
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();

  // Check if this jmg has a solver
  bool valid_solver = true;
  if (!solver)
  {
    valid_solver = false;
  }
  // Check if this jmg's IK solver can handle multiple tips (non-chain solver)
  else if (poses_in.size() > 1)
  {
    std::string error_msg;
    if (!solver->supportsGroup(jmg, &error_msg))
    {
      ROS_ERROR_NAMED(LOGNAME, "Kinematics solver %s does not support joint group %s.  Error: %s",
                      typeid(*solver).name(), jmg->getName().c_str(), error_msg.c_str());
      valid_solver = false;
    }
  }

  if (!valid_solver)
  {
    // Check if there are subgroups that can solve this for us (non-chains)
    if (poses_in.size() > 1)
    {
      // Forward to setFromIKSubgroups() to allow different subgroup IK solvers to work together
      return setFromIKSubgroups(jmg, poses_in, tips_in, consistency_limit_sets, attempts, timeout, constraint, options);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
      return false;
    }
  }

  // Check that no, or only one set of consistency limits have been passed in, and choose that one
  std::vector<double> consistency_limits;
  if (consistency_limit_sets.size() > 1)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid number (%zu) of sets of consistency limits for a setFromIK request "
                             "that is being solved by a single IK solver",
                    consistency_limit_sets.size());
    return false;
  }
  else if (consistency_limit_sets.size() == 1)
    consistency_limits = consistency_limit_sets[0];

  const std::vector<std::string>& solver_tip_frames = solver->getTipFrames();

  // Track which possible tips frames we have filled in so far
  std::vector<bool> tip_frames_used(solver_tip_frames.size(), false);

  // Create vector to hold the output frames in the same order as solver_tip_frames
  std::vector<geometry_msgs::Pose> ik_queries(solver_tip_frames.size());

  // Bring each pose to the frame of the IK solver
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    // Make non-const
    Eigen::Affine3d pose = poses_in[i];
    std::string pose_frame = tips_in[i];

    // Remove extra slash
    if (!pose_frame.empty() && pose_frame[0] == '/')
      pose_frame = pose_frame.substr(1);

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(pose, solver))
      return false;

    // try all of the solver's possible tip frames to see if they uniquely align with any of our passed in pose tip
    // frames
    bool found_valid_frame = false;
    std::size_t solver_tip_id;  // our current index
    for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
    {
      // Check if this tip frame is already accounted for
      if (tip_frames_used[solver_tip_id])
        continue;  // already has a pose

      // check if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
      std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

      // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings
      // more often that we need to
      if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
        solver_tip_frame = solver_tip_frame.substr(1);

      if (pose_frame != solver_tip_frame)
      {
        if (hasAttachedBody(pose_frame))
        {
          const AttachedBody* ab = getAttachedBody(pose_frame);
          const EigenSTL::vector_Affine3d& ab_trans = ab->getFixedTransforms();
          if (ab_trans.size() != 1)
          {
            ROS_ERROR_NAMED(LOGNAME, "Cannot use an attached body "
                                     "with multiple geometries as a reference frame.");
            return false;
          }
          pose_frame = ab->getAttachedLinkName();
          pose = pose * ab_trans[0].inverse(Eigen::Isometry);
        }
        if (pose_frame != solver_tip_frame)
        {
          const robot_model::LinkModel* lm = getLinkModel(pose_frame);
          if (!lm)
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Pose frame '" << pose_frame << "' does not exist.");
            return false;
          }
          const robot_model::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
          for (robot_model::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
            if (Transforms::sameFrame(it->first->getName(), solver_tip_frame))
            {
              pose_frame = solver_tip_frame;
              pose = pose * it->second;
              break;
            }
        }

      }  // end if pose_frame

      // Check if this pose frame works
      if (pose_frame == solver_tip_frame)
      {
        found_valid_frame = true;
        break;
      }

    }  // end for solver_tip_frames

    // Make sure one of the tip frames worked
    if (!found_valid_frame)
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot compute IK for query %zu pose reference frame '%s'", i, pose_frame.c_str());
      // Debug available tip frames
      std::stringstream ss;
      for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
        ss << solver_tip_frames[solver_tip_id] << ", ";
      ROS_ERROR_NAMED(LOGNAME, "Available tip frames: [%s]", ss.str().c_str());
      return false;
    }

    // Remove that tip from the list of available tip frames because each can only have one pose
    tip_frames_used[solver_tip_id] = true;

    // Convert Eigen pose to geometry_msgs pose
    geometry_msgs::Pose ik_query;
    tf::poseEigenToMsg(pose, ik_query);

    // Save into vectors
    ik_queries[solver_tip_id] = ik_query;
  }  // end for poses_in

  // Create poses for all remaining tips a solver expects, even if not passed into this function
  for (std::size_t solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
  {
    // Check if this tip frame is already accounted for
    if (tip_frames_used[solver_tip_id])
      continue;  // already has a pose

    // Process this tip
    std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

    // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more
    // often that we need to
    if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
      solver_tip_frame = solver_tip_frame.substr(1);

    // Get the pose of a different EE tip link
    Eigen::Affine3d current_pose = getGlobalLinkTransform(solver_tip_frame);

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(current_pose, solver))
      return false;

    // Convert Eigen pose to geometry_msgs pose
    geometry_msgs::Pose ik_query;
    tf::poseEigenToMsg(current_pose, ik_query);

    // Save into vectors - but this needs to be ordered in the same order as the IK solver expects its tip frames
    ik_queries[solver_tip_id] = ik_query;

    // Remove that tip from the list of available tip frames because each can only have one pose
    tip_frames_used[solver_tip_id] = true;
  }

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = jmg->getDefaultIKTimeout();

  if (attempts == 0)
    attempts = jmg->getDefaultIKAttempts();

  // set callback function
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&ikCallbackFnAdapter, this, jmg, constraint, _1, _2, _3);

  // Bijection
  const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();

  bool first_seed = true;
  std::vector<double> initial_values;
  for (unsigned int st = 0; st < attempts; ++st)
  {
    std::vector<double> seed(bij.size());

    // the first seed is the current robot state joint values
    if (first_seed)
    {
      first_seed = false;
      copyJointGroupPositions(jmg, initial_values);
      for (std::size_t i = 0; i < bij.size(); ++i)
        seed[i] = initial_values[bij[i]];
    }
    else
    {
      ROS_DEBUG_NAMED(LOGNAME, "Rerunning IK solver with random joint positions");

      // sample a random seed
      random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
      std::vector<double> random_values;
      jmg->getVariableRandomPositions(rng, random_values);
      for (std::size_t i = 0; i < bij.size(); ++i)
        seed[i] = random_values[bij[i]];

      if (options.lock_redundant_joints)
      {
        std::vector<unsigned int> red_joints;
        solver->getRedundantJoints(red_joints);
        copyJointGroupPositions(jmg, initial_values);
        for (std::size_t i = 0; i < red_joints.size(); ++i)
          seed[red_joints[i]] = initial_values[bij[red_joints[i]]];
      }
    }

    // compute the IK solution
    std::vector<double> ik_sol;
    moveit_msgs::MoveItErrorCodes error;

    if (solver->searchPositionIK(ik_queries, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error, options,
                                 this))
    {
      std::vector<double> solution(bij.size());
      for (std::size_t i = 0; i < bij.size(); ++i)
        solution[bij[i]] = ik_sol[i];
      setJointGroupPositions(jmg, solution);
      return true;
    }
  }
  return false;
}

bool RobotState::setFromIKSubgroups(const JointModelGroup* jmg, const EigenSTL::vector_Affine3d& poses_in,
                                    const std::vector<std::string>& tips_in,
                                    const std::vector<std::vector<double> >& consistency_limits, unsigned int attempts,
                                    double timeout, const GroupStateValidityCallbackFn& constraint,
                                    const kinematics::KinematicsQueryOptions& options)
{
  // Assume we have already ran setFromIK() and those checks

  // Get containing subgroups
  std::vector<const JointModelGroup*> sub_groups;
  jmg->getSubgroups(sub_groups);

  // Error check
  if (poses_in.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of poses (%zu) must be the same as number of sub-groups (%zu)", poses_in.size(),
                    sub_groups.size());
    return false;
  }

  if (tips_in.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of tip names (%zu) must be same as number of sub-groups (%zu)", tips_in.size(),
                    sub_groups.size());
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of consistency limit vectors must be the same as number of sub-groups");
    return false;
  }

  for (std::size_t i = 0; i < consistency_limits.size(); ++i)
  {
    if (consistency_limits[i].size() != sub_groups[i]->getVariableCount())
    {
      ROS_ERROR_NAMED(LOGNAME, "Number of joints in consistency_limits is %zu but it should be should be %u", i,
                      sub_groups[i]->getVariableCount());
      return false;
    }
  }

  // Populate list of kin solvers for the various subgroups
  std::vector<kinematics::KinematicsBaseConstPtr> solvers;
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    kinematics::KinematicsBaseConstPtr solver = sub_groups[i]->getSolverInstance();
    if (!solver)
    {
      ROS_ERROR_NAMED(LOGNAME, "Could not find solver for group '%s'", sub_groups[i]->getName().c_str());
      return false;
    }
    solvers.push_back(solver);
  }

  // Make non-const versions
  EigenSTL::vector_Affine3d transformed_poses = poses_in;
  std::vector<std::string> pose_frames = tips_in;

  // Each each pose's tip frame naming
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    Eigen::Affine3d& pose = transformed_poses[i];
    std::string& pose_frame = pose_frames[i];

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(pose, solvers[i]))
      return false;

    // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
    std::string solver_tip_frame = solvers[i]->getTipFrame();

    // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more
    // often that we need to
    if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
      solver_tip_frame = solver_tip_frame.substr(1);

    if (pose_frame != solver_tip_frame)
    {
      if (hasAttachedBody(pose_frame))
      {
        const AttachedBody* ab = getAttachedBody(pose_frame);
        const EigenSTL::vector_Affine3d& ab_trans = ab->getFixedTransforms();
        if (ab_trans.size() != 1)
        {
          ROS_ERROR_NAMED(LOGNAME, "Cannot use an attached body with multiple geometries as a reference frame.");
          return false;
        }
        pose_frame = ab->getAttachedLinkName();
        pose = pose * ab_trans[0].inverse(Eigen::Isometry);
      }
      if (pose_frame != solver_tip_frame)
      {
        const robot_model::LinkModel* lm = getLinkModel(pose_frame);
        if (!lm)
          return false;
        const robot_model::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
        for (robot_model::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
          if (it->first->getName() == solver_tip_frame)
          {
            pose_frame = solver_tip_frame;
            pose = pose * it->second;
            break;
          }
      }
    }

    if (pose_frame != solver_tip_frame)
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot compute IK for query pose reference frame '%s', desired: '%s'",
                      pose_frame.c_str(), solver_tip_frame.c_str());
      return false;
    }
  }

  // Convert Eigen poses to geometry_msg format
  std::vector<geometry_msgs::Pose> ik_queries(poses_in.size());
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = boost::bind(&ikCallbackFnAdapter, this, jmg, constraint, _1, _2, _3);

  for (std::size_t i = 0; i < transformed_poses.size(); ++i)
  {
    Eigen::Quaterniond quat(transformed_poses[i].linear());
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
  for (unsigned int st = 0; st < attempts; ++st)
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
        for (std::size_t i = 0; i < bij.size(); ++i)
          seed[i] = initial_values[bij[i]];
      }
      else
      {
        // sample a random seed
        random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
        std::vector<double> random_values;
        sub_groups[sg]->getVariableRandomPositions(rng, random_values);
        for (std::size_t i = 0; i < bij.size(); ++i)
          seed[i] = random_values[bij[i]];
      }

      // compute the IK solution
      std::vector<double> ik_sol;
      moveit_msgs::MoveItErrorCodes error;
      const std::vector<double>& climits = consistency_limits.empty() ? std::vector<double>() : consistency_limits[sg];
      if (solvers[sg]->searchPositionIK(ik_queries[sg], seed, timeout, climits, ik_sol, error))
      {
        std::vector<double> solution(bij.size());
        for (std::size_t i = 0; i < bij.size(); ++i)
          solution[bij[i]] = ik_sol[i];
        setJointGroupPositions(sub_groups[sg], solution);
      }
      else
      {
        found_solution = false;
        break;
      }
      ROS_DEBUG_NAMED(LOGNAME, "IK attempt: %d of %d", st, attempts);
    }
    if (found_solution)
    {
      std::vector<double> full_solution;
      copyJointGroupPositions(jmg, full_solution);
      if (constraint ? constraint(this, jmg, &full_solution[0]) : true)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Found IK solution");
        return true;
      }
    }
  }
  return false;
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const Eigen::Vector3d& direction,
                                        bool global_reference_frame, double distance, const MaxEEFStep& max_step,
                                        const JumpThreshold& jump_threshold,
                                        const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  // this is the Cartesian pose we start from, and have to move in the direction indicated
  const Eigen::Affine3d& start_pose = getGlobalLinkTransform(link);

  // the direction can be in the local reference frame (in which case we rotate it)
  const Eigen::Vector3d rotated_direction = global_reference_frame ? direction : start_pose.linear() * direction;

  // The target pose is built by applying a translation to the start pose for the desired direction and distance
  Eigen::Affine3d target_pose = start_pose;
  target_pose.translation() += rotated_direction * distance;

  // call computeCartesianPath for the computed target pose in the global reference frame
  return (distance *
          computeCartesianPath(group, traj, link, target_pose, true, max_step, jump_threshold, validCallback, options));
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const Eigen::Affine3d& target,
                                        bool global_reference_frame, const MaxEEFStep& max_step,
                                        const JumpThreshold& jump_threshold,
                                        const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  const std::vector<const JointModel*>& cjnt = group->getContinuousJointModels();
  // make sure that continuous joints wrap
  for (std::size_t i = 0; i < cjnt.size(); ++i)
    enforceBounds(cjnt[i]);

  // this is the Cartesian pose we start from, and we move in the direction indicated
  Eigen::Affine3d start_pose = getGlobalLinkTransform(link);

  // the target can be in the local reference frame (in which case we rotate it)
  Eigen::Affine3d rotated_target = global_reference_frame ? target : start_pose * target;

  Eigen::Quaterniond start_quaternion(start_pose.linear());
  Eigen::Quaterniond target_quaternion(rotated_target.linear());

  if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
  {
    ROS_ERROR_NAMED(LOGNAME,
                    "Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                    "MaxEEFStep.translation components must be non-negative and at least one component must be "
                    "greater than zero");
    return 0.0;
  }

  double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this trajectory
  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
    translation_steps = floor(translation_distance / max_step.translation);

  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
    rotation_steps = floor(rotation_distance / max_step.rotation);

  // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  if (jump_threshold.factor > 0 && steps < MIN_STEPS_FOR_JUMP_THRESH)
    steps = MIN_STEPS_FOR_JUMP_THRESH;

  traj.clear();
  traj.push_back(RobotStatePtr(new RobotState(*this)));

  double last_valid_percentage = 0.0;
  for (std::size_t i = 1; i <= steps; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

    // Explicitly use a single IK attempt only: We want a smooth trajectory.
    // Random seeding (of additional attempts) would probably create IK jumps.
    if (setFromIK(group, pose, link->getName(), 1, 0.0, validCallback, options))
      traj.push_back(RobotStatePtr(new RobotState(*this)));
    else
      break;

    last_valid_percentage = percentage;
  }

  last_valid_percentage *= testJointSpaceJump(group, traj, jump_threshold);

  return last_valid_percentage;
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const EigenSTL::vector_Affine3d& waypoints,
                                        bool global_reference_frame, const MaxEEFStep& max_step,
                                        const JumpThreshold& jump_threshold,
                                        const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  double percentage_solved = 0.0;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    // Don't test joint space jumps for every waypoint, test them later on the whole trajectory.
    static const JumpThreshold no_joint_space_jump_test;
    std::vector<RobotStatePtr> waypoint_traj;
    double wp_percentage_solved = computeCartesianPath(group, waypoint_traj, link, waypoints[i], global_reference_frame,
                                                       max_step, no_joint_space_jump_test, validCallback, options);
    if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = (double)(i + 1) / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1);
      traj.insert(traj.end(), start, waypoint_traj.end());
    }
    else
    {
      percentage_solved += wp_percentage_solved / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1);
      traj.insert(traj.end(), start, waypoint_traj.end());
      break;
    }
  }

  percentage_solved *= testJointSpaceJump(group, traj, jump_threshold);

  return percentage_solved;
}

double RobotState::testJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                      const JumpThreshold& jump_threshold)
{
  double percentage_solved = 1.0;
  if (traj.size() <= 1)
    return percentage_solved;

  if (jump_threshold.factor > 0.0)
    percentage_solved *= testRelativeJointSpaceJump(group, traj, jump_threshold.factor);

  if (jump_threshold.revolute > 0.0 || jump_threshold.prismatic > 0.0)
    percentage_solved *= testAbsoluteJointSpaceJump(group, traj, jump_threshold.revolute, jump_threshold.prismatic);

  return percentage_solved;
}

double RobotState::testRelativeJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                              double jump_threshold_factor)
{
  if (traj.size() < MIN_STEPS_FOR_JUMP_THRESH)
  {
    ROS_WARN_NAMED(LOGNAME, "The computed trajectory is too short to detect jumps in joint-space "
                            "Need at least %zu steps, only got %zu. Try a lower max_step.",
                   MIN_STEPS_FOR_JUMP_THRESH, traj.size());
  }

  std::vector<double> dist_vector;
  dist_vector.reserve(traj.size() - 1);
  double total_dist = 0.0;
  for (std::size_t i = 1; i < traj.size(); ++i)
  {
    double dist_prev_point = traj[i]->distance(*traj[i - 1], group);
    dist_vector.push_back(dist_prev_point);
    total_dist += dist_prev_point;
  }

  double percentage = 1.0;
  // compute the average distance between the states we looked at
  double thres = jump_threshold_factor * (total_dist / (double)dist_vector.size());
  for (std::size_t i = 0; i < dist_vector.size(); ++i)
    if (dist_vector[i] > thres)
    {
      ROS_DEBUG_NAMED(LOGNAME, "Truncating Cartesian path due to detected jump in joint-space distance");
      percentage = (double)(i + 1) / (double)(traj.size());
      traj.resize(i + 1);
      break;
    }

  return percentage;
}

double RobotState::testAbsoluteJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                              double revolute_threshold, double prismatic_threshold)
{
  struct LimitData
  {
    double limit;
    bool check;
  };
  LimitData data[2] = { { revolute_threshold, revolute_threshold > 0.0 },
                        { prismatic_threshold, prismatic_threshold > 0.0 } };
  bool still_valid = true;
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (std::size_t traj_ix = 0, ix_end = traj.size() - 1; traj_ix != ix_end; ++traj_ix)
  {
    for (auto& joint : joints)
    {
      unsigned int type_index;
      switch (joint->getType())
      {
        case JointModel::REVOLUTE:
          type_index = 0;
          break;
        case JointModel::PRISMATIC:
          type_index = 1;
          break;
        default:
          ROS_WARN_NAMED(LOGNAME, "Joint %s has not supported type %s. \n"
                                  "testAbsoluteJointSpaceJump only supports prismatic and revolute joints.",
                         joint->getName().c_str(), joint->getTypeName().c_str());
          continue;
      }
      if (!data[type_index].check)
        continue;

      double distance = traj[traj_ix]->distance(*traj[traj_ix + 1], joint);
      if (distance > data[type_index].limit)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Truncating Cartesian path due to detected jump of %.4f > %.4f in joint %s", distance,
                        data[type_index].limit, joint->getName().c_str());
        still_valid = false;
        break;
      }
    }

    if (!still_valid)
    {
      double percent_valid = (double)(traj_ix + 1) / (double)(traj.size());
      traj.resize(traj_ix + 1);
      return percent_valid;
    }
  }
  return 1.0;
}

void RobotState::computeAABB(std::vector<double>& aabb) const
{
  BOOST_VERIFY(checkLinkTransforms());

  core::AABB bounding_box;
  std::vector<const LinkModel*> links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    Eigen::Affine3d transform = getGlobalLinkTransform(links[i]);  // intentional copy, we will translate
    const Eigen::Vector3d& extents = links[i]->getShapeExtentsAtOrigin();
    transform.translate(links[i]->getCenteredBoundingBoxOffset());
    bounding_box.extendWithTransformedBox(transform, extents);
  }
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin();
       it != attached_body_map_.end(); ++it)
  {
    const EigenSTL::vector_Affine3d& transforms = it->second->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr>& shapes = it->second->getShapes();
    for (std::size_t i = 0; i < transforms.size(); ++i)
    {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes[i].get());
      bounding_box.extendWithTransformedBox(transforms[i], extents);
    }
  }

  aabb.clear();
  aabb.resize(6, 0.0);
  if (!bounding_box.isEmpty())
  {
    // The following is a shorthand for something like:
    // aabb[0, 2, 4] = bounding_box.min(); aabb[1, 3, 5] = bounding_box.max();
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned, Eigen::InnerStride<2> >(aabb.data(), 3) = bounding_box.min();
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned, Eigen::InnerStride<2> >(aabb.data() + 1, 3) = bounding_box.max();
  }
}

void RobotState::printStatePositions(std::ostream& out) const
{
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (std::size_t i = 0; i < nm.size(); ++i)
    out << nm[i] << "=" << position_[i] << std::endl;
}

void RobotState::printDirtyInfo(std::ostream& out) const
{
  out << "  * Dirty Joint Transforms: " << std::endl;
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
    if (jm[i]->getVariableCount() > 0 && dirtyJointTransform(jm[i]))
      out << "    " << jm[i]->getName() << std::endl;
  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL")
      << std::endl;
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;
}

void RobotState::printStateInfo(std::ostream& out) const
{
  out << "Robot State @" << this << std::endl;

  std::size_t n = robot_model_->getVariableCount();
  if (position_)
  {
    out << "  * Position: ";
    for (std::size_t i = 0; i < n; ++i)
      out << position_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Position: NULL" << std::endl;

  if (velocity_)
  {
    out << "  * Velocity: ";
    for (std::size_t i = 0; i < n; ++i)
      out << velocity_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Velocity: NULL" << std::endl;

  if (acceleration_)
  {
    out << "  * Acceleration: ";
    for (std::size_t i = 0; i < n; ++i)
      out << acceleration_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Acceleration: NULL" << std::endl;

  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL")
      << std::endl;
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;

  printTransforms(out);
}

void RobotState::printTransform(const Eigen::Affine3d& transform, std::ostream& out) const
{
  Eigen::Quaterniond q(transform.linear());
  out << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
      << transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
      << "]" << std::endl;
}

void RobotState::printTransforms(std::ostream& out) const
{
  if (!variable_joint_transforms_)
  {
    out << "No transforms computed" << std::endl;
    return;
  }

  out << "Joint transforms:" << std::endl;
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (std::size_t i = 0; i < jm.size(); ++i)
  {
    out << "  " << jm[i]->getName();
    const int idx = jm[i]->getJointIndex();
    if (dirty_joint_transforms_[idx])
      out << " [dirty]";
    out << ": ";
    printTransform(variable_joint_transforms_[idx], out);
  }

  out << "Link poses:" << std::endl;
  const std::vector<const LinkModel*>& lm = robot_model_->getLinkModels();
  for (std::size_t i = 0; i < lm.size(); ++i)
  {
    out << "  " << lm[i]->getName() << ": ";
    printTransform(global_link_transforms_[lm[i]->getLinkIndex()], out);
  }
}

std::string RobotState::getStateTreeString(const std::string& prefix) const
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
  for (int y = 0; y < 4; ++y)
  {
    ss << pfx;
    for (int x = 0; x < 4; ++x)
    {
      ss << std::setw(8) << pose(y, x) << " ";
    }
    ss << std::endl;
  }
}
}

void RobotState::getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0,
                                         bool last) const
{
  std::string pfx = pfx0 + "+--";

  ss << pfx << "Joint: " << jm->getName() << std::endl;

  pfx = pfx0 + (last ? "   " : "|  ");

  for (std::size_t i = 0; i < jm->getVariableCount(); ++i)
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

  for (std::vector<const JointModel*>::const_iterator it = lm->getChildJointModels().begin();
       it != lm->getChildJointModels().end(); ++it)
    getStateTreeJointString(ss, *it, pfx, it + 1 == lm->getChildJointModels().end());
}

std::ostream& operator<<(std::ostream& out, const RobotState& s)
{
  s.printStateInfo(out);
  return out;
}

}  // end of namespace core
}  // end of namespace moveit
