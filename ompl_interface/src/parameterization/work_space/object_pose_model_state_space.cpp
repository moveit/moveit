/*********************************************************************
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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface/parameterization/work_space/object_pose_model_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ros/console.h>
#include <ompl/tools/debug/Profiler.h>

ompl_interface::ObjectPoseModelStateSpace::ObjectPoseModelStateSpace(const ModelBasedStateSpaceSpecification &spec) : 
  ModelBasedStateSpace(spec)
{
  if (spec.kinematics_allocator_)
    constructSpace(spec.joint_model_group_, spec.kinematics_allocator_);
  else
    if (!spec.kinematics_subgroup_allocators_.empty())
      constructSpace(spec.joint_model_group_, spec.kinematics_subgroup_allocators_);
    else
      ROS_FATAL("No kinematics solvers specified. Unable to construct a ObjectPoseModelStateSpace");
}

ompl::base::State* ompl_interface::ObjectPoseModelStateSpace::allocState(void) const
{
  StateType *s = new StateType();
  allocStateComponents(s);
  return s;
}

void ompl_interface::ObjectPoseModelStateSpace::freeState(ob::State *state) const
{
  ModelBasedStateSpace::freeState(state);
}

void ompl_interface::ObjectPoseModelStateSpace::copyState(ob::State *destination, const ob::State *source) const
{
  // copy the state data
  ModelBasedStateSpace::copyState(destination, source);
    
  // compute additional stuff if needed
  computeStateK(destination);
}

void ompl_interface::ObjectPoseModelStateSpace::interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{    
  // we want to interpolate in Cartesian space; we do not have a guarantee that from and to
  // have their poses computed, but this is very unlikely to happen (depends how the planner gets its input states)
  // we cover the worst case scenario by copying the state to a temp location, since the copy operation also computes 
  // all the kinematics stuff
  
  if (!from->as<StateType>()->poseComputed())
  {
    ob::State *temp = allocState();
    copyState(temp, from);
    if (temp->as<StateType>()->poseComputed())
      interpolate(temp, to, t, state);
    else
    {  
      ModelBasedStateSpace::interpolate(from, to, t, state);
      state->as<StateType>()->markInvalid();
    }
    freeState(temp);
    return;
  }
  
  if (!to->as<StateType>()->poseComputed())
  {
    ob::State *temp = allocState();
    copyState(temp, to);   
    if (temp->as<StateType>()->poseComputed())
      interpolate(from, temp, t, state);
    else
    {   
      ModelBasedStateSpace::interpolate(from, to, t, state);
      state->as<StateType>()->markInvalid();
    }
    freeState(temp);
    return;
  }
  
  ModelBasedStateSpace::interpolate(from, to, t, state);

  // after interpolation we cannot be sure about the joint values (we use them as seed only)
  // so we recompute IK
  state->as<StateType>()->setJointsComputed(false);
  computeStateIK(state);
}

void ompl_interface::ObjectPoseModelStateSpace::beforeStateSample(ob::State *sampled) const
{
  ModelBasedStateSpace::beforeStateSample(sampled);
  sampled->as<StateType>()->setPoseComputed(false);
}

void ompl_interface::ObjectPoseModelStateSpace::afterStateSample(ob::State *sampled) const
{  
  computeStateFK(sampled);
  ModelBasedStateSpace::afterStateSample(sampled);
}

namespace ompl_interface
{
static inline Eigen::Affine3d se3ToEigen(const ob::State *state)
{
  const ob::SE3StateSpace::StateType *se3_state = state->as<ob::SE3StateSpace::StateType>();
  const ob::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  return Eigen::Affine3d(Eigen::Translation3d(se3_state->getX(), se3_state->getY(), se3_state->getZ()) * 
                         Eigen::Quaterniond(so3_state.w, so3_state.x, so3_state.y, so3_state.z).toRotationMatrix());
}

static inline void eigenToSE3(const Eigen::Affine3d &t, ob::State *state)
{
  ob::SE3StateSpace::StateType *se3_state = state->as<ob::SE3StateSpace::StateType>();
  ob::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  se3_state->setXYZ(t.translation().x(), t.translation().y(), t.translation().z());
  Eigen::Quaterniond q(t.rotation());
  so3_state.x = q.x(); so3_state.y = q.y(); so3_state.z = q.z(); so3_state.w = q.w();
}

}

void ompl_interface::ObjectPoseModelStateSpace::copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const
{
  // if joint values are not computed, we compute them
  if (!state->as<StateType>()->jointsComputed())
  {
    ob::State *temp = allocState();
    copyState(temp, state);
    bool cont = false;
    if (computeStateIK(temp))
      copyToKinematicState(js, temp);
    else
      cont = true;
    freeState(temp);
    if (!cont)
      return;
  }
  
  if (poses_.size() == 1)
    poses_[0].joint_model_.copyToKinematicState(js, state->as<StateType>()->as<ob::CompoundState>(0)->components[0]);
  else
  {
    std::map<const pm::KinematicModel::JointModel*, pm::KinematicState::JointState*> jmap;
    for (std::size_t i = 0 ; i < js.size() ; ++i)
      jmap[js[i]->getJointModel()] = js[i];
    for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    {
      const std::vector<const pm::KinematicModel::JointModel*> &subgroup_joint_models = poses_[i].subgroup_->getJointModels();
      std::vector<pm::KinematicState::JointState*> subgroup_joint_states(subgroup_joint_models.size());
      for (std::size_t j = 0 ; j < subgroup_joint_models.size() ; ++j)
        subgroup_joint_states[j] = jmap[subgroup_joint_models[j]];
      poses_[i].joint_model_.copyToKinematicState(subgroup_joint_states, state->as<StateType>()->as<ob::CompoundState>(i)->components[0]);
    }
  }
}

void ompl_interface::ObjectPoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const
{
  state->as<StateType>()->clearKnownInformation();
  state->as<StateType>()->setJointsComputed(true);
  state->as<StateType>()->setPoseComputed(false);
  if (poses_.size() == 1)
    poses_[0].joint_model_.copyToOMPLState(state->as<StateType>()->as<ob::CompoundState>(0)->components[0], js);
  else
    for (std::size_t i = 0 ; i < js.size() ; ++i)
    {
      double *ompl_val = getValueAddressAtName(state, js[i]->getName());
      const std::vector<double> &vals = js[i]->getVariableValues();
      for (std::size_t j = 0 ; j < vals.size() ; ++j)
        ompl_val[j] = vals[j];
    }
  computeStateFK(state);
}

void ompl_interface::ObjectPoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<double> &values) const
{ 
  state->as<StateType>()->clearKnownInformation(); 
  state->as<StateType>()->setJointsComputed(true);
  state->as<StateType>()->setPoseComputed(false);  
  if (poses_.size() == 1)
    poses_[0].joint_model_.copyToOMPLState(state->as<StateType>()->as<ob::CompoundState>(0)->components[0], values);
  else
  {
    const std::vector<const pm::KinematicModel::JointModel*> &jm = spec_.joint_model_group_->getJointModels();
    unsigned int vindex = 0;
    for (std::size_t i = 0 ; i < jm.size() ; ++i)
    { 
      double *ompl_val = getValueAddressAtName(state, jm[i]->getName());
      const unsigned int k = jm[i]->getVariableCount();
      for (std::size_t j = 0 ; j < k ; ++j)
        ompl_val[j] = values[vindex++];
    }
  }
  computeStateFK(state);
}

ompl_interface::ObjectPoseModelStateSpace::PoseComponent::PoseComponent(const pm::KinematicModel::JointModelGroup *subgroup, 
                                                                        const planning_scene::KinematicsAllocatorFn &kinematics_allocator) :
  subgroup_(subgroup), kinematics_solver_(kinematics_allocator(subgroup)),
  joint_model_(subgroup->getJointModels())
{
  state_space_ = joint_model_.getStateSpace();
  fk_link_.resize(1, kinematics_solver_->getTipFrame());      
  joint_names_ = kinematics_solver_->getJointNames();
  joint_val_count_.resize(joint_names_.size());
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
    joint_val_count_[i] = subgroup_->getJointModel(joint_names_[i])->getVariableCount();
  variable_count_ = subgroup_->getVariableCount();
}

bool ompl_interface::ObjectPoseModelStateSpace::PoseComponent::computeStateFK(const ob::State *state, ob::SE3StateSpace::StateType *se3_state) const
{
  // read the values from the joint state by name, in the order expected by the kinematics solver
  std::vector<double> values(variable_count_);
  unsigned int vindex = 0;
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
  {
    const double *v = state_space_->getValueAddressAtName(state, joint_names_[i]);
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      values[vindex++] = v[j];
  }
  
  // compute forward kinematics for the link of interest
  std::vector<geometry_msgs::Pose> poses;
  if (!kinematics_solver_->getPositionFK(fk_link_, values, poses))
    return false;
  
  // copy the resulting data to the desired location in the state
  se3_state->setXYZ(poses[0].position.x, poses[0].position.y, poses[0].position.z);
  ob::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  so3_state.x = poses[0].orientation.x;
  so3_state.y = poses[0].orientation.y;
  so3_state.z = poses[0].orientation.z;
  so3_state.w = poses[0].orientation.w;
  
  return true;
}

bool ompl_interface::ObjectPoseModelStateSpace::PoseComponent::computeStateIK(ob::State *state, const ob::SE3StateSpace::StateType *se3_state) const
{
  // read the values from the joint state by name, in the order expected by the kinematics solver; use these as the seed
  std::vector<double> seed_values(variable_count_);
  std::vector<double*> jaddr(joint_names_.size());
  unsigned int vindex = 0;
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
  {
    double *v = jaddr[i] = state_space_->getValueAddressAtName(state, joint_names_[i]);
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      seed_values[vindex++] = v[j];
  }
  
  // construct the pose
  geometry_msgs::Pose pose;
  pose.position.x = se3_state->getX();
  pose.position.y = se3_state->getY();
  pose.position.z = se3_state->getZ();
  const ob::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  pose.orientation.x = so3_state.x;
  pose.orientation.y = so3_state.y;
  pose.orientation.z = so3_state.z;
  pose.orientation.w = so3_state.w;
  
  // run IK
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes dummy;
  if (!kinematics_solver_->getPositionIK(pose, seed_values, solution, dummy))
    return false;
  
  // copy solution to the joint state 
  vindex = 0;
  for (std::size_t i = 0 ; i < jaddr.size() ; ++i)
    for (unsigned int j = 0 ; j < joint_val_count_[i] ; ++j)
      jaddr[i][j] = solution[vindex++];
  
  return true;      
}

void ompl_interface::ObjectPoseModelStateSpace::constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                                                               const planning_scene::KinematicsAllocatorFn &ik_allocator)
{
  poses_.push_back(PoseComponent(group, ik_allocator));
  constructSpaceFromPoses();
}

void ompl_interface::ObjectPoseModelStateSpace::constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                                                               const planning_scene::KinematicsAllocatorMapFn &ik_allocator)
{
  for (std::map<const pm::KinematicModel::JointModelGroup*, planning_scene::KinematicsAllocatorFn>::const_iterator it = ik_allocator.begin() ; it != ik_allocator.end() ; ++it)
    poses_.push_back(PoseComponent(it->first, it->second));
  constructSpaceFromPoses();
}

void ompl_interface::ObjectPoseModelStateSpace::constructSpaceFromPoses(void)
{
  std::sort(poses_.begin(), poses_.end());
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    addSubspace(poses_[i].state_space_, 0.0);
  addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.0);
  setName(getJointModelGroupName() + "_ObjectPoseModel");
  lock();
}

bool ompl_interface::ObjectPoseModelStateSpace::computeStateFK(ob::State *state) const
{
  if (state->as<StateType>()->poseComputed())
    return true;
  if (!poses_[0].computeStateFK(state->as<StateType>()->components[0], state->as<StateType>()->components[componentCount_ - 1]->as<ob::SE3StateSpace::StateType>()))
  {
    state->as<StateType>()->markInvalid();
    return false;
  }
  state->as<StateType>()->setPoseComputed(true);
  return true;
}

bool ompl_interface::ObjectPoseModelStateSpace::computeStateIK(ob::State *state) const
{  
  if (state->as<StateType>()->jointsComputed())
    return true;
  if (!poses_[0].computeStateIK(state->as<StateType>()->components[0], state->as<StateType>()->components[componentCount_ - 1]->as<ob::SE3StateSpace::StateType>()))
  {
    state->as<StateType>()->markInvalid();
    return false;
  }
  if (poses_.size() > 1)
  {
    ob::SE3StateSpace::StateType *temp = components_.back()->allocState()->as<ob::SE3StateSpace::StateType>();
    Eigen::Affine3d stf = se3ToEigen(state->as<StateType>()->components[componentCount_ - 1]);
    for (std::size_t i = 1 ; i < poses_.size() ; ++i)
    {
      eigenToSE3(stf * offset_transforms_[i-1], temp);
      if (!poses_[i].computeStateIK(state->as<StateType>()->components[i], temp))
      {
        state->as<StateType>()->markInvalid();
        return false;
      }
    }
    components_.back()->freeState(temp);
  }
  state->as<StateType>()->setJointsComputed(true);
  return true;
}

bool ompl_interface::ObjectPoseModelStateSpace::computeStateK(ob::State *state) const
{
  if (state->as<StateType>()->jointsComputed() && !state->as<StateType>()->poseComputed())
    return computeStateFK(state);
  if (!state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return computeStateIK(state);
  if (state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return true;
  state->as<StateType>()->markInvalid();
  return false;
}
