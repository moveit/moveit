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

#include "ompl_interface/parameterization/work_space/pose_model_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ros/console.h>

ompl_interface::PoseModelStateSpace::PoseModelStateSpace(const ModelBasedStateSpaceSpecification &spec) : 
  ModelBasedStateSpace(spec)
{
  if (spec.kinematics_allocator_)
    constructSpace(spec.joint_model_group_, spec.kinematics_allocator_);
  else
    if (!spec.kinematics_subgroup_allocators_.empty())
      constructSpace(spec.joint_model_group_, spec.kinematics_subgroup_allocators_);
    else
      ROS_FATAL("No kinematics solvers specified. Unable to construct a PoseModelStateSpace");
}

ompl::base::State* ompl_interface::PoseModelStateSpace::allocState(void) const
{
  StateType *s = new StateType();
  allocStateComponents(s);
  return s;
}

void ompl_interface::PoseModelStateSpace::freeState(ob::State *state) const
{
  ModelBasedStateSpace::freeState(state);
}

void ompl_interface::PoseModelStateSpace::copyState(ob::State *destination, const ob::State *source) const
{
  // copy the state data
  ModelBasedStateSpace::copyState(destination, source);
  
  // copy additional data
  destination->as<StateType>()->pose_computed = source->as<StateType>()->pose_computed;
  destination->as<StateType>()->joints_computed = source->as<StateType>()->joints_computed;
  
  // compute additional stuff if needed
  computeStateK(destination);
}

void ompl_interface::PoseModelStateSpace::interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{    
  // we want to interpolate in Cartesian space; we do not have a guarantee that from and to
  // have their poses computed, but this is very unlikely to happen (depends how the planner gets its input states)
  // we cover the worst case scenario by copying the state to a temp location, since the copy operation also computes 
  // all the kinematics stuff
  
  if (!from->as<StateType>()->pose_computed)
  {
    ob::State *temp = allocState();
    copyState(temp, from);
    interpolate(temp, to, t, state);
    freeState(temp);
    return;
  }
  
  if (!to->as<StateType>()->pose_computed)
  {
    ob::State *temp = allocState();
    copyState(temp, to);
    interpolate(from, temp, t, state);
    freeState(temp);
    return;
  }
  
  ModelBasedStateSpace::interpolate(from, to, t, state);
  
  // after interpolation we cannot be sure about the joint values (we use them as seed only)
  // so we recompute IK
  state->as<StateType>()->joints_computed = false;
  computeStateIK(state);
}
  
void ompl_interface::PoseModelStateSpace::copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const
{
  // if joint values are not computed, we compute them
  if (!state->as<StateType>()->joints_computed)
  {
    ob::State *temp = allocState();
    copyState(temp, state);
    if (computeStateIK(temp))
      copyToKinematicState(js, temp);
    else
      ROS_ERROR("Cannot copy invalid OMPL state to KinematicState");
    freeState(temp);
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

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const
{
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
  state->as<StateType>()->joints_computed = true;
  state->as<StateType>()->pose_computed = false;
  computeStateIK(state);
}

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<double> &values) const
{
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
  state->as<StateType>()->joints_computed = true;
  state->as<StateType>()->pose_computed = false;
  computeStateIK(state);
}

ompl_interface::PoseModelStateSpace::PoseComponent::PoseComponent(const pm::KinematicModel::JointModelGroup *subgroup, 
                                                                  const kc::IKAllocator &kinematics_allocator) :
  subgroup_(subgroup), kinematics_solver_(kinematics_allocator(subgroup)),
  joint_model_(subgroup->getJointModels())
{
  ob::CompoundStateSpace *cspace = new ob::CompoundStateSpace();
  state_space_.reset(cspace);
  se3_component_ = new ob::SE3StateSpace();    
  se3_component_->setName(subgroup_->getName() + "_Workspace");
  cspace->addSubSpace(joint_model_.getStateSpace(), 0.0);
  cspace->addSubSpace(ob::StateSpacePtr(se3_component_), 1.0);
  cspace->lock();
  fk_link_.resize(1, kinematics_solver_->getTipFrame());      
  joint_names_ = kinematics_solver_->getJointNames();
  joint_val_count_.resize(joint_names_.size());
  for (std::size_t i = 0 ; i < joint_names_.size() ; ++i)
    joint_val_count_[i] = subgroup_->getJointModel(joint_names_[i])->getVariableCount();
  variable_count_ = subgroup_->getVariableCount();
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateFK(ob::State *state) const
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
  ob::SE3StateSpace::StateType *se3_state = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(1);
  se3_state->setXYZ(poses[0].position.x, poses[0].position.y, poses[0].position.z);
  ob::SO3StateSpace::StateType &so3_state = se3_state->rotation();
  so3_state.x = poses[0].orientation.x;
  so3_state.y = poses[0].orientation.y;
  so3_state.z = poses[0].orientation.z;
  so3_state.w = poses[0].orientation.w;
  
  return true;
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateIK(ob::State *state) const
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
  const ob::SE3StateSpace::StateType *se3_state = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(1);
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

void ompl_interface::PoseModelStateSpace::constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                                                         const kc::IKAllocator &ik_allocator)
{
  poses_.push_back(PoseComponent(group, ik_allocator));
  constructSpaceFromPoses();
}

void ompl_interface::PoseModelStateSpace::constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                                                         const kc::IKSubgroupAllocator &ik_allocator)
{
  for (std::map<const pm::KinematicModel::JointModelGroup*, kc::IKAllocator>::const_iterator it = ik_allocator.begin() ; it != ik_allocator.end() ; ++it)
    poses_.push_back(PoseComponent(it->first, it->second));
  constructSpaceFromPoses();
}

void ompl_interface::PoseModelStateSpace::constructSpaceFromPoses(void)
{
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    addSubSpace(poses_[i].state_space_, 1.0);  
  setName(getJointModelGroupName() + "_PoseModel");
  lock();
}

bool ompl_interface::PoseModelStateSpace::computeStateFK(ob::State *state) const
{
  if (state->as<StateType>()->pose_computed)
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateFK(state->as<StateType>()->components[i]))
      return false;
  state->as<StateType>()->pose_computed = true;
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateIK(ob::State *state) const
{  
  if (state->as<StateType>()->joints_computed)
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateIK(state->as<StateType>()->components[i]))
      return false;
  state->as<StateType>()->joints_computed = true;
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateK(ob::State *state) const
{
  if (state->as<StateType>()->joints_computed && !state->as<StateType>()->pose_computed)
    return computeStateFK(state);
  if (!state->as<StateType>()->joints_computed && state->as<StateType>()->pose_computed)
    return computeStateIK(state);
  return state->as<StateType>()->joints_computed && state->as<StateType>()->pose_computed;
}
