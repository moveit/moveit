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
#include <ros/console.h>

const std::string ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE = "PoseModel";

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

void ompl_interface::PoseModelStateSpace::interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
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

ompl_interface::PoseModelStateSpace::PoseComponent::PoseComponent(const pm::KinematicModel::JointModelGroup *subgroup, 
                                                                  const planning_scene::KinematicsAllocatorFn &kinematics_allocator) :
  subgroup_(subgroup), kinematics_solver_(kinematics_allocator(subgroup)),
  joint_model_(subgroup->getJointModels())
{
  ob::CompoundStateSpace *cspace = new ob::CompoundStateSpace();
  state_space_.reset(cspace);
  se3_component_ = new ob::SE3StateSpace();    
  se3_component_->setName(subgroup_->getName() + "_Workspace");
  cspace->addSubspace(joint_model_.getStateSpace(), 0.0);
  cspace->addSubspace(ob::StateSpacePtr(se3_component_), 1.0);
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
                                                         const planning_scene::KinematicsAllocatorFn &ik_allocator)
{
  poses_.push_back(PoseComponent(group, ik_allocator));
  constructSpaceFromPoses();
}

void ompl_interface::PoseModelStateSpace::constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                                                         const planning_scene::KinematicsAllocatorMapFn &ik_allocator)
{
  for (std::map<const pm::KinematicModel::JointModelGroup*, planning_scene::KinematicsAllocatorFn>::const_iterator it = ik_allocator.begin() ; it != ik_allocator.end() ; ++it)
    poses_.push_back(PoseComponent(it->first, it->second));
  constructSpaceFromPoses();
}

void ompl_interface::PoseModelStateSpace::constructSpaceFromPoses(void)
{
  std::sort(poses_.begin(), poses_.end());
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    addSubspace(poses_[i].state_space_, 1.0);  
  setName(getJointModelGroupName() + "_PoseModel");
  lock();
}

bool ompl_interface::PoseModelStateSpace::computeStateFK(ob::State *state) const
{
  if (state->as<StateType>()->poseComputed())
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateFK(state->as<StateType>()->components[i]))
    {
      state->as<StateType>()->markInvalid();
      return false;
    }
  state->as<StateType>()->setPoseComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateIK(ob::State *state) const
{  
  if (state->as<StateType>()->jointsComputed())
    return true;
  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
    if (!poses_[i].computeStateIK(state->as<StateType>()->components[i]))
    {
      state->as<StateType>()->markInvalid();
      return false;
    }
  state->as<StateType>()->setJointsComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateK(ob::State *state) const
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
