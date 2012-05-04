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

#include "ompl_interface/parameterization/joint_space/joint_model_state_space_helper.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/console.h>

void ompl_interface::JointModelStateSpaceHelper::constructSpace(const std::vector<const pm::KinematicModel::JointModel*> &joints)
{
  ob::StateSpacePtr space;
  
  joint_mapping_.clear();
  variable_mapping_.clear();
  
  ob::RealVectorStateSpace *rv = NULL;
  std::vector<std::size_t> rv_joints;
  std::vector<std::size_t> rv_var;
  std::size_t nvar = 0;
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    ob::StateSpacePtr to_add;
    
    const pm::KinematicModel::RevoluteJointModel* revolute_joint =
      dynamic_cast<const pm::KinematicModel::RevoluteJointModel*>(joints[i]);
    if (revolute_joint && revolute_joint->isContinuous())
    {
      ob::SO2StateSpace *space = new ob::SO2StateSpace();
      space->setName(revolute_joint->getName());
      joint_mapping_.push_back(i);
      variable_mapping_.push_back(nvar++);
      to_add.reset(space);
    }
    else
    {
      const pm::KinematicModel::PlanarJointModel* planar_joint =
	dynamic_cast<const pm::KinematicModel::PlanarJointModel*>(joints[i]);
      if (planar_joint)
      {
	ob::SE2StateSpace *space = new ob::SE2StateSpace();
	space->setName(planar_joint->getName());
	joint_mapping_.push_back(i);
	variable_mapping_.push_back(nvar);
	nvar += 3;
	to_add.reset(space);
      }
      else
      {
	const pm::KinematicModel::FloatingJointModel* floating_joint =
	  dynamic_cast<const pm::KinematicModel::FloatingJointModel*>(joints[i]);
	if (floating_joint)
	{
	  ob::SE3StateSpace *space = new ob::SE3StateSpace();
	  space->setName(floating_joint->getName());
	  joint_mapping_.push_back(i);
	  variable_mapping_.push_back(nvar);
	  nvar += 7;
	  to_add.reset(space);
	}
	else
	{
	  const pm::KinematicModel::PrismaticJointModel* prismatic_joint =
	    dynamic_cast<const pm::KinematicModel::PrismaticJointModel*>(joints[i]);
	  if (revolute_joint || prismatic_joint)
	  {
	    if (!rv)
	      rv = new ob::RealVectorStateSpace();
            
            std::pair<double,double> bounds;
            joints[i]->getVariableBounds(joints[i]->getName(), bounds);
            rv->addDimension(joints[i]->getName(), bounds.first, bounds.second);
	    rv_joints.push_back(i);
	    rv_var.push_back(nvar++);
	  }
	}
	// otherwise, unprocessed joint (fixed)
      }
      
    }
    // if any space was created, remember it
    if (to_add)
      space = space + to_add;
  }
  
  // add real vector components on last position
  if (rv)
  {
    std::string rv_name;
    // see if this space was previously created
    for (std::size_t i = 0 ; i < rv_joints.size() ; ++i)
    {
      if (!rv_name.empty())
	rv_name += ",";
      rv_name += joints[rv_joints[i]]->getName();
      joint_mapping_.push_back(rv_joints[i]);
      variable_mapping_.push_back(rv_var[i]);
    }
    rv->setName("J(" + rv_name + ")");
    space = space + ob::StateSpacePtr(rv);
  }
  
  if (!space)
  {
    ROS_WARN("Empty OMPL state space");
    return;
  }
  
  // we make the assumption later on this is a compound space, so ensure this is always the case:
  if (!space->isCompound() || (space->isCompound() && space->as<ob::CompoundStateSpace>()->isLocked()))
  {
    ob::CompoundStateSpace *csm = new ob::CompoundStateSpace();
    csm->addSubspace(space, 1.0);
    space.reset(csm);
  }
  
  ob::CompoundStateSpace *state_space = new ob::CompoundStateSpace();
  
  // heuristically set some weights for the subspaces, based on dimension
  unsigned int ns = space->as<ob::CompoundStateSpace>()->getSubspaceCount();
  std::string name;
  for (unsigned int i = 0 ; i < ns ; ++i)
  {
    const ob::StateSpacePtr &c = space->as<ob::CompoundStateSpace>()->getSubspace(i);
    state_space->addSubspace(c, (double)c->getDimension());
    if (i == 0)
      name = c->getName();
    else
      name = name + "," + c->getName();
  }
  state_space->setName("J(" + name + ")");
  state_space->lock();
  components_ = state_space->getSubspaces();
  
  state_space_.reset(state_space);
}

void ompl_interface::JointModelStateSpaceHelper::copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const
{
  const ob::CompoundState *cstate = state->as<ob::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < components_.size() ; ++i)
    if (components_[i]->getType() == ob::STATE_SPACE_SO2)
      js[joint_mapping_[j++]]->setVariableValues(&cstate->as<ob::SO2StateSpace::StateType>(i)->value);
    else
      if (components_[i]->getType() == ob::STATE_SPACE_SE2)
      {
	std::vector<double> values(3);
	values[0] = cstate->as<ob::SE2StateSpace::StateType>(i)->getX();
	values[1] = cstate->as<ob::SE2StateSpace::StateType>(i)->getY();
	values[2] = cstate->as<ob::SE2StateSpace::StateType>(i)->getYaw();
	js[joint_mapping_[j++]]->setVariableValues(values);
      }
      else
	if (components_[i]->getType() == ob::STATE_SPACE_SE3)
	{
	  std::vector<double> values(7);
	  values[0] = cstate->as<ob::SE3StateSpace::StateType>(i)->getX();
	  values[1] = cstate->as<ob::SE3StateSpace::StateType>(i)->getY();
	  values[2] = cstate->as<ob::SE3StateSpace::StateType>(i)->getZ();
	  values[3] = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation().x;
	  values[4] = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation().y;
	  values[5] = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation().z;
	  values[6] = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation().w;
	  js[joint_mapping_[j++]]->setVariableValues(values);
	}
	else
	  if (components_[i]->getType() == ob::STATE_SPACE_REAL_VECTOR)
	  {
	    const double *vals = cstate->as<ob::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      js[joint_mapping_[j++]]->setVariableValues(vals + k);
	  }
	  else
	    ROS_ERROR("Cannot convert OMPL state to kinematic state");
}

void ompl_interface::JointModelStateSpaceHelper::copyToOMPLState(ob::State *state, const std::vector<double> &values) const
{
  state->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
  ob::CompoundState *cstate = state->as<ob::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < components_.size() ; ++i)
    if (components_[i]->getType() == ob::STATE_SPACE_SO2)
      cstate->as<ob::SO2StateSpace::StateType>(i)->value = values[variable_mapping_[j++]];
    else
      if (components_[i]->getType() == ob::STATE_SPACE_SE2)
      {
	cstate->as<ob::SE2StateSpace::StateType>(i)->setX(values[variable_mapping_[j]]);
	cstate->as<ob::SE2StateSpace::StateType>(i)->setY(values[variable_mapping_[j] + 1]);
	cstate->as<ob::SE2StateSpace::StateType>(i)->setYaw(values[variable_mapping_[j++] + 2]);
      }
      else
	if (components_[i]->getType() == ob::STATE_SPACE_SE3)
	{
	  cstate->as<ob::SE3StateSpace::StateType>(i)->setXYZ(values[variable_mapping_[j]], values[variable_mapping_[j] + 1], values[variable_mapping_[j] + 2]);
	  ob::SO3StateSpace::StateType &rot = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation();
	  rot.x = values[variable_mapping_[j] + 3];
	  rot.y = values[variable_mapping_[j] + 4];
	  rot.z = values[variable_mapping_[j] + 5];
	  rot.w = values[variable_mapping_[j++] + 6];
	}
	else
	  if (components_[i]->getType() == ob::STATE_SPACE_REAL_VECTOR)
	  {
	    double *vals = cstate->as<ob::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      vals[k] = values[variable_mapping_[j++]];
	  }
	  else
	    ROS_ERROR("Cannot convert vector of doubles to OMPL state");
}

void ompl_interface::JointModelStateSpaceHelper::copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const
{  
  state->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
  ob::CompoundState *cstate = state->as<ob::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < components_.size() ; ++i)
    if (components_[i]->getType() == ob::STATE_SPACE_SO2)
      cstate->as<ob::SO2StateSpace::StateType>(i)->value = js[joint_mapping_[j++]]->getVariableValues()[0];
    else
      if (components_[i]->getType() == ob::STATE_SPACE_SE2)
      {
	const std::vector<double> &values = js[joint_mapping_[j++]]->getVariableValues();
	cstate->as<ob::SE2StateSpace::StateType>(i)->setX(values[0]);
	cstate->as<ob::SE2StateSpace::StateType>(i)->setY(values[1]);
	cstate->as<ob::SE2StateSpace::StateType>(i)->setYaw(values[2]);
      }
      else
	if (components_[i]->getType() == ob::STATE_SPACE_SE3)
	{
	  const std::vector<double> &values = js[joint_mapping_[j++]]->getVariableValues();
	  cstate->as<ob::SE3StateSpace::StateType>(i)->setXYZ(values[0], values[1], values[2]);
	  ob::SO3StateSpace::StateType &rot = cstate->as<ob::SE3StateSpace::StateType>(i)->rotation();
	  rot.x = values[3];
	  rot.y = values[4];
	  rot.z = values[5];
	  rot.w = values[6];
	}
	else
	  if (components_[i]->getType() == ob::STATE_SPACE_REAL_VECTOR)
	  {
	    double *vals = cstate->as<ob::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      vals[k] = js[joint_mapping_[j++]]->getVariableValues()[0];
	  }
	  else
	    ROS_ERROR("Cannot convert kinematic state to OMPL state");
}

