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

#include "ompl_interface/detail/kinematic_model_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/console.h>

ompl_interface::KMStateSpace::KMStateSpace(const planning_models::KinematicModel::JointModelGroup* jmg) : ompl::base::CompoundStateSpace(), jmg_(jmg)
{
  constructSpace(jmg_->getJointModels());
  setName(jmg_->getName());
}

ompl_interface::KMStateSpace::KMStateSpace(const std::vector<const planning_models::KinematicModel::JointModel*> &joints) : ompl::base::CompoundStateSpace(), jmg_(NULL)
{
  constructSpace(joints);
  std::string nm;
  for (std::size_t i = 0 ; i < components_.size() ; ++i)
  {
    if (!nm.empty())
      nm += ",";
    nm += components_[i]->getName();
  }
  setName(nm);
}

ompl_interface::KMStateSpace::~KMStateSpace(void)
{
}

ompl::base::State* ompl_interface::KMStateSpace::allocState(void) const
{
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl_interface::KMStateSpace::freeState(ompl::base::State *state) const
{
  CompoundStateSpace::freeState(state);
}

void ompl_interface::KMStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->tag = source->as<StateType>()->tag;  
}

void ompl_interface::KMStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
  CompoundStateSpace::interpolate(from, to, t, state);
  if (from->as<StateType>()->tag >= 0 && to->as<StateType>()->tag >= 0)
    state->as<StateType>()->tag = t < 0.5 ? from->as<StateType>()->tag : to->as<StateType>()->tag;
  else
    state->as<StateType>()->tag = std::max(from->as<StateType>()->tag, to->as<StateType>()->tag);
}

void ompl_interface::KMStateSpace::copyToKinematicState(planning_models::KinematicState &kstate, const ompl::base::State *state) const
{
  if (jmg_)
    copyToKinematicState(kstate.getJointStateGroup(jmg_->getName())->getJointStateVector(), state);
  else
  {
    std::vector<planning_models::KinematicState::JointState*> js(joints_.size(), NULL);
    for (std::size_t i = 0 ; i < joints_.size() ; ++i)
      js[i] = kstate.getJointState(joints_[i]->getName());
    copyToKinematicState(js, state);
  }
}

void ompl_interface::KMStateSpace::copyToKinematicState(const std::vector<planning_models::KinematicState::JointState*> &js, const ompl::base::State *state) const
{
  const ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    if (components_[i]->getType() == ompl::base::STATE_SPACE_SO2)
      js[joint_mapping_[j++]]->setVariableValues(&cstate->as<ompl::base::SO2StateSpace::StateType>(i)->value);
    else
      if (components_[i]->getType() == ompl::base::STATE_SPACE_SE2)
      {
	std::vector<double> values(3);
	values[0] = cstate->as<ompl::base::SE2StateSpace::StateType>(i)->getX();
	values[1] = cstate->as<ompl::base::SE2StateSpace::StateType>(i)->getY();
	values[2] = cstate->as<ompl::base::SE2StateSpace::StateType>(i)->getYaw();
	js[joint_mapping_[j++]]->setVariableValues(values);
      }
      else
	if (components_[i]->getType() == ompl::base::STATE_SPACE_SE3)
	{
	  std::vector<double> values(7);
	  values[0] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->getX();
	  values[1] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->getY();
	  values[2] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->getZ();
	  values[3] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation().x;
	  values[4] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation().y;
	  values[5] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation().z;
	  values[6] = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation().w;
	  js[joint_mapping_[j++]]->setVariableValues(values);
	}
	else
	  if (components_[i]->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
	  {
	    const double *vals = cstate->as<ompl::base::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      js[joint_mapping_[j++]]->setVariableValues(vals + k);
	  }
	  else
	    ROS_ERROR("Cannot convert OMPL state to kinematic state");
}

void ompl_interface::KMStateSpace::copyToOMPLState(ompl::base::State *state, const planning_models::KinematicState &kstate) const
{
  if (jmg_)
    copyToOMPLState(state, kstate.getJointStateGroup(jmg_->getName())->getJointStateVector());
  else
  {
    std::vector<planning_models::KinematicState::JointState*> js(joints_.size(), NULL);
    for (std::size_t i = 0 ; i < joints_.size() ; ++i)
      js[i] = kstate.getJointState(joints_[i]->getName());
    copyToOMPLState(state, js);
  }
}

void ompl_interface::KMStateSpace::copyToOMPLState(ompl::base::State *state, const std::vector<double> &values) const
{
  ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    if (components_[i]->getType() == ompl::base::STATE_SPACE_SO2)
      cstate->as<ompl::base::SO2StateSpace::StateType>(i)->value = values[variable_mapping_[j++]];
    else
      if (components_[i]->getType() == ompl::base::STATE_SPACE_SE2)
      {
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setX(values[variable_mapping_[j]]);
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setY(values[variable_mapping_[j] + 1]);
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setYaw(values[variable_mapping_[j++] + 2]);
      }
      else
	if (components_[i]->getType() == ompl::base::STATE_SPACE_SE3)
	{
	  cstate->as<ompl::base::SE3StateSpace::StateType>(i)->setXYZ(values[variable_mapping_[j]], values[variable_mapping_[j] + 1], values[variable_mapping_[j] + 2]);
	  ompl::base::SO3StateSpace::StateType &rot = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation();
	  rot.x = values[variable_mapping_[j] + 3];
	  rot.y = values[variable_mapping_[j] + 4];
	  rot.z = values[variable_mapping_[j] + 5];
	  rot.w = values[variable_mapping_[j++] + 6];
	}
	else
	  if (components_[i]->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
	  {
	    double *vals = cstate->as<ompl::base::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      vals[k] = values[variable_mapping_[j++]];
	  }
	  else
	    ROS_ERROR("Cannot convert vector of doubles to OMPL state");
}

void ompl_interface::KMStateSpace::copyToOMPLState(ompl::base::State *state, const std::vector<planning_models::KinematicState::JointState*> &js) const
{
  ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
  unsigned int j = 0;
  for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    if (components_[i]->getType() == ompl::base::STATE_SPACE_SO2)
      cstate->as<ompl::base::SO2StateSpace::StateType>(i)->value = js[joint_mapping_[j++]]->getVariableValues()[0];
    else
      if (components_[i]->getType() == ompl::base::STATE_SPACE_SE2)
      {
	const std::vector<double> &values = js[joint_mapping_[j++]]->getVariableValues();
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setX(values[0]);
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setY(values[1]);
	cstate->as<ompl::base::SE2StateSpace::StateType>(i)->setYaw(values[2]);
      }
      else
	if (components_[i]->getType() == ompl::base::STATE_SPACE_SE3)
	{
	  const std::vector<double> &values = js[joint_mapping_[j++]]->getVariableValues();
	  cstate->as<ompl::base::SE3StateSpace::StateType>(i)->setXYZ(values[0], values[1], values[2]);
	  ompl::base::SO3StateSpace::StateType &rot = cstate->as<ompl::base::SE3StateSpace::StateType>(i)->rotation();
	  rot.x = values[3];
	  rot.y = values[4];
	  rot.z = values[5];
	  rot.w = values[6];
	}
	else
	  if (components_[i]->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
	  {
	    double *vals = cstate->as<ompl::base::RealVectorStateSpace::StateType>(i)->values;
	    const unsigned int d = components_[i]->getDimension();
	    for (unsigned int k = 0 ; k < d ; ++k)
	      vals[k] = js[joint_mapping_[j++]]->getVariableValues()[0];
	  }
	  else
	    ROS_ERROR("Cannot convert kinematic state to OMPL state");
}

void ompl_interface::KMStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    if (components_[i]->getType() == ompl::base::STATE_SPACE_SE3)
    {
      ompl::base::RealVectorBounds b(3);
      b.setLow(0, minX); b.setLow(1, minY); b.setLow(2, minZ);
      b.setHigh(0, maxX); b.setHigh(1, maxY); b.setHigh(2, maxZ);
      components_[i]->as<ompl::base::SE3StateSpace>()->setBounds(b);
    }
    else
      if (components_[i]->getType() == ompl::base::STATE_SPACE_SE2)
      {
	ompl::base::RealVectorBounds b(2);
	b.setLow(0, minX); b.setLow(1, minY);
	b.setHigh(0, maxX); b.setHigh(1, maxY);
	components_[i]->as<ompl::base::SE2StateSpace>()->setBounds(b);
      }
}

void ompl_interface::KMStateSpace::constructSpace(const std::vector<const planning_models::KinematicModel::JointModel*> &joints)
{
  ompl::base::StateSpacePtr space;
  
  joints_ = joints;
  joint_mapping_.clear();
  variable_mapping_.clear();
  
  ompl::base::RealVectorStateSpace *rv = NULL;
  std::vector<std::size_t> rv_joints;
  std::vector<std::size_t> rv_var;
  std::size_t nvar = 0;
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    ompl::base::StateSpacePtr to_add;
    
    const planning_models::KinematicModel::RevoluteJointModel* revolute_joint =
      dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(joints[i]);
    if (revolute_joint && revolute_joint->isContinuous())
    {
      ompl::base::SO2StateSpace *space = new ompl::base::SO2StateSpace();
      space->setName(revolute_joint->getName());
      joint_mapping_.push_back(i);
      variable_mapping_.push_back(nvar++);
      to_add.reset(space);
    }
    else
    {
      const planning_models::KinematicModel::PlanarJointModel* planar_joint =
	dynamic_cast<const planning_models::KinematicModel::PlanarJointModel*>(joints[i]);
      if (planar_joint)
      {
	ompl::base::SE2StateSpace *space = new ompl::base::SE2StateSpace();
	space->setName(planar_joint->getName());
	joint_mapping_.push_back(i);
	variable_mapping_.push_back(nvar);
	nvar += 3;
	to_add.reset(space);
      }
      else
      {
	const planning_models::KinematicModel::FloatingJointModel* floating_joint =
	  dynamic_cast<const planning_models::KinematicModel::FloatingJointModel*>(joints[i]);
	if (floating_joint)
	{
	  ompl::base::SE3StateSpace *space = new ompl::base::SE3StateSpace();
	  space->setName(floating_joint->getName());
	  joint_mapping_.push_back(i);
	  variable_mapping_.push_back(nvar);
	  nvar += 7;
	  to_add.reset(space);
	}
	else
	{
	  const planning_models::KinematicModel::PrismaticJointModel* prismatic_joint =
	    dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>(joints[i]);
	  if (revolute_joint || prismatic_joint)
	  {
	    if (!rv)
	      rv = new ompl::base::RealVectorStateSpace();
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
    space = space + ompl::base::StateSpacePtr(rv);
  }
  
  if (!space)
  {
    ROS_WARN("Empty OMPL state space");
    return;
  }
  
  // we make the assumption later on this is a compound space, so ensure this is always the case:
  if (!space->isCompound() || (space->isCompound() && space->as<ompl::base::CompoundStateSpace>()->isLocked()))
  {
    ompl::base::CompoundStateSpace *csm = new ompl::base::CompoundStateSpace();
    csm->addSubSpace(space, 1.0);
    space.reset(csm);
  }

  // heuristically set some weights for the subspaces, based on dimension
  unsigned int ns = space->as<ompl::base::CompoundStateSpace>()->getSubSpaceCount();
  
  for (unsigned int i = 0 ; i < ns ; ++i)
  {
    const ompl::base::StateSpacePtr &c = space->as<ompl::base::CompoundStateSpace>()->getSubSpace(i);
    addSubSpace(c, (double)c->getDimension());
  }
  lock();
  
  setPlanningVolume(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
}
