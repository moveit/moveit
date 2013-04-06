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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/parameterization/model_based_joint_state_space.h>

ompl_interface::ModelBasedJointStateSpace::ModelBasedJointStateSpace(const robot_model::JointModel *joint_model,
                                                                     const robot_model::JointModel::Bounds &joint_bounds) :
  ompl::base::StateSpace(), joint_model_(joint_model), joint_bounds_(joint_bounds)
{
  // set the state space name
  setName(joint_model_->getName());
  
  if (joint_bounds_.size() != joint_model_->getVariableBounds().size())
  {
    logError("Joint '%s' from group has incorrect bounds specified. Using the default bounds instead.",  joint_model_->getName().c_str());
    joint_bounds_ = joint_model_->getVariableBounds();
  }
}

ompl_interface::ModelBasedJointStateSpace::ModelBasedJointStateSpace(const robot_model::JointModel *joint_model) :
  ompl::base::StateSpace(), joint_model_(joint_model)
{
  // set the state space name
  setName(joint_model_->getName());
  joint_bounds_ = joint_model_->getVariableBounds();
}

ompl_interface::ModelBasedJointStateSpace::~ModelBasedJointStateSpace()
{
}

ompl::base::State* ompl_interface::ModelBasedJointStateSpace::allocState() const
{
  StateType *st = new StateType();
  st->joint_state = new robot_state::JointState(joint_model_);
  return st;
}

void ompl_interface::ModelBasedJointStateSpace::freeState(ompl::base::State *state) const
{
  delete state->as<StateType>()->joint_state;
  delete state->as<StateType>();
}

unsigned int ompl_interface::ModelBasedJointStateSpace::getDimension() const
{
  return joint_model_->getStateSpaceDimension();
}

double ompl_interface::ModelBasedJointStateSpace::getMaximumExtent() const
{  
  return joint_model_->getMaximumExtent(joint_bounds_);
}

void ompl_interface::ModelBasedJointStateSpace::enforceBounds(ompl::base::State *state) const
{
  joint_model_->enforceBounds(state->as<StateType>()->joint_state->getVariableValues(), joint_bounds_);
}

bool ompl_interface::ModelBasedJointStateSpace::satisfiesBounds(const ompl::base::State *state) const
{
  return joint_model_->satisfiesBounds(state->as<StateType>()->joint_state->getVariableValues(), joint_bounds_, std::numeric_limits<double>::epsilon());
}

void ompl_interface::ModelBasedJointStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  *destination->as<StateType>()->joint_state = *source->as<StateType>()->joint_state;
}

double ompl_interface::ModelBasedJointStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  return state1->as<StateType>()->joint_state->distance(state2->as<StateType>()->joint_state);
}

bool ompl_interface::ModelBasedJointStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  return state1->as<StateType>()->joint_state->getVariableValues() == state2->as<StateType>()->joint_state->getVariableValues();
}

void ompl_interface::ModelBasedJointStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
  from->as<StateType>()->joint_state->interpolate(to->as<StateType>()->joint_state, t, state->as<StateType>()->joint_state);
  propagateJointStateUpdate(state);
}

unsigned int ompl_interface::ModelBasedJointStateSpace::getSerializationLength() const
{
  return sizeof(double) * joint_model_->getVariableCount();
}

void ompl_interface::ModelBasedJointStateSpace::serialize(void *serialization, const ompl::base::State *state) const
{
  memcpy(serialization, &state->as<StateType>()->joint_state->getVariableValues()[0], joint_model_->getVariableCount() * sizeof(double));
}

void ompl_interface::ModelBasedJointStateSpace::deserialize(ompl::base::State *state, const void *serialization) const
{ 
  memcpy(&state->as<StateType>()->joint_state->getVariableValues()[0], serialization, joint_model_->getVariableCount() * sizeof(double));
}

void ompl_interface::ModelBasedJointStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  if (joint_model_->getType() == robot_model::JointModel::PLANAR)
  {
    joint_bounds_[0].first = minX;
    joint_bounds_[0].second = maxX;
    joint_bounds_[1].first = minY;
    joint_bounds_[1].second = maxY;
  }
  else
    if (joint_model_->getType() == robot_model::JointModel::FLOATING)
    {
      joint_bounds_[0].first = minX;
      joint_bounds_[0].second = maxX;
      joint_bounds_[1].first = minY;
      joint_bounds_[1].second = maxY;
      joint_bounds_[2].first = minZ;
      joint_bounds_[2].second = maxZ;
    }
}

double* ompl_interface::ModelBasedJointStateSpace::getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const
{
  if (index >= joint_model_->getVariableCount())
    return NULL;
  return &state->as<StateType>()->joint_state->getVariableValues()[index];
}

void ompl_interface::ModelBasedJointStateSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
  out << "JointState(" << joint_model_->getName() << ") = [ ";
  for (std::size_t j = 0 ; j < state->as<StateType>()->joint_state->getVariableValues().size() ; ++j)
    out << state->as<StateType>()->joint_state->getVariableValues()[j] << " ";
  out << "]" << std::endl;
}

void ompl_interface::ModelBasedJointStateSpace::printSettings(std::ostream &out) const
{
  out << "ModelBasedJointStateSpace '" << getName() << "' at " << this << std::endl;
}

void ompl_interface::ModelBasedJointStateSpace::propagateJointStateUpdate(ompl::base::State *state) const
{
  // update the transform the joint applies
  joint_model_->updateTransform(state->as<StateType>()->joint_state->getVariableValues(), state->as<StateType>()->joint_state->getVariableTransform());
  
  // propagate updates to mimic joints
  state->as<StateType>()->joint_state->updateMimicJoints();
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedJointStateSpace::allocDefaultStateSampler() const
{
  class DefaultStateSampler : public ompl::base::StateSampler
  {
  public:
    
    DefaultStateSampler(const ompl::base::StateSpace *space) : ompl::base::StateSampler(space),
                                                               values_(space->as<ModelBasedJointStateSpace>()->getJointModel()->getVariableCount())
    {
    }
    
    virtual void sampleUniform(ompl::base::State *state)
    {
      std::vector<double> &values = state->as<ModelBasedJointStateSpace::StateType>()->joint_state->getVariableValues();
      values.clear(); // clear first since getRandomValues() does .push_back()
      
      // generate random values accorrding to JointModel
      space_->as<ModelBasedJointStateSpace>()->getJointModel()->getVariableRandomValues(moveit_rng_, values, space_->as<ModelBasedJointStateSpace>()->getJointBounds());
      
      // propagate transforms
      space_->as<ModelBasedJointStateSpace>()->propagateJointStateUpdate(state);
    }
    
    virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
    {     
      std::vector<double> &values = state->as<ModelBasedJointStateSpace::StateType>()->joint_state->getVariableValues();
      values.clear(); // clear first since getRandomValues() does .push_back()
      
      // generate random values accorrding to JointModel
      space_->as<ModelBasedJointStateSpace>()->getJointModel()->getVariableRandomValuesNearBy(moveit_rng_, values, space_->as<ModelBasedJointStateSpace>()->getJointBounds(),
                                                                                              near->as<ModelBasedJointStateSpace::StateType>()->joint_state->getVariableValues(),
                                                                                              distance);
      // propagate transforms
      space_->as<ModelBasedJointStateSpace>()->propagateJointStateUpdate(state);
    }
    
    virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
    {
      sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
    }
    
  protected:
    std::vector<double> values_;    
    random_numbers::RandomNumberGenerator moveit_rng_;
  };
  
  return ompl::base::StateSamplerPtr(static_cast<ompl::base::StateSampler*>(new DefaultStateSampler(this)));
}
