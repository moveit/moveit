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

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <boost/bind.hpp>

ompl_interface::ModelBasedStateSpace::ModelBasedStateSpace(const ModelBasedStateSpaceSpecification &spec) : ompl::base::CompoundStateSpace(), spec_(spec)
{
  // set the state space name
  setName(spec_.joint_model_group_->getName());
  
  // make sure we have bounds for every joint stored within the spec (use default bounds if not specified)
  const std::vector<const robot_model::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
  for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
    if (spec_.joints_bounds_.size() <= i)
      spec_.joints_bounds_.push_back(joint_model_vector[i]->getVariableBounds());
    else
      if (spec_.joints_bounds_[i].size() != joint_model_vector[i]->getVariableBounds().size())
      {
        logError("Joint '%s' from group '%s' has incorrect bounds specified. Using the default bounds instead.",
                 joint_model_vector[i]->getName().c_str(), spec_.joint_model_group_->getName().c_str());
        spec_.joints_bounds_[i] = joint_model_vector[i]->getVariableBounds();
      }
  
  // construct the state space components, subspace by subspace
  for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
    addSubspace(ompl::base::StateSpacePtr(new ModelBasedJointStateSpace(joint_model_vector[i], spec_.joints_bounds_[i])), joint_model_vector[i]->getDistanceFactor());
  jointSubspaceCount_ = joint_model_vector.size();
  
  // default settings
  setTagSnapToSegment(0.95);
  
  /// expose parameters
  params_.declareParam<double>("tag_snap_to_segment",
                               boost::bind(&ModelBasedStateSpace::setTagSnapToSegment, this, _1),
                               boost::bind(&ModelBasedStateSpace::getTagSnapToSegment, this));
}

ompl_interface::ModelBasedStateSpace::~ModelBasedStateSpace()
{
}

double ompl_interface::ModelBasedStateSpace::getTagSnapToSegment() const
{
  return tag_snap_to_segment_;
}

void ompl_interface::ModelBasedStateSpace::setTagSnapToSegment(double snap)
{
  if (snap < 0.0 || snap > 1.0)
    logWarn("Snap to segment for tags is a ratio. It's value must be between 0.0 and 1.0. Value remains as previously set (%lf)", tag_snap_to_segment_);
  else
  {
    tag_snap_to_segment_ = snap;
    tag_snap_to_segment_complement_ = 1.0 - tag_snap_to_segment_;
  }
}

ompl::base::State* ompl_interface::ModelBasedStateSpace::allocState() const
{ 
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl_interface::ModelBasedStateSpace::freeState(ompl::base::State *state) const
{
  CompoundStateSpace::freeState(state);
}

void ompl_interface::ModelBasedStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->tag = source->as<StateType>()->tag; 
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
  destination->as<StateType>()->distance = source->as<StateType>()->distance;
}

double ompl_interface::ModelBasedStateSpace::getMaximumExtent() const
{ 
  double total = 0.0;
  for (unsigned int i = 0 ; i < jointSubspaceCount_ ; ++i)
  {
    double e = components_[i]->getMaximumExtent();
    total += e * e * weights_[i];
  }
  return sqrt(total);
}

double ompl_interface::ModelBasedStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  double total = 0.0;
  for (unsigned int i = 0 ; i < jointSubspaceCount_ ; ++i)
  {
    double d = components_[i]->distance(state1->as<StateType>()->components[i], state2->as<StateType>()->components[i]);
    total += d * d * weights_[i];
  }
  return sqrt(total);
}

void ompl_interface::ModelBasedStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{  
  // clear any cached info (such as validity known or not)
  state->as<StateType>()->clearKnownInformation();

  // perform the actual interpolation
  CompoundStateSpace::interpolate(from, to, t, state);
  
  // compute tag
  if (from->as<StateType>()->tag >= 0 && t < 1.0 - tag_snap_to_segment_)
    state->as<StateType>()->tag = from->as<StateType>()->tag;
  else
    if (to->as<StateType>()->tag >= 0 && t > tag_snap_to_segment_)
      state->as<StateType>()->tag = to->as<StateType>()->tag;
    else
      state->as<StateType>()->tag = -1;
}

void ompl_interface::ModelBasedStateSpace::afterStateSample(ompl::base::State *sample) const
{ 
  sample->as<StateType>()->clearKnownInformation();
}

namespace ompl_interface
{
class WrappedStateSampler : public ompl::base::StateSampler
{
public:
  
  WrappedStateSampler(const ompl::base::StateSpace *space, const ompl::base::StateSamplerPtr &wrapped) : ompl::base::StateSampler(space), wrapped_(wrapped)
  {
  }
  
  virtual void sampleUniform(ompl::base::State *state)
  {
    wrapped_->sampleUniform(state);
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
  virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
  {    
    wrapped_->sampleUniformNear(state, near, distance);
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
  virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
  {
    wrapped_->sampleGaussian(state, mean, stdDev);
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
protected:
  
  ompl::base::StateSamplerPtr wrapped_;
};
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedStateSpace::allocSubspaceStateSampler(const ompl::base::StateSpace *subspace) const
{
  return ompl::base::StateSamplerPtr(new WrappedStateSampler(this, ompl::base::CompoundStateSpace::allocSubspaceStateSampler(subspace)));
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedStateSpace::allocStateSampler() const
{
  return ompl::base::StateSamplerPtr(new WrappedStateSampler(this, ompl::base::CompoundStateSpace::allocStateSampler()));
}

void ompl_interface::ModelBasedStateSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
  ompl::base::CompoundStateSpace::printState(state, out);
  if (state->as<StateType>()->isStartState())
    out << "* start state"  << std::endl;
  if (state->as<StateType>()->isGoalState())
    out << "* goal state"  << std::endl;
  if (state->as<StateType>()->isValidityKnown())
  {
    if (state->as<StateType>()->isMarkedValid())
      out << "* valid state"  << std::endl;
    else
      out << "* invalid state"  << std::endl;
  }
  out << "Tag: " << state->as<StateType>()->tag << std::endl;
}

void ompl_interface::ModelBasedStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  for (std::size_t i = 0 ; i < jointSubspaceCount_ ; ++i)
    components_[i]->as<ModelBasedJointStateSpace>()->setPlanningVolume(minX, maxX, minY, maxY, minZ, maxZ);
}

void ompl_interface::ModelBasedStateSpace::copyToRobotState(robot_state::JointStateGroup* jsg, const ompl::base::State *state) const
{
  const std::vector<robot_state::JointState*> &dest = jsg->getJointStateVector();
  for (std::size_t i = 0 ; i < dest.size() ; ++i)
    *dest[i] = *state->as<ompl::base::CompoundState>()->as<ModelBasedJointStateSpace::StateType>(i)->joint_state;
  jsg->updateLinkTransforms();
}

void ompl_interface::ModelBasedStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::JointStateGroup* jsg) const
{
  const std::vector<robot_state::JointState*> &src = jsg->getJointStateVector();
  for (std::size_t i = 0 ; i < src.size() ; ++i)
    *state->as<ompl::base::CompoundState>()->as<ModelBasedJointStateSpace::StateType>(i)->joint_state = *src[i];     
  state->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
}
