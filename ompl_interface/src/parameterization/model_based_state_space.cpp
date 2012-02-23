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

#include "ompl_interface/parameterization/model_based_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/console.h>

namespace ompl_interface
{
static void setPlanningVolumeAux(ob::StateSpace *space, double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  if (space->getType() == ob::STATE_SPACE_SE3)
  {
    ob::RealVectorBounds b(3);
    b.setLow(0, minX); b.setLow(1, minY); b.setLow(2, minZ);
    b.setHigh(0, maxX); b.setHigh(1, maxY); b.setHigh(2, maxZ);
    space->as<ob::SE3StateSpace>()->setBounds(b);
  }
  else
    if (space->getType() == ob::STATE_SPACE_SE2)
    {
      ob::RealVectorBounds b(2);
      b.setLow(0, minX); b.setLow(1, minY);
      b.setHigh(0, maxX); b.setHigh(1, maxY);
      space->as<ob::SE2StateSpace>()->setBounds(b);
    }
  if (space->isCompound())
  {
    const std::vector<ob::StateSpacePtr> &c = space->as<ob::CompoundStateSpace>()->getSubSpaces();
    for (std::size_t i = 0 ; i < c.size() ; ++i)
      setPlanningVolumeAux(c[i].get(), minX, maxX, minY, maxY, minZ, maxZ);
  }
}
}

void ompl_interface::ModelBasedStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  setPlanningVolumeAux(this, minX, maxX, minY, maxY, minZ, maxZ);
}

ompl::base::State* ompl_interface::ModelBasedStateSpace::allocState(void) const
{
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl_interface::ModelBasedStateSpace::freeState(ob::State *state) const
{
  CompoundStateSpace::freeState(state);
}

void ompl_interface::ModelBasedStateSpace::copyState(ob::State *destination, const ob::State *source) const
{
  CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
  destination->as<StateType>()->distance = source->as<StateType>()->distance;
  destination->as<StateType>()->tag = source->as<StateType>()->tag;
}

void ompl_interface::ModelBasedStateSpace::interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{  
  state->as<StateType>()->clearKnownInformation();
  CompoundStateSpace::interpolate(from, to, t, state);
  if (from->as<StateType>()->tag >= 0 && to->as<StateType>()->tag >= 0)
    state->as<StateType>()->tag = t < 0.5 ? from->as<StateType>()->tag : to->as<StateType>()->tag;
  else
    state->as<StateType>()->tag = std::max(from->as<StateType>()->tag, to->as<StateType>()->tag);
}

void ompl_interface::ModelBasedStateSpace::beforeStateSample(ob::State *sampled) const
{
  sampled->as<StateType>()->clearKnownInformation();
}

void ompl_interface::ModelBasedStateSpace::afterStateSample(ob::State *sampled) const
{
}

namespace ompl_interface
{
class ModelBasedStateSpace::WrappedStateSampler : public ob::StateSampler
{
public:
  
  WrappedStateSampler(const ob::StateSpace *space, const ob::StateSamplerPtr &wrapped) : ob::StateSampler(space), wrapped_(wrapped)
  {
  }
  
  virtual void sampleUniform(ob::State *state)
  {
    space_->as<ModelBasedStateSpace>()->beforeStateSample(state);
    wrapped_->sampleUniform(state);
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
  virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
  {  
    space_->as<ModelBasedStateSpace>()->beforeStateSample(state);
    wrapped_->sampleUniformNear(state, near, distance); 
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
  virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
  {
    space_->as<ModelBasedStateSpace>()->beforeStateSample(state);
    wrapped_->sampleGaussian(state, mean, stdDev);
    space_->as<ModelBasedStateSpace>()->afterStateSample(state);
  }
  
protected:
  
  ob::StateSamplerPtr wrapped_;    
};
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedStateSpace::allocStateSampler(void) const
{
  ompl::base::StateSamplerPtr ss = ob::CompoundStateSpace::allocStateSampler();
  if (dynamic_cast<WrappedStateSampler*>(ss.get()))
    return ss;
  else
    return ob::StateSamplerPtr(new WrappedStateSampler(this, ss));
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedStateSpace::allocDefaultStateSampler(void) const
{
  return ob::StateSamplerPtr(new WrappedStateSampler(this, ob::CompoundStateSpace::allocDefaultStateSampler()));
}

void ompl_interface::ModelBasedStateSpace::printState(const ob::State *state, std::ostream &out) const
{
  CompoundStateSpace::printState(state, out);
  out << "Tag: " << state->as<StateType>()->tag << std::endl;
  out << "Validity known: " << (state->as<StateType>()->isValidityKnown() ? "Yes" : "No") << std::endl;
  if (state->as<StateType>()->isValidityKnown())
    out << "Validity value: " << (state->as<StateType>()->isMarkedValid() ? "Yes" : "No") << std::endl;
  if (state->as<StateType>()->isGoalDistanceKnown())
    out << "Distance to goal: " << state->as<StateType>()->distance << std::endl;
}

void ompl_interface::ModelBasedStateSpace::copyToKinematicState(pm::KinematicState &kstate, const ob::State *state) const
{
  copyToKinematicState(kstate.getJointStateGroup(getJointModelGroupName())->getJointStateVector(), state);
}

void ompl_interface::ModelBasedStateSpace::copyToOMPLState(ob::State *state, const pm::KinematicState &kstate) const
{
  copyToOMPLState(state, kstate.getJointStateGroup(getJointModelGroupName())->getJointStateVector());
}

