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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/ompl_interface/detail/constraints_library.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/profiler/profiler.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl_interface
{
class ConstraintApproximationStateSampler : public ompl::base::StateSampler
{
public:
  ConstraintApproximationStateSampler(const ompl::base::StateSpace* space,
                                      const ConstraintApproximationStateStorage* state_storage, std::size_t milestones)
    : ompl::base::StateSampler(space), state_storage_(state_storage)
  {
    max_index_ = milestones - 1;
    inv_dim_ = space->getDimension() > 0 ? 1.0 / (double)space->getDimension() : 1.0;
  }

  virtual void sampleUniform(ompl::base::State* state)
  {
    space_->copyState(state, state_storage_->getState(rng_.uniformInt(0, max_index_)));
  }

  virtual void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance)
  {
    int index = -1;
    int tag = near->as<ModelBasedStateSpace::StateType>()->tag;

    if (tag >= 0)
    {
      const ConstrainedStateMetadata& md = state_storage_->getMetadata(tag);
      if (!md.first.empty())
      {
        std::size_t matt = md.first.size() / 3;
        std::size_t att = 0;
        do
        {
          index = md.first[rng_.uniformInt(0, md.first.size() - 1)];
        } while (dirty_.find(index) != dirty_.end() && ++att < matt);
        if (att >= matt)
          index = -1;
        else
          dirty_.insert(index);
      }
    }
    if (index < 0)
      index = rng_.uniformInt(0, max_index_);

    double dist = space_->distance(near, state_storage_->getState(index));

    if (dist > distance)
    {
      double d = pow(rng_.uniform01(), inv_dim_) * distance;
      space_->interpolate(near, state_storage_->getState(index), d / dist, state);
    }
    else
      space_->copyState(state, state_storage_->getState(index));
  }

  virtual void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev)
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }

protected:
  /** \brief The states to sample from */
  const ConstraintApproximationStateStorage* state_storage_;
  std::set<std::size_t> dirty_;
  unsigned int max_index_;
  double inv_dim_;
};

bool interpolateUsingStoredStates(const ConstraintApproximationStateStorage* state_storage,
                                  const ompl::base::State* from, const ompl::base::State* to, const double t,
                                  ompl::base::State* state)
{
  int tag_from = from->as<ModelBasedStateSpace::StateType>()->tag;
  int tag_to = to->as<ModelBasedStateSpace::StateType>()->tag;

  if (tag_from < 0 || tag_to < 0)
    return false;

  if (tag_from == tag_to)
    state_storage->getStateSpace()->copyState(state, to);
  else
  {
    const ConstrainedStateMetadata& md = state_storage->getMetadata(tag_from);

    std::map<std::size_t, std::pair<std::size_t, std::size_t> >::const_iterator it = md.second.find(tag_to);
    if (it == md.second.end())
      return false;
    const std::pair<std::size_t, std::size_t>& istates = it->second;
    std::size_t index = (std::size_t)((istates.second - istates.first + 2) * t + 0.5);

    if (index == 0)
      state_storage->getStateSpace()->copyState(state, from);
    else
    {
      --index;
      if (index >= istates.second - istates.first)
        state_storage->getStateSpace()->copyState(state, to);
      else
        state_storage->getStateSpace()->copyState(state, state_storage->getState(istates.first + index));
    }
  }
  return true;
}

InterpolationFunction ConstraintApproximation::getInterpolationFunction() const
{
  if (explicit_motions_ && milestones_ > 0 && milestones_ < state_storage_->size())
    return boost::bind(&interpolateUsingStoredStates, state_storage_, _1, _2, _3, _4);
  return InterpolationFunction();
}

ompl::base::StateSamplerPtr allocConstraintApproximationStateSampler(
    const ompl::base::StateSpace* space, const std::vector<int>& expected_signature,
    const ConstraintApproximationStateStorage* state_storage, std::size_t milestones)
{
  std::vector<int> sig;
  space->computeSignature(sig);
  if (sig != expected_signature)
    return ompl::base::StateSamplerPtr();
  else
    return ompl::base::StateSamplerPtr(new ConstraintApproximationStateSampler(space, state_storage, milestones));
}

ConstraintApproximation::ConstraintApproximation(const std::string& group,
                                                 const std::string& state_space_parameterization, bool explicit_motions,
                                                 const moveit_msgs::Constraints& msg, const std::string& filename,
                                                 const ompl::base::StateStoragePtr& storage, std::size_t milestones)
  : group_(group)
  , state_space_parameterization_(state_space_parameterization)
  , explicit_motions_(explicit_motions)
  , constraint_msg_(msg)
  , ompldb_filename_(filename)
  , state_storage_ptr_(storage)
  , milestones_(milestones)
{
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
  if (milestones_ == 0)
    milestones_ = state_storage_->size();
}

ompl::base::StateSamplerAllocator
ConstraintApproximation::getStateSamplerAllocator(const moveit_msgs::Constraints& msg) const
{
  if (state_storage_->size() == 0)
    return ompl::base::StateSamplerAllocator();
  return boost::bind(&allocConstraintApproximationStateSampler, _1, space_signature_, state_storage_, milestones_);
}
}  // ompl_interface
