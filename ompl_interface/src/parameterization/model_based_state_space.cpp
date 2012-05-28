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
#include <boost/bind.hpp>

ompl_interface::ModelBasedStateSpace::ModelBasedStateSpace(const ModelBasedStateSpaceSpecification &spec) : ompl::base::CompoundStateSpace(), spec_(spec)
{
  // set the state space name
  setName(spec_.joint_model_group_->getName());
  
  // make sure we have bounds for every joint stored within the spec (use default bounds if not specified)
  const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
  for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
    if (spec_.joints_bounds_.size() <= i)
      spec_.joints_bounds_.push_back(joint_model_vector[i]->getVariableBounds());
    else
      if (spec_.joints_bounds_[i].size() != joint_model_vector[i]->getVariableBounds().size())
      {
        ROS_ERROR_STREAM("Joint " << joint_model_vector[i]->getName() << "  from group " << spec_.joint_model_group_->getName() << " has incorrect bounds specified. Using the default bounds instead.");
        spec_.joints_bounds_[i] = joint_model_vector[i]->getVariableBounds();
      }
  
  // construct the state space components, subspace by subspace
  for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
    addSubspace(ompl::base::StateSpacePtr(new ModelBasedJointStateSpace(joint_model_vector[i], spec_.joints_bounds_[i])), joint_model_vector[i]->getDistanceFactor());
  
  // default settings
  setTagSnapToSegment(0.95);
  
  /// expose parameters
  params_.declareParam<double>("tag_snap_to_segment", msg_,
                               boost::bind(&ModelBasedStateSpace::setTagSnapToSegment, this, _1),
                               boost::bind(&ModelBasedStateSpace::getTagSnapToSegment, this));
}

ompl_interface::ModelBasedStateSpace::~ModelBasedStateSpace(void)
{
}

double ompl_interface::ModelBasedStateSpace::getTagSnapToSegment(void) const
{
  return tag_snap_to_segment_;
}

void ompl_interface::ModelBasedStateSpace::setTagSnapToSegment(double snap)
{
  if (snap < 0.0 || snap > 1.0)
    ROS_WARN("Snap to segment for tags is a ratio. It's value must be between 0.0 and 1.0. Value remains as previously set (%lf)", tag_snap_to_segment_);
  else
  {
    tag_snap_to_segment_ = snap;
    tag_snap_to_segment_complement_ = 1.0 - tag_snap_to_segment_;
  }
}

ompl::base::State* ompl_interface::ModelBasedStateSpace::allocState(void) const
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

void ompl_interface::ModelBasedStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{ 
  state->as<StateType>()->clearKnownInformation();
  CompoundStateSpace::interpolate(from, to, t, state);
  
  if (from->as<StateType>()->tag >= 0 && t < 1.0 - tag_snap_to_segment_)
    state->as<StateType>()->tag = from->as<StateType>()->tag;
  else
    if (to->as<StateType>()->tag >= 0 && t > tag_snap_to_segment_)
      state->as<StateType>()->tag = to->as<StateType>()->tag;
    else
      state->as<StateType>()->tag = -1;
}

void ompl_interface::ModelBasedStateSpace::setBounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  for (std::size_t i = 0 ; i < componentCount_ ; ++i)
    components_[i]->as<ModelBasedJointStateSpace>()->setBounds(minX, maxX, minY, maxY, minZ, maxZ);
}
