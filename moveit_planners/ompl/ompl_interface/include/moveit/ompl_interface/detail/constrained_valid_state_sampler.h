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

#ifndef MOVEIT_OMPL_INTERFACE_DETAIL_CONSTRAINED_VALID_STATE_SAMPLER_
#define MOVEIT_OMPL_INTERFACE_DETAIL_CONSTRAINED_VALID_STATE_SAMPLER_

#include <ompl/base/StateSampler.h>
#include <ompl/base/ValidStateSampler.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/macros/class_forward.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

MOVEIT_CLASS_FORWARD(ValidStateSampler);

/** @class ValidConstrainedSampler
 *  This class defines a sampler that tries to find a valid sample that satisfies the specified constraints */
class ValidConstrainedSampler : public ompl::base::ValidStateSampler
{
public:
  ValidConstrainedSampler(
      const ModelBasedPlanningContext* pc, const kinematic_constraints::KinematicConstraintSetPtr& ks,
      const constraint_samplers::ConstraintSamplerPtr& cs = constraint_samplers::ConstraintSamplerPtr());

  virtual bool sample(ompl::base::State* state);
  virtual bool project(ompl::base::State* state);
  virtual bool sampleNear(ompl::base::State* state, const ompl::base::State* near, const double distance);

private:
  const ModelBasedPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  robot_state::RobotState work_state_;
  double inv_dim_;
  ompl::RNG rng_;
};
}

#endif
