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

#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>

ompl_interface::ConstrainedGoalSampler::ConstrainedGoalSampler(const ModelBasedPlanningContext *pc,
                                                               const kinematic_constraints::KinematicConstraintSetPtr &ks,
                                                               const constraint_samplers::ConstraintSamplerPtr &cs) :
  ob::GoalLazySamples(pc->getOMPLSimpleSetup().getSpaceInformation(), boost::bind(&ConstrainedGoalSampler::sampleUsingConstraintSampler, this, _1, _2), false),
  planning_context_(pc), kinematic_constraint_set_(ks), constraint_sampler_(cs), work_state_(pc->getCompleteInitialRobotState()),
  work_joint_group_state_(work_state_.getJointStateGroup(planning_context_->getJointModelGroupName())), verbose_display_(0)
{
  if (!constraint_sampler_)
    default_sampler_ = si_->allocStateSampler();
  logDebug("Constructed a ConstrainedGoalSampler instance at address %p", this);
  startSampling();
}

bool ompl_interface::ConstrainedGoalSampler::sampleUsingConstraintSampler(const ob::GoalLazySamples *gls, ob::State *newGoal)
{
  unsigned int ma = planning_context_->getMaximumGoalSamplingAttempts();
  
  // terminate after too many attempts
  if (gls->samplingAttemptsCount() >= ma)
    return false;
  
  // terminate after a maximum number of samples
  if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
    return false;
  
  // terminate the sampling thread when a solution has been found
  if (planning_context_->getOMPLSimpleSetup().getProblemDefinition()->hasSolution())
    return false;

  unsigned int ma2 = ma/2;
  for (unsigned int a = gls->samplingAttemptsCount() ; a < ma && gls->isSampling() ; ++a)
  {
    bool verbose = false;
    if (gls->getStateCount() == 0 && a >= ma2)
      if (verbose_display_ < 1)
      {
	verbose = true;
	verbose_display_++;
      }

    if (constraint_sampler_)
    {
      if (constraint_sampler_->sample(work_joint_group_state_, planning_context_->getCompleteInitialRobotState(), planning_context_->getMaximumStateSamplingAttempts()))
      { 
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
        {
          planning_context_->getOMPLStateSpace()->copyToOMPLState(newGoal, work_joint_group_state_);
          if (static_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(newGoal, verbose))
            return true;
        }
      }
    }
    else
    {
      default_sampler_->sampleUniform(newGoal);
      if (static_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(newGoal, verbose))
      {
        planning_context_->getOMPLStateSpace()->copyToRobotState(work_joint_group_state_, newGoal);
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          return true;
      }
    }
  }
  return false;
}
