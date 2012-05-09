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

#include "ompl_interface/detail/constrained_goal_sampler.h"
#include "ompl_interface/model_based_planning_context.h"
#include <ompl/geometric/ik/GAIK.h>

ompl_interface::ConstrainedGoalSampler::ConstrainedGoalSampler(const ModelBasedPlanningContext *pc,
                                                               const kc::KinematicConstraintSetPtr &ks,
                                                               const constraint_samplers::ConstraintSamplerPtr &cs) :
  ob::GoalLazySamples(pc->getOMPLSimpleSetup().getSpaceInformation(),
                      cs ? boost::bind(&ConstrainedGoalSampler::sampleUsingConstraintSampler, this, _1, _2) :
                      boost::bind(&ConstrainedGoalSampler::sampleUsingGAIK, this, _1, _2), false),
  planning_context_(pc), kinematic_constraint_set_(ks), constraint_sampler_(cs), state_(pc->getCompleteInitialRobotState())
{
  ROS_DEBUG("Constructed a ConstrainedGoalSampler instance at address %p", this);
  startSampling();
}
  
bool ompl_interface::ConstrainedGoalSampler::sampleUsingGAIK(const ob::GoalLazySamples *gls, ob::State *newGoal)
{
  unsigned int ma = planning_context_->getMaximumGoalSamplingAttempts();

  // terminate after too many attempts
  if (gls->samplingAttemptsCount() >= ma)
    return false;
  // terminate after a maximum number of samples
  if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
    return false;
  
  // this class is NOT a valid goal region from a thread safety point of view;
  // HOWEVER, it is only used with GAIK, which is single-threaded, so this is safe
  class ConstrainedGoalRegion : public ob::GoalRegion
  {
  public:
    ConstrainedGoalRegion(const ModelBasedPlanningContext *pc, const kc::KinematicConstraintSet *ks, pm::KinematicState *state) :
      ob::GoalRegion(pc->getOMPLSimpleSetup().getSpaceInformation()), planning_context_(pc), kinematic_constraint_set_(ks),
      state_(state), joint_state_group_(state_->getJointStateGroup(pc->getJointModelGroupName()))
    {
    }
    
    virtual double distanceGoal(const ob::State *st) const
    {
      planning_context_->getOMPLStateSpace()->copyToKinematicState(*state_, st);
      joint_state_group_->updateLinkTransforms();
      return kinematic_constraint_set_->decide(*state_).distance;
    }
    
    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
      planning_context_->getOMPLStateSpace()->copyToKinematicState(*state_, st);
      joint_state_group_->updateLinkTransforms();
      kinematic_constraints::ConstraintEvaluationResult cer = kinematic_constraint_set_->decide(*state_);
      if (distance)
        *distance = cer.distance;
      return cer.satisfied;
    }
    
  protected:
    
    const ModelBasedPlanningContext     *planning_context_;
    const kc::KinematicConstraintSet    *kinematic_constraint_set_;
    pm::KinematicState                  *state_;
    pm::KinematicState::JointStateGroup *joint_state_group_;
  };
  
  ConstrainedGoalRegion reg(planning_context_, kinematic_constraint_set_.get(), &state_);
  ompl::geometric::GAIK g(si_);
  while (gls->isSampling())
    if (g.solve(0.1, reg, newGoal))
    {
      newGoal->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
      newGoal->as<ModelBasedStateSpace::StateType>()->markGoalState();
      return true;
    }
  return false;
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

  std::vector<double> values;
  for (unsigned int a = 0 ; a < ma && gls->isSampling() ; ++a)
    if (constraint_sampler_->sample(values, planning_context_->getCompleteInitialRobotState(), planning_context_->getMaximumStateSamplingAttempts()))
    {
      state_.getJointStateGroup(planning_context_->getJointModelGroupName())->setStateValues(values);
      if (kinematic_constraint_set_->decide(state_).satisfied)
      {
        planning_context_->getOMPLStateSpace()->copyToOMPLState(newGoal, values);   
        newGoal->as<ModelBasedStateSpace::StateType>()->markGoalState();
        return true;
      }
    }
  return false;
}
