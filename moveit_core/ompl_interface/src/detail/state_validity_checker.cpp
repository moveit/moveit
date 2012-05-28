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

#include "ompl_interface/detail/state_validity_checker.h"
#include "ompl_interface/model_based_planning_context.h"
#include <ompl/tools/debug/Profiler.h>

ompl_interface::StateValidityChecker::StateValidityChecker(const ModelBasedPlanningContext *pc) :
  ompl::base::StateValidityChecker(pc->getOMPLSimpleSetup().getSpaceInformation()), planning_context_(pc),
  group_name_(pc->getJointModelGroupName()), tss_(pc->getCompleteInitialRobotState()), verbose_(false)
{
  collision_request_with_distance_.distance = true;
}

void ompl_interface::StateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
  collision_request_simple_.verbose = flag;
  collision_request_with_distance_.verbose = flag;
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state) const
{  
  //  ompl::tools::Profiler::ScopedBlock sblock("isValid");
  
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();  
  
  planning_models::KinematicState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToKinematicState(*kstate, state);
  kstate->getJointStateGroup(group_name_)->updateLinkTransforms();
  
  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset && !kset->decide(*kstate, verbose_).satisfied)
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    
    if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
      kset->decide(*kstate, true);
    
    return false;
  }
  
  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose_))
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    
    if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
      planning_context_->getPlanningScene()->isStateFeasible(*kstate, true);
    
    return false;
  }  
  
  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_simple_, res, *kstate);
  if (res.collision == false)
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
    return true;
  }
  else
  {   
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    
    if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
    {
      collision_detection::CollisionRequest temp = collision_request_simple_;
      temp.verbose = true;
      planning_context_->getPlanningScene()->checkCollision(temp, res, *kstate);
    }
    
    return false;
  }
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
{
  //  ompl::tools::Profiler::ScopedBlock sblock("isValidD");
  
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown() && state->as<ModelBasedStateSpace::StateType>()->isGoalDistanceKnown())
  {
    dist = state->as<ModelBasedStateSpace::StateType>()->distance;
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();
  }
  
  planning_models::KinematicState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToKinematicState(*kstate, state);
  kstate->getJointStateGroup(group_name_)->updateLinkTransforms();
  
  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset)
  {
    kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*kstate, verbose_);
    if (!cer.satisfied)
    {
      dist = cer.distance;
      const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(dist); 
      
      if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
        kset->decide(*kstate, true);
      
      return false;
    }
  }
  
  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose_))
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    
    if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
      planning_context_->getPlanningScene()->isStateFeasible(*kstate, true);
    
    return false;
  }  
  
  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_distance_, res, *kstate);
  dist = res.distance;
  if (res.collision == false)
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid(dist);
    return true;
  }
  else
  {
    const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(dist);
    
    if (state->as<ModelBasedStateSpace::StateType>()->isInputState() && !verbose_)
    {
      collision_detection::CollisionRequest temp = collision_request_with_distance_;
      temp.verbose = true;
      planning_context_->getPlanningScene()->checkCollision(temp, res, *kstate);
    }
    
    return false;
  }
}
