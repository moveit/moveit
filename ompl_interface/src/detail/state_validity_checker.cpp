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

ompl_interface::StateValidityChecker::StateValidityChecker(const PlanningConfiguration *pc) :
  ompl::base::StateValidityChecker(pc->getOMPLSimpleSetup().getSpaceInformation()), planning_config_(pc),
  group_name_(planning_config_->getJointModelGroupName())
{
  collision_request_with_distance_.distance = true;
}

void ompl_interface::StateValidityChecker::useNewStartingState(void)
{
  tss_.reset(new TSStateStorage(planning_config_->getStartState()));
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state) const
{  
  //  ompl::Profiler::ScopedBlock sblock("isValid");

  planning_models::KinematicState *kstate = tss_->getStateStorage();
  planning_config_->getKMStateSpace().copyToKinematicState(*kstate, state);
  kstate->getJointStateGroup(group_name_)->updateLinkTransforms();
  
  double distance = 0.0;
  if (!planning_config_->getPathConstraints()->decide(*kstate, distance))
    return false;
  
  collision_detection::CollisionResult res;
  planning_config_->getPlanningScene()->checkCollision(collision_request_simple_, res, *kstate);
  return res.collision == false;
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
{
  //  ompl::Profiler::ScopedBlock sblock("isValid");
  
  planning_models::KinematicState *kstate = tss_->getStateStorage();
  planning_config_->getKMStateSpace().copyToKinematicState(*kstate, state);
  kstate->getJointStateGroup(group_name_)->updateLinkTransforms();
  
  double distance = 0.0;
  if (!planning_config_->getPathConstraints()->decide(*kstate, distance))
  {
    dist = distance;
    return false;
  }
  collision_detection::CollisionResult res;
  planning_config_->getPlanningScene()->checkCollision(collision_request_with_distance_, res, *kstate);
  dist = res.distance;
  return res.collision == false;
}
