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

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

ompl_interface::StateValidityChecker::StateValidityChecker(const ModelBasedPlanningContext *pc)
  : ompl::base::StateValidityChecker(pc->getOMPLSimpleSetup()->getSpaceInformation())
  , planning_context_(pc)
  , group_name_(pc->getGroupName())
  , tss_(pc->getCompleteInitialRobotState())
  , verbose_(false)
{
  specs_.clearanceComputationType = ompl::base::StateValidityCheckerSpecs::APPROXIMATE;
  specs_.hasValidDirectionComputation = false;

  collision_request_with_distance_.distance = true;
  collision_request_with_cost_.cost = true;

  collision_request_simple_.group_name = planning_context_->getGroupName();
  collision_request_with_distance_.group_name = planning_context_->getGroupName();
  collision_request_with_cost_.group_name = planning_context_->getGroupName();

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;

  collision_request_with_distance_verbose_ = collision_request_with_distance_;
  collision_request_with_distance_verbose_.verbose = true;
}

void ompl_interface::StateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state, bool verbose) const
{
  //  moveit::Profiler::ScopedBlock sblock("isValid");
  return planning_context_->useStateValidityCache() ? isValidWithCache(state, verbose) :
                                                      isValidWithoutCache(state, verbose);
}

bool ompl_interface::StateValidityChecker::isValid(const ompl::base::State *state, double &dist, bool verbose) const
{
  //  moveit::Profiler::ScopedBlock sblock("isValid");
  return planning_context_->useStateValidityCache() ? isValidWithCache(state, dist, verbose) :
                                                      isValidWithoutCache(state, dist, verbose);
}

double ompl_interface::StateValidityChecker::cost(const ompl::base::State *state) const
{
  double cost = 0.0;

  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_cost_, res, *kstate);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin();
       it != res.cost_sources.end(); ++it)
    cost += it->cost * it->getVolume();

  return cost;
}

double ompl_interface::StateValidityChecker::clearance(const ompl::base::State *state) const
{
  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_distance_, res, *kstate);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}

bool ompl_interface::StateValidityChecker::isValidWithoutCache(const ompl::base::State *state, bool verbose) const
{
  // check bounds
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      logInform("State outside bounds");
    return false;
  }

  // convert ompl state to moveit robot state
  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset && !kset->decide(*kstate, verbose).satisfied)
    return false;

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose))
    return false;

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_simple_verbose_ : collision_request_simple_, res, *kstate);
  return res.collision == false;
}

bool ompl_interface::StateValidityChecker::isValidWithoutCache(const ompl::base::State *state, double &dist,
                                                               bool verbose) const
{
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      logInform("State outside bounds");
    return false;
  }

  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset)
  {
    kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*kstate, verbose);
    if (!cer.satisfied)
    {
      dist = cer.distance;
      return false;
    }
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose))
  {
    dist = 0.0;
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_with_distance_verbose_ : collision_request_with_distance_, res, *kstate);
  dist = res.distance;
  return res.collision == false;
}

bool ompl_interface::StateValidityChecker::isValidWithCache(const ompl::base::State *state, bool verbose) const
{
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      logInform("State outside bounds");
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset && !kset->decide(*kstate, verbose).satisfied)
  {
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose))
  {
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_simple_verbose_ : collision_request_simple_, res, *kstate);
  if (res.collision == false)
  {
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
    return true;
  }
  else
  {
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
    return false;
  }
}

bool ompl_interface::StateValidityChecker::isValidWithCache(const ompl::base::State *state, double &dist,
                                                            bool verbose) const
{
  if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown() &&
      state->as<ModelBasedStateSpace::StateType>()->isGoalDistanceKnown())
  {
    dist = state->as<ModelBasedStateSpace::StateType>()->distance;
    return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();
  }

  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      logInform("State outside bounds");
    const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(0.0);
    return false;
  }

  robot_state::RobotState *kstate = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

  // check path constraints
  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  if (kset)
  {
    kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*kstate, verbose);
    if (!cer.satisfied)
    {
      dist = cer.distance;
      const_cast<ob::State *>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid(dist);
      return false;
    }
  }

  // check feasibility
  if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate, verbose))
  {
    dist = 0.0;
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose ? collision_request_with_distance_verbose_ : collision_request_with_distance_, res, *kstate);
  dist = res.distance;
  return res.collision == false;
}
