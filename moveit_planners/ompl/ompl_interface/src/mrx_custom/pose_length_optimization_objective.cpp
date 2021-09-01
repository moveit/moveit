/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, MakinaRocks, Inc.
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
 *   * Neither the name of MakinaRocks nor the names of its
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

/* Author: Vinnam Kim */

#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/mrx_custom/pose_length_optimization_objective.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

namespace ompl_interface
{
PoseLengthOptimizationObjective::PoseLengthOptimizationObjective(const ModelBasedPlanningContext* pc,
                                                                 const SpaceInformationPtr& si)
  : PathLengthOptimizationObjective(si)
  , tss_s1_(pc->getCompleteInitialRobotState())
  , tss_s2_(pc->getCompleteInitialRobotState())
  , ee_name_(pc->getJointModelGroup()->getEndEffectorName())
{
  description_ = "Pose Length";
  ROS_INFO("PoseLengthOptimizationObjective!! ee_name: %s", ee_name_.c_str());
}

Cost PoseLengthOptimizationObjective::stateCost(const State*) const
{
  return identityCost();
}

Cost PoseLengthOptimizationObjective::motionCost(const State* s1, const State* s2) const
{
  auto& ss1 = *tss_s1_.getStateStorage();
  auto& ss2 = *tss_s2_.getStateStorage();

  si_->getStateSpace()->as<ModelBasedStateSpace>()->copyToRobotState(ss1, s1);
  si_->getStateSpace()->as<ModelBasedStateSpace>()->copyToRobotState(ss2, s2);

  const auto& ft1 = ss1.getFrameTransform(ee_name_);
  const auto& ft2 = ss2.getFrameTransform(ee_name_);

  return Cost((ft1.translation() - ft2.translation()).norm());
}

Cost PoseLengthOptimizationObjective::motionCostHeuristic(const State* s1, const State* s2) const
{
  return motionCost(s1, s2);
}

InformedSamplerPtr PoseLengthOptimizationObjective::allocInformedStateSampler(const ProblemDefinitionPtr& probDefn,
                                                                              unsigned int maxNumberCalls) const
{
  // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
  // available, if not a rejection-based technique can be used
  return std::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}
}  // namespace ompl_interface
