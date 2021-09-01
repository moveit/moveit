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
#include <moveit/ompl_interface/mrx_custom/time_length_optimization_objective.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

namespace ompl_interface
{
TimeLengthOptimizationObjective::TimeLengthOptimizationObjective(const ModelBasedPlanningContext* pc,
                                                                 const SpaceInformationPtr& si)
  : PathLengthOptimizationObjective(si)

{
  description_ = "Time Length";
  ROS_INFO("TimeLengthOptimizationObjective!!");

  const auto rm = pc->getCompleteInitialRobotState().getRobotModel();

  const auto jmg = si->getStateSpace()->as<ModelBasedStateSpace>()->getJointModelGroup();
  double vscale = pc->getMotionPlanRequest().max_velocity_scaling_factor;

  for (size_t i = 0; i < jmg->getVariableCount(); i++)
  {
    const auto vn = jmg->getVariableNames()[i];

    const auto& b = rm->getVariableBounds(vn);

    double v_max = std::min(fabs(b.max_velocity_ * vscale), fabs(b.min_velocity_ * vscale));

    if (rm->getJointOfVariable(vn)->isPassive())
      v_max = -1.0;

    v_bounds.push_back(v_max);

    ROS_INFO("Variable name %s has v_max = %f", vn.c_str(), v_max);
  }
}

Cost TimeLengthOptimizationObjective::stateCost(const State*) const
{
  return identityCost();
}

Cost TimeLengthOptimizationObjective::motionCost(const State* s1, const State* s2) const
{
  std::vector<double> vs1(v_bounds.size());
  std::vector<double> vs2(v_bounds.size());

  memcpy(vs1.data(), s1->as<ModelBasedStateSpace::StateType>()->values, sizeof(double) * v_bounds.size());
  memcpy(vs2.data(), s2->as<ModelBasedStateSpace::StateType>()->values, sizeof(double) * v_bounds.size());

  double max_t = 0.0;

  for (size_t i = 0; i < v_bounds.size(); i++)
  {
    double dq = std::abs(vs2[i] - vs1[i]);

    double t = std::max(dq / v_bounds[i], 0.0);

    if (max_t < t)
      max_t = t;
  }

  return Cost(max_t);
}

Cost TimeLengthOptimizationObjective::motionCostHeuristic(const State* s1, const State* s2) const
{
  return motionCost(s1, s2);
}

InformedSamplerPtr TimeLengthOptimizationObjective::allocInformedStateSampler(const ProblemDefinitionPtr& probDefn,
                                                                              unsigned int maxNumberCalls) const
{
  // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
  // available, if not a rejection-based technique can be used
  return std::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}
}  // namespace ompl_interface
