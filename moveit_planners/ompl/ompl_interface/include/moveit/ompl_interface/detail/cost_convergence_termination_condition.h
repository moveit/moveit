/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, PickNik LLC
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
*   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Henning Kayser */
/* Description: A termination condition for early-stopping OMPL* planners based on cost-convergence. */

#include <ompl/base/PlannerTerminationCondition.h>
namespace ompl_interface
{
namespace ob = ompl::base;
class CostConvergenceTerminationCondition : public ob::PlannerTerminationCondition
{
public:
  CostConvergenceTerminationCondition(size_t solutions_window = 10, double convergence_threshold = 0.9)
    : ob::PlannerTerminationCondition([] { return false; })
    , solutions_window_(solutions_window)
    , convergence_threshold_(convergence_threshold)
  {
  }

  // Compute the running average cost of the last solutions (solution_window) and terminate if the cost of a
  // new solution is higher than convergence_threshold times the average cost.
  void processNewSolution(const std::vector<const ob::State*>& solution_states, const ob::Cost solution_cost)
  {
    ++solutions_;
    size_t solutions = std::min(solutions_, solutions_window_);
    double new_cost = ((solutions - 1) * average_cost_ + solution_cost.value()) / solutions;
    double cost_ratio = new_cost / average_cost_;
    average_cost_ = new_cost;
    if (solutions == solutions_window_ && cost_ratio > convergence_threshold_)
    {
      ROS_DEBUG_NAMED("cost_convergence_termination_condition",
                      "Cost of optimizing planner converged after %lu solutions", solutions_);
      terminate();
    }
  }

private:
  double average_cost_;
  size_t solutions_ = 0;

  const size_t solutions_window_;
  const double convergence_threshold_;
};
}
