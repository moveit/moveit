#include <iostream>
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
    std::cout << " test" << std::endl;
  }

  // Compute the running average cost of the last solutions (solution_window) and terminate if the cost of a
  // new solution is higher than convergence_threshold times the average cost.
  void processNewSolution(const std::vector<const ob::State*>& solution_states, const ob::Cost solution_cost)
  {
    ++solutions_;
    size_t solutions = std::min(solutions_, solutions_window_);
    double new_cost = ((solutions - 1) * average_cost_ + solution_cost.value()) / solutions;
    double cost_ratio = average_cost_ / new_cost;
    average_cost_ = new_cost;
    if (solutions == solutions_window_ && cost_ratio > convergence_threshold_)
    {
      std::cout << "Optimizing planner converged after " << solutions_ << " solutions" << std::endl;
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
