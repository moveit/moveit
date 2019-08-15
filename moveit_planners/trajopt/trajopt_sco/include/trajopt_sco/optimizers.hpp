#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <functional>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/modeling.hpp>
/*
 * Algorithms for non-convex, constrained optimization
 */

namespace sco
{
enum OptStatus
{
  OPT_CONVERGED,
  OPT_SCO_ITERATION_LIMIT,  // hit iteration limit before convergence
  OPT_PENALTY_ITERATION_LIMIT,
  OPT_FAILED,
  INVALID
};
static const char* OptStatus_strings[] = { "CONVERGED",
                                           "SCO_ITERATION_LIMIT",
                                           "PENALTY_ITERATION_LIMIT",
                                           "FAILED",
                                           "INVALID" };
inline std::string statusToString(OptStatus status) { return OptStatus_strings[status]; }
struct OptResults
{
  DblVec x;  // solution estimate
  OptStatus status;
  double total_cost;
  DblVec cost_vals;
  DblVec cnt_viols;
  int n_func_evals, n_qp_solves;
  void clear()
  {
    x.clear();
    status = INVALID;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
  }
  OptResults() { clear(); }
};
std::ostream& operator<<(std::ostream& o, const OptResults& r);

class Optimizer
{
  /*
   * Solves an optimization problem
   */
public:
  virtual ~Optimizer() = default;
  virtual OptStatus optimize() = 0;
  virtual void setProblem(OptProbPtr prob) { prob_ = prob; }
  void initialize(const DblVec& x);
  DblVec& x() { return results_.x; }
  OptResults& results() { return results_; }
  typedef std::function<void(OptProb*, OptResults&)> Callback;
  void addCallback(const Callback& f);  // called before each iteration
protected:
  std::vector<Callback> callbacks_;
  void callCallbacks();
  OptProbPtr prob_;
  OptResults results_;
};

struct BasicTrustRegionSQPParameters
{
  double improve_ratio_threshold;  // minimum ratio true_improve/approx_improve
                                   // to accept step
  double min_trust_box_size;       // if trust region gets any smaller, exit and
                                   // report convergence
  double min_approx_improve;       // if model improves less than this, exit and
                                   // report convergence
  double min_approx_improve_frac;  // if model improves less than this, exit and
                                   // report convergence
  double max_iter;                 // The max number of iterations
  double trust_shrink_ratio;       // if improvement is less than
  // improve_ratio_threshold, shrink trust region by
  // this ratio
  double trust_expand_ratio;  // if improvement is less than
                              // improve_ratio_threshold, shrink trust region by
                              // this ratio
  double cnt_tolerance;       // after convergence of penalty subproblem, if
  // constraint violation is less than this, we're done
  double max_merit_coeff_increases;   // number of times that we jack up penalty
                                      // coefficient
  double merit_coeff_increase_ratio;  // ratio that we increate coeff each time
  double max_time;                    // not yet implemented
  double merit_error_coeff;           // initial penalty coefficient
  double trust_box_size;              // current size of trust region (component-wise)

  BasicTrustRegionSQPParameters();
};

class BasicTrustRegionSQP : public Optimizer
{
  /*
   * Alternates between convexifying objectives and constraints and then solving
   * convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an
   * infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible
   * constraints)
   */
public:
  BasicTrustRegionSQP();
  BasicTrustRegionSQP(OptProbPtr prob);
  void setProblem(OptProbPtr prob) override;
  void setParameters(const BasicTrustRegionSQPParameters& param) { param_ = param; }
  const BasicTrustRegionSQPParameters& getParameters() const { return param_; }
  BasicTrustRegionSQPParameters& getParameters() { return param_; }
  OptStatus optimize() override;

protected:
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const DblVec& x);
  ModelPtr model_;
  BasicTrustRegionSQPParameters param_;
};
}
