#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <constants.h>
#include <cmath>
#include <Eigen/SparseCore>
#include <fstream>
#include <signal.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace sco
{
double OSQP_INFINITY = std::numeric_limits<double>::infinity();

ModelPtr createOSQPModel()
{
  ModelPtr out(new OSQPModel());
  return out;
}

OSQPModel::OSQPModel()
{
  // Define Solver settings as default
  // see https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
  osqp_set_default_settings(&osqp_settings_);
  // tuning parameters to be less accurate, but add a polishing step
  osqp_settings_.eps_abs = 1e-4;
  osqp_settings_.eps_rel = 1e-6;
  osqp_settings_.max_iter = 8192;
  osqp_settings_.polish = 1;
  osqp_settings_.verbose = false;

  // Initialize data
  osqp_data_.A = nullptr;
  osqp_data_.P = nullptr;
  osqp_workspace_ = nullptr;
}

OSQPModel::~OSQPModel()
{
  // Cleanup
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);
  if (osqp_data_.A != nullptr)
    c_free(osqp_data_.A);
  if (osqp_data_.P != nullptr)
    c_free(osqp_data_.P);
}

Var OSQPModel::addVar(const std::string& name)
{
  vars_.push_back(new VarRep(vars_.size(), name, this));
  lbs_.push_back(-OSQP_INFINITY);
  ubs_.push_back(OSQP_INFINITY);
  return vars_.back();
}

Cnt OSQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/)
{
  throw std::runtime_error("NOT IMPLEMENTED");
  return Cnt();
}

void OSQPModel::removeVars(const VarVector& vars)
{
  IntVec inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void OSQPModel::removeCnts(const CntVector& cnts)
{
  IntVec inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void OSQPModel::updateObjective()
{
  const size_t n = vars_.size();
  osqp_data_.n = n;

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, q_, n, true);
  eigenToCSC(sm, P_row_indices_, P_column_pointers_, P_csc_data_);

  if (osqp_data_.P != nullptr)
    c_free(osqp_data_.P);
  osqp_data_.P = csc_matrix(osqp_data_.n,
                            osqp_data_.n,
                            P_csc_data_.size(),
                            P_csc_data_.data(),
                            P_row_indices_.data(),
                            P_column_pointers_.data());

  osqp_data_.q = q_.data();
}

void OSQPModel::updateConstraints()
{
  const size_t n = vars_.size();
  const size_t m = cnts_.size();
  osqp_data_.m = m + n;

  Eigen::SparseMatrix<double> sm;
  Eigen::VectorXd v;
  exprToEigen(cnt_exprs_, sm, v, n);
  Eigen::SparseMatrix<double> sm_e(m + n, n);
  Eigen::SparseMatrix<double> sm_e2 = sm;
  sm.conservativeResize(m + n, Eigen::NoChange_t(n));

  l_.clear();
  l_.resize(m + n, -OSQP_INFINITY);
  u_.clear();
  u_.resize(m + n, OSQP_INFINITY);

  for (int i_cnt = 0; i_cnt < m; ++i_cnt)
  {
    l_[i_cnt] = (cnt_types_[i_cnt] == INEQ) ? -OSQP_INFINITY : v[i_cnt];
    u_[i_cnt] = v[i_cnt];
  }

  for (int i_bnd = 0; i_bnd < n; ++i_bnd)
  {
    l_[i_bnd + m] = fmax(lbs_[i_bnd], -OSQP_INFINITY);
    u_[i_bnd + m] = fmin(ubs_[i_bnd], OSQP_INFINITY);
    sm.insert(i_bnd + m, i_bnd) = 1.;
  }

  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);

  if (osqp_data_.A != nullptr)
    c_free(osqp_data_.A);
  osqp_data_.A = csc_matrix(osqp_data_.m,
                            osqp_data_.n,
                            A_csc_data_.size(),
                            A_csc_data_.data(),
                            A_row_indices_.data(),
                            A_column_pointers_.data());

  osqp_data_.l = l_.data();
  osqp_data_.u = u_.data();
}

void OSQPModel::createOrUpdateSolver()
{
  updateObjective();
  updateConstraints();

  // TODO atm we are not updating the workspace, but recreating it each time.
  // In the future, we will checking sparsity did not change and update instead
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);
  // Setup workspace - this should be called only once
  osqp_workspace_ = osqp_setup(&osqp_data_, &osqp_settings_);
}

void OSQPModel::update()
{
  {
    int inew = 0;
    for (unsigned iold = 0; iold < vars_.size(); ++iold)
    {
      const Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lbs_[inew] = lbs_[iold];
        ubs_[inew] = ubs_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    vars_.resize(inew);
    lbs_.resize(inew);
    ubs_.resize(inew);
  }
  {
    int inew = 0;
    for (unsigned iold = 0; iold < cnts_.size(); ++iold)
    {
      const Cnt& cnt = cnts_[iold];
      if (!cnt.cnt_rep->removed)
      {
        cnts_[inew] = cnt;
        cnt_exprs_[inew] = cnt_exprs_[iold];
        cnt_types_[inew] = cnt_types_[iold];
        cnt.cnt_rep->index = inew;
        ++inew;
      }
      else
        delete cnt.cnt_rep;
    }
    cnts_.resize(inew);
    cnt_exprs_.resize(inew);
    cnt_types_.resize(inew);
  }
}

void OSQPModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const int varind = vars[i].var_rep->index;
    lbs_[varind] = lower[i];
    ubs_[varind] = upper[i];
  }
}
DblVec OSQPModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const int varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus OSQPModel::optimize()
{
  update();
  createOrUpdateSolver();

  // Solve Problem
  const int retcode = osqp_solve(osqp_workspace_);

  if (retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    solution_ = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + vars_.size());
    int status = osqp_workspace_->info->status_val;
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    else if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
             status == OSQP_DUAL_INFEASIBLE || status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }
  return CVX_FAILED;
}
void OSQPModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void OSQPModel::setObjective(const QuadExpr& expr) { objective_ = expr; }
void OSQPModel::writeToFile(const std::string& /*fname*/)
{
  return;  // NOT IMPLEMENTED
}
VarVector OSQPModel::getVars() const { return vars_; }
}
