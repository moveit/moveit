#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <Eigen/Eigen>
#include <fstream>
#include <signal.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/qpoases_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace qpOASES;

namespace sco
{
double QPOASES_INFTY = qpOASES::INFTY;

ModelPtr createqpOASESModel()
{
  ModelPtr out(new qpOASESModel());
  return out;
}

qpOASESModel::qpOASESModel()
{
  // set to be fast. More details at:
  // https://www.coin-or.org/qpOASES/doc/3.2/doxygen/classOptions.html
  // https://projects.coin-or.org/qpOASES/browser/stable/3.2/src/Options.cpp#L191
  qpoases_options_.setToMPC();
  qpoases_options_.printLevel = qpOASES::PL_NONE;
  // enable regularisation to deal with degenerate Hessians
  qpoases_options_.enableRegularisation = qpOASES::BT_TRUE;
  qpoases_options_.ensureConsistency();
}

qpOASESModel::~qpOASESModel() {}
Var qpOASESModel::addVar(const std::string& name)
{
  vars_.push_back(new VarRep(vars_.size(), name, this));
  lb_.push_back(-QPOASES_INFTY);
  ub_.push_back(QPOASES_INFTY);
  return vars_.back();
}

Cnt qpOASESModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt qpOASESModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt qpOASESModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return 0;
}

void qpOASESModel::removeVars(const VarVector& vars)
{
  IntVec inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void qpOASESModel::removeCnts(const CntVector& cnts)
{
  IntVec inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void qpOASESModel::updateObjective()
{
  const size_t n = vars_.size();

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, g_, n, true, true);
  eigenToCSC(sm, H_row_indices_, H_column_pointers_, H_csc_data_);

  H_ = SymSparseMat(vars_.size(), vars_.size(), H_row_indices_.data(), H_column_pointers_.data(), H_csc_data_.data());
  H_.createDiagInfo();
}

void qpOASESModel::updateConstraints()
{
  const size_t n = vars_.size();
  const size_t m = cnts_.size();

  lbA_.clear();
  lbA_.resize(m, -QPOASES_INFTY);
  ubA_.clear();
  ubA_.resize(m, QPOASES_INFTY);

  Eigen::SparseMatrix<double> sm;
  Eigen::VectorXd v;
  exprToEigen(cnt_exprs_, sm, v, n);

  for (int i_cnt = 0; i_cnt < m; ++i_cnt)
  {
    lbA_[i_cnt] = (cnt_types_[i_cnt] == INEQ) ? -QPOASES_INFTY : v[i_cnt];
    ubA_[i_cnt] = v[i_cnt];
  }

  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);
  A_ = SparseMatrix(cnts_.size(), vars_.size(), A_row_indices_.data(), A_column_pointers_.data(), A_csc_data_.data());
}

bool qpOASESModel::updateSolver()
{
  bool solver_updated = false;
  if (!qpoases_problem_ || vars_.size() != qpoases_problem_->getNV() || cnts_.size() != qpoases_problem_->getNC())
  {
    // Create Problem - this should be called only once
    qpoases_problem_.reset(new SQProblem(vars_.size(), cnts_.size()));
    qpoases_problem_->setOptions(qpoases_options_);
    solver_updated = true;
  }
  return solver_updated;
}

void qpOASESModel::createSolver()
{
  qpoases_problem_.reset();
  updateSolver();
}

void qpOASESModel::update()
{
  {
    int inew = 0;
    for (unsigned iold = 0; iold < vars_.size(); ++iold)
    {
      const Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lb_[inew] = lb_[iold];
        ub_[inew] = ub_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    vars_.resize(inew);
    lb_.resize(inew, QPOASES_INFTY);
    ub_.resize(inew, -QPOASES_INFTY);
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

void qpOASESModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const int varind = vars[i].var_rep->index;
    lb_[varind] = lower[i];
    ub_[varind] = upper[i];
  }
}
DblVec qpOASESModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const int varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus qpOASESModel::optimize()
{
  update();
  updateObjective();
  updateConstraints();
  updateSolver();
  qpOASES::returnValue val = qpOASES::RET_QP_SOLUTION_STARTED;

  // Solve Problem
  int nWSR = 255;
  if (qpoases_problem_->isInitialised())
  {
    val = qpoases_problem_->hotstart(
        &H_, g_.data(), &A_, lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), nWSR, nullptr);
  }

  if (val != qpOASES::SUCCESSFUL_RETURN)
  {
    // TODO ATM this means we are creating a new solver even if updateSolver
    //      returned true and the problem is not initialized. Still, it makes
    //      tests pass.
    createSolver();

    val = qpoases_problem_->init(&H_, g_.data(), &A_, lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), nWSR, nullptr);
  }

  if (val == qpOASES::SUCCESSFUL_RETURN)
  {
    // opt += m_objective.affexpr.constant;
    solution_.resize(vars_.size(), 0.);
    val = qpoases_problem_->getPrimalSolution(solution_.data());
    return CVX_SOLVED;
  }
  else if (val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
  {
    return CVX_INFEASIBLE;
  }
  else
  {
    return CVX_FAILED;
  }
}
void qpOASESModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void qpOASESModel::setObjective(const QuadExpr& expr) { objective_ = expr; }
void qpOASESModel::writeToFile(const std::string& /*fname*/)
{
  return;  // NOT IMPLEMENTED
}
VarVector qpOASESModel::getVars() const { return vars_; }
}
