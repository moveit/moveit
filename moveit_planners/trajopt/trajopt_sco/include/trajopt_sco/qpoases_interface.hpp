#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <qpOASES.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
/**
 * qpOASESModel uses the LGPL solver qpOASES to solve a linearly constrained QP.
 * qpOASES solves a problem in the form:
 * ```
 * min   1/2*x'Hx + x'g
 * s.t.  lb  <=  x <= ub
 * lbA <= Ax <= ubA
 * ```
 *
 * More informations about the solver are available at:
 * https://projects.coin-or.org/qpOASES
 */
class qpOASESModel : public Model
{
  /** pointer to a qpOASES Sequential Quadratic Problem*/
  std::shared_ptr<qpOASES::SQProblem> qpoases_problem_;
  qpOASES::Options qpoases_options_; /**< qpOASES solver options */

  qpOASES::SymSparseMat H_; /**< Quadratic cost matrix */
  qpOASES::SparseMatrix A_; /**< Constraints matrix */

  /** Updates qpOASES Hessian matrix from QuadExpr expression.
   *  Transforms QuadExpr objective_ into the qpOASES sparse matrix H_*/
  void updateObjective();

  /** Updates qpOASES constraints from AffExpr expression.
   *  Transforms AffExpr cntr_exprs_ into the qpOASES sparse matrix A_, and
   *  vectors lbA_ and ubA_ */
  void updateConstraints();

  /**
   * Instantiates a new qpOASES problem if it has not been instantiated yet
   * or if the size of the problem has changed.
   *
   * @returns true if a new qpOASES problem has been instantiated
   */
  bool updateSolver();

  /**
   * Instantiates a new qpOASES problem
   */
  void createSolver();

  VarVector vars_;                 /**< model variables */
  CntVector cnts_;                 /**< model's constraints sizes */
  DblVec lb_, ub_;                 /**< variables bounds */
  AffExprVector cnt_exprs_;        /**< constraints expressions */
  ConstraintTypeVector cnt_types_; /**< constraints types */
  DblVec solution_;                /**< optimizizer's solution for current model */

  IntVec H_row_indices_;     /**< row indices for Hessian, CSC format */
  IntVec H_column_pointers_; /**< column pointers for Hessian, CSC format */
  DblVec H_csc_data_;        /**< Hessian values in CSC format */
  Eigen::VectorXd g_;        /**< gradient of the optimization problem */

  IntVec A_row_indices_;     /**< row indices for constraint matrix, CSC format */
  IntVec A_column_pointers_; /**< column pointers for constraint matrix, CSC format */
  DblVec A_csc_data_;        /**< constraint matrix values in CSC format */
  DblVec lbA_, ubA_;         /**< linear constraints upper and lower limits */

  QuadExpr objective_; /**< objective QuadExpr expression */

public:
  qpOASESModel();
  virtual ~qpOASESModel();

  Var addVar(const std::string& name) override;
  Cnt addEqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const QuadExpr&, const std::string& name) override;
  void removeVars(const VarVector& vars) override;
  void removeCnts(const CntVector& cnts) override;

  void update() override;
  void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper) override;
  DblVec getVarValues(const VarVector& vars) const override;
  virtual CvxOptStatus optimize() override;
  virtual void setObjective(const AffExpr&) override;
  virtual void setObjective(const QuadExpr&) override;
  virtual void writeToFile(const std::string& fname) override;
  virtual VarVector getVars() const override;
};
}
