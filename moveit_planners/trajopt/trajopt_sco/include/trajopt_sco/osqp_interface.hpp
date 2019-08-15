#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <osqp.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
/**
 * OSQPModel uses the BSD solver OSQP to solve a linearly constrained QP.
 * OSQP solves a problem in the form:
 * ```
 * min   1/2*x'Px + q'x
 * s.t.  l <= Ax <= u
 * ```
 *
 * More informations about the solver are available at:
 * https://osqp.org/docs/
 */
class OSQPModel : public Model
{
  OSQPSettings osqp_settings_; /**< OSQP Settings */
                               /** OSQPData. Some fields here (`osp_data_.A` and `osp_data_.P`) are
                                *  automatically allocated by OSQP, but deallocated by us. */
  OSQPData osqp_data_;
  /** OSQP Workspace. Memory here is managed by OSQP */
  OSQPWorkspace* osqp_workspace_;

  /** Updates OSQP quadratic cost matrix from QuadExpr expression.
   *  Transforms QuadExpr objective_ into the OSQP CSC matrix P_ */
  void updateObjective();

  /** Updates qpOASES constraints from AffExpr expression.
   *  Transforms AffExpr cntr_exprs_ and box bounds lbs_ and ubs_ into the
   *  OSQP CSC matrix A_, and vectors lbA_ and ubA_ */
  void updateConstraints();

  /** Creates or updates the solver and its workspace */
  void createOrUpdateSolver();

  VarVector vars_;                 /**< model variables */
  CntVector cnts_;                 /**< model's constraints sizes */
  DblVec lbs_, ubs_;               /**< variables bounds */
  AffExprVector cnt_exprs_;        /**< constraints expressions */
  ConstraintTypeVector cnt_types_; /**< constraints types */
  DblVec solution_;                /**< optimizizer's solution for current model */

  std::vector<c_int> P_row_indices_;     /**< row indices for P, CSC format */
  std::vector<c_int> P_column_pointers_; /**< column pointers for P, CSC format */
  DblVec P_csc_data_;                    /**< P values in CSC format */
  Eigen::VectorXd q_;                    /**< linear part of the objective */

  std::vector<c_int> A_row_indices_;     /**< row indices for constraint matrix, CSC format */
  std::vector<c_int> A_column_pointers_; /**< column pointers for constraint matrix, CSC format */
  DblVec A_csc_data_;                    /**< constraint matrix values in CSC format */
  DblVec l_, u_;                         /**< linear constraints upper and lower limits */

  QuadExpr objective_; /**< objective QuadExpr expression */

public:
  OSQPModel();
  virtual ~OSQPModel();

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
