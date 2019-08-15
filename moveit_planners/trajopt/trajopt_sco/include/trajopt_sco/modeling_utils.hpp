#pragma once
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/num_diff.hpp>
#include <trajopt_sco/sco_common.hpp>
/**
@file modeling_utils.hpp
@brief Build problem from user-defined functions
Utilities for creating Cost and Constraint objects from functions
using numerical derivatives or user-defined analytic derivatives.
 */

namespace sco
{
enum PenaltyType
{
  SQUARED,
  ABS,
  HINGE
};

/**
x is the big solution vector of the whole problem. vars are variables that
index into the vector x
this function extracts (from x) the values of the variables in vars
 */
Eigen::VectorXd getVec(const DblVec& x, const VarVector& vars);
/**
Same idea as above, but different output type
 */
DblVec getDblVec(const DblVec& x, const VarVector& vars);

AffExpr affFromValGrad(double y, const Eigen::VectorXd& x, const Eigen::VectorXd& dydx, const VarVector& vars);

class CostFromFunc : public Cost
{
public:
  /// supply function, obtain derivative and hessian numerically
  CostFromFunc(ScalarOfVectorPtr f, const VarVector& vars, const std::string& name, bool full_hessian = false);
  double value(const DblVec& x) override;
  ConvexObjectivePtr convex(const DblVec& x, Model* model) override;
  VarVector getVars() override { return vars_; }
protected:
  ScalarOfVectorPtr f_;
  VarVector vars_;
  bool full_hessian_;
  double epsilon_;
};

class CostFromErrFunc : public Cost
{
public:
  /// supply error function, obtain derivative numerically
  CostFromErrFunc(VectorOfVectorPtr f,
                  const VarVector& vars,
                  const Eigen::VectorXd& coeffs,
                  PenaltyType pen_type,
                  const std::string& name);
  /// supply error function and gradient
  CostFromErrFunc(VectorOfVectorPtr f,
                  MatrixOfVectorPtr dfdx,
                  const VarVector& vars,
                  const Eigen::VectorXd& coeffs,
                  PenaltyType pen_type,
                  const std::string& name);
  double value(const DblVec& x) override;
  ConvexObjectivePtr convex(const DblVec& x, Model* model) override;
  VarVector getVars() override { return vars_; }
protected:
  VectorOfVectorPtr f_;
  MatrixOfVectorPtr dfdx_;
  VarVector vars_;
  Eigen::VectorXd coeffs_;
  PenaltyType pen_type_;
  double epsilon_;
};

class ConstraintFromErrFunc : public Constraint
{
public:
  /// supply error function, obtain derivative numerically
  ConstraintFromErrFunc(VectorOfVectorPtr f,
                        const VarVector& vars,
                        const Eigen::VectorXd& coeffs,
                        ConstraintType type,
                        const std::string& name);
  /// supply error function and gradient
  ConstraintFromErrFunc(VectorOfVectorPtr f,
                        MatrixOfVectorPtr dfdx,
                        const VarVector& vars,
                        const Eigen::VectorXd& coeffs,
                        ConstraintType type,
                        const std::string& name);
  DblVec value(const DblVec& x) override;
  ConvexConstraintsPtr convex(const DblVec& x, Model* model) override;
  ConstraintType type() override { return type_; }
  VarVector getVars() override { return vars_; }
protected:
  VectorOfVectorPtr f_;
  MatrixOfVectorPtr dfdx_;
  VarVector vars_;
  Eigen::VectorXd coeffs_;
  ConstraintType type_;
  double epsilon_;
  Eigen::VectorXd scaling_;
};

std::string AffExprToString(const AffExpr& aff);
}
