#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigenvalues>
#include <iostream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

namespace sco
{
const double DEFAULT_EPSILON = 1e-5;

Eigen::VectorXd getVec(const DblVec& x, const VarVector& vars)
{
  Eigen::VectorXd out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
    out[i] = x[static_cast<long unsigned int>(vars[i].var_rep->index)];
  return out;
}

DblVec getDblVec(const DblVec& x, const VarVector& vars)
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
    out[i] = x[static_cast<long unsigned int>(vars[i].var_rep->index)];
  return out;
}

AffExpr affFromValGrad(double y, const Eigen::VectorXd& x, const Eigen::VectorXd& dydx, const VarVector& vars)
{
  AffExpr aff;
  aff.constant = y - dydx.dot(x);
  aff.coeffs = util::toDblVec(dydx);
  aff.vars = vars;
  aff = cleanupAff(aff);
  return aff;
}

CostFromFunc::CostFromFunc(ScalarOfVectorPtr f, const VarVector& vars, const std::string& name, bool full_hessian)
  : Cost(name), f_(f), vars_(vars), full_hessian_(full_hessian), epsilon_(DEFAULT_EPSILON)
{
}

double CostFromFunc::value(const DblVec& xin)
{
  Eigen::VectorXd x = getVec(xin, vars_);
  return f_->call(x);
}

ConvexObjectivePtr CostFromFunc::convex(const DblVec& xin, Model* model)
{
  Eigen::VectorXd x = getVec(xin, vars_);

  ConvexObjectivePtr out(new ConvexObjective(model));
  if (!full_hessian_)
  {
    double val;
    Eigen::VectorXd grad, hess;
    calcGradAndDiagHess(*f_, x, epsilon_, val, grad, hess);
    hess = hess.cwiseMax(Eigen::VectorXd::Zero(hess.size()));
    QuadExpr& quad = out->quad_;
    quad.affexpr.constant = val - grad.dot(x) + .5 * x.dot(hess.cwiseProduct(x));
    quad.affexpr.vars = vars_;
    quad.affexpr.coeffs = util::toDblVec(grad - hess.cwiseProduct(x));
    quad.vars1 = vars_;
    quad.vars2 = vars_;
    quad.coeffs = util::toDblVec(hess * .5);
  }
  else
  {
    double val;
    Eigen::VectorXd grad;
    Eigen::MatrixXd hess;
    calcGradHess(f_, x, epsilon_, val, grad, hess);

    Eigen::MatrixXd pos_hess = Eigen::MatrixXd::Zero(x.size(), x.size());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(hess);
    Eigen::VectorXd eigvals = es.eigenvalues();
    Eigen::MatrixXd eigvecs = es.eigenvectors();
    for (long int i = 0, end = x.size(); i != end; ++i)
    {  // tricky --- eigen size() is signed
      if (eigvals(i) > 0)
        pos_hess += eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose();
    }

    QuadExpr& quad = out->quad_;
    quad.affexpr.constant = val - grad.dot(x) + .5 * x.dot(pos_hess * x);
    quad.affexpr.vars = vars_;
    quad.affexpr.coeffs = util::toDblVec(grad - pos_hess * x);

    size_t nquadterms = static_cast<size_t>((x.size() * (x.size() - 1)) / 2);
    quad.coeffs.reserve(nquadterms);
    quad.vars1.reserve(nquadterms);
    quad.vars2.reserve(nquadterms);
    for (long int i = 0, end = x.size(); i != end; ++i)
    {  // tricky --- eigen size() is signed
      quad.vars1.push_back(vars_[static_cast<size_t>(i)]);
      quad.vars2.push_back(vars_[static_cast<size_t>(i)]);
      quad.coeffs.push_back(pos_hess(i, i) / 2);
      for (long int j = i + 1; j != end; ++j)
      {  // tricky --- eigen size() is signed
        quad.vars1.push_back(vars_[static_cast<size_t>(i)]);
        quad.vars2.push_back(vars_[static_cast<size_t>(j)]);
        quad.coeffs.push_back(pos_hess(i, j));
      }
    }
  }

  return out;
}

CostFromErrFunc::CostFromErrFunc(VectorOfVectorPtr f,
                                 const VarVector& vars,
                                 const Eigen::VectorXd& coeffs,
                                 PenaltyType pen_type,
                                 const std::string& name)
  : Cost(name), f_(f), vars_(vars), coeffs_(coeffs), pen_type_(pen_type), epsilon_(DEFAULT_EPSILON)
{
}
CostFromErrFunc::CostFromErrFunc(VectorOfVectorPtr f,
                                 MatrixOfVectorPtr dfdx,
                                 const VarVector& vars,
                                 const Eigen::VectorXd& coeffs,
                                 PenaltyType pen_type,
                                 const std::string& name)
  : Cost(name), f_(f), dfdx_(dfdx), vars_(vars), coeffs_(coeffs), pen_type_(pen_type), epsilon_(DEFAULT_EPSILON)
{
}
double CostFromErrFunc::value(const DblVec& xin)
{
  Eigen::VectorXd x = getVec(xin, vars_);
  Eigen::VectorXd err = f_->call(x);

  switch (pen_type_)
  {
    case SQUARED:
      err = err.array().square();
      break;
    case ABS:
      err = err.array().abs();
      break;
    case HINGE:
      err = err.cwiseMax(Eigen::VectorXd::Zero(err.size()));
      break;
    default:
      assert(0 && "unreachable");
  }

  if (coeffs_.size() > 0)
    err.array() *= coeffs_.array();

  return err.array().sum();
}
ConvexObjectivePtr CostFromErrFunc::convex(const DblVec& xin, Model* model)
{
  Eigen::VectorXd x = getVec(xin, vars_);
  Eigen::MatrixXd jac = (dfdx_) ? dfdx_->call(x) : calcForwardNumJac(*f_, x, epsilon_);
  ConvexObjectivePtr out(new ConvexObjective(model));
  Eigen::VectorXd y = f_->call(x);
  for (int i = 0; i < jac.rows(); ++i)
  {
    AffExpr aff = affFromValGrad(y[i], x, jac.row(i), vars_);
    double weight = 1;
    if (coeffs_.size() > 0)
    {
      if (coeffs_[i] == 0)
        continue;

      weight = coeffs_[i];
    }
    switch (pen_type_)
    {
      case SQUARED:
      {
        QuadExpr quad = exprSquare(aff);
        exprScale(quad, weight);
        out->addQuadExpr(quad);
        break;
      }
      case ABS:
      {
        exprScale(aff, weight);
        out->addAbs(aff, 1);
        break;
      }
      case HINGE:
      {
        exprScale(aff, weight);
        out->addHinge(aff, 1);
        break;
      }
      default:
        assert(0 && "unreachable");
    }
  }
  return out;
}

ConstraintFromErrFunc::ConstraintFromErrFunc(VectorOfVectorPtr f,
                                             const VarVector& vars,
                                             const Eigen::VectorXd& coeffs,
                                             ConstraintType type,
                                             const std::string& name)
  : Constraint(name), f_(f), vars_(vars), coeffs_(coeffs), type_(type), epsilon_(DEFAULT_EPSILON)
{
}

ConstraintFromErrFunc::ConstraintFromErrFunc(VectorOfVectorPtr f,
                                             MatrixOfVectorPtr dfdx,
                                             const VarVector& vars,
                                             const Eigen::VectorXd& coeffs,
                                             ConstraintType type,
                                             const std::string& name)
  : Constraint(name), f_(f), dfdx_(dfdx), vars_(vars), coeffs_(coeffs), type_(type), epsilon_(DEFAULT_EPSILON)
{
}

DblVec ConstraintFromErrFunc::value(const DblVec& xin)
{
  Eigen::VectorXd x = getVec(xin, vars_);
  Eigen::VectorXd err = f_->call(x);
  if (coeffs_.size() > 0)
    err.array() *= coeffs_.array();
  return util::toDblVec(err);
}

ConvexConstraintsPtr ConstraintFromErrFunc::convex(const DblVec& xin, Model* model)
{
  Eigen::VectorXd x = getVec(xin, vars_);
  Eigen::MatrixXd jac = (dfdx_) ? dfdx_->call(x) : calcForwardNumJac(*f_, x, epsilon_);
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  Eigen::VectorXd y = f_->call(x);
  for (int i = 0; i < jac.rows(); ++i)
  {
    AffExpr aff = affFromValGrad(y[i], x, jac.row(i), vars_);
    if (coeffs_.size() > 0)
    {
      if (coeffs_[i] == 0)
        continue;
      exprScale(aff, coeffs_[i]);
    }
    if (type() == INEQ)
      out->addIneqCnt(aff);
    else
      out->addEqCnt(aff);
  }
  return out;
}

std::string AffExprToString(const AffExpr& aff)
{
  std::string out;
  for (size_t i = 0; i < aff.vars.size(); i++)
  {
    if (i != 0)
      out.append(" + ");
    std::string term = std::to_string(aff.coeffs[i]) + "*" + aff.vars[i].var_rep->name;
    out.append(term);
  }
  out.append(" + " + std::to_string(aff.constant));
  return out;
}
}
