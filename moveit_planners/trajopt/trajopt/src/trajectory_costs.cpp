#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

#include "trajopt/trajectory_costs.hpp"

namespace
{
/** @brief Returns the difference between each row of a matrixXd and the row before */
static Eigen::MatrixXd diffAxis0(const Eigen::MatrixXd& in)
{
  return in.middleRows(1, in.rows() - 1) - in.middleRows(0, in.rows() - 1);
}
}  // namespace

namespace trajopt
{
//////////// Joint cost functions /////////////////

//////////////////// Position /////////////////////
JointPosEqCost::JointPosEqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                               int& first_step, int& last_step)
  : Cost("JointPosEq"), vars_(vars), coeffs_(coeffs), targets_(targets), first_step_(first_step), last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // pos = x1 - targ
      sco::AffExpr pos;
      sco::exprInc(pos, sco::exprMult(vars(i, j), 1));
      sco::exprDec(pos, targets_[j]);
      // expr_ = coeff * vel^2
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(pos), coeffs_[j]));
    }
  }
}
double JointPosEqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = (getTraj(xvec, vars_));
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())).rowwise() - targets_.transpose();
  // Element-wise square it, multiply it by a diagonal matrix of coefficients, and sums output
  return (diff.array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointPosEqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointPosIneqCost::JointPosIneqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                                   const Eigen::VectorXd& upper_tols, const Eigen::VectorXd& lower_tols,
                                   int& first_step, int& last_step)
  : Cost("JointPosIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      // pos = x1 - targ
      sco::AffExpr pos;
      sco::exprInc(pos, sco::exprMult(vars(i, j), 1));
      sco::exprDec(pos, targets_[j]);
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprDec(expr, pos);           // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);  // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprDec(expr_neg, pos);          // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);  // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

double JointPosIneqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd pos = traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols());
  // Subtract targets to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (pos.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  return diff1.cwiseMax(0).sum() + diff2.cwiseMax(0).sum();
}

sco::ConvexObjectivePtr JointPosIneqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addHinge(expr, 1);
  }
  return out;
}

JointPosEqConstraint::JointPosEqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                           const Eigen::VectorXd& targets, int& first_step, int& last_step)
  : EqConstraint("JointPosEq")
  , vars_(vars)
  , coeffs_(coeffs)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // pos = x1 - targ
      sco::AffExpr pos;
      sco::exprInc(pos, sco::exprMult(vars(i, j), 1));
      sco::exprDec(pos, targets_[j]);
      // expr_ = coeff * vel - Not squared b/c QuadExpr cnt not yet supported (TODO)
      expr_vec_.push_back(sco::exprMult(pos, coeffs_[j]));
    }
  }
}

DblVec JointPosEqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())).rowwise() - targets_.transpose();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return util::toDblVec((diff.array().square()).matrix() * coeffs_.asDiagonal());
}
sco::ConvexConstraintsPtr JointPosEqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addEqCnt(expr);
  }
  return out;
}

JointPosIneqConstraint::JointPosIneqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                               const Eigen::VectorXd& targets, const Eigen::VectorXd& upper_tols,
                                               const Eigen::VectorXd& lower_tols, int& first_step, int& last_step)
  : IneqConstraint("JointPosIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      // pos = x1 - targ
      sco::AffExpr pos;
      sco::exprInc(pos, sco::exprMult(vars(i, j), 1));
      sco::exprDec(pos, targets_[j]);
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprDec(expr, pos);           // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);  // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form lower limit expr = (upper_tol-(vel-targ))
      sco::exprDec(expr_neg, pos);          // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);  // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

DblVec JointPosIneqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd pos = diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()));
  // Subtract targets to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (pos.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  Eigen::MatrixXd out(diff1.rows(), diff1.cols() + diff2.cols());
  out << diff1, diff2;
  return util::toDblVec(out);
}

sco::ConvexConstraintsPtr JointPosIneqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addIneqCnt(expr);
  }
  return out;
}

//////////////////// Velocity /////////////////////
JointVelEqCost::JointVelEqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                               int& first_step, int& last_step)
  : Cost("JointVelEq"), vars_(vars), coeffs_(coeffs), targets_(targets), first_step_(first_step), last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      sco::AffExpr vel;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      exprDec(vel, targets_[j]);
      // expr_ = coeff * vel^2
      exprInc(expr_, exprMult(exprSquare(vel), coeffs_[j]));
    }
  }
}
double JointVelEqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = (getTraj(xvec, vars_));
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff = (diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()))).rowwise() -
                         targets_.transpose();
  // Element-wise square it, multiply it by a diagonal matrix of coefficients, and sums output
  return (diff.array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointVelEqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointVelIneqCost::JointVelIneqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                                   const Eigen::VectorXd& upper_tols, const Eigen::VectorXd& lower_tols,
                                   int& first_step, int& last_step)
  : Cost("JointVelIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      sco::AffExpr vel;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      sco::exprDec(vel, targets_[j]);  // offset to center about 0

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, vel);             // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form lower limit expr =  (upper_tol-(vel-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, vel);             // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

double JointVelIneqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd vel = diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (vel.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  return diff1.cwiseMax(0).sum() + diff2.cwiseMax(0).sum();
}

sco::ConvexObjectivePtr JointVelIneqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addHinge(expr, 1);
  }
  return out;
}

JointVelEqConstraint::JointVelEqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                           const Eigen::VectorXd& targets, int& first_step, int& last_step)
  : EqConstraint("JointVelEq")
  , vars_(vars)
  , coeffs_(coeffs)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      sco::AffExpr vel;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      sco::exprDec(vel, targets_[j]);
      // expr_ = coeff * vel - Not squared b/c QuadExpr cnt not yet supported (TODO)
      expr_vec_.push_back(sco::exprMult(vel, coeffs_[j]));
    }
  }
}

DblVec JointVelEqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff = (diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()))).rowwise() -
                         targets_.transpose();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return util::toDblVec((diff.array().square()).matrix() * coeffs_.asDiagonal());
}
sco::ConvexConstraintsPtr JointVelEqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addEqCnt(expr);
  }
  return out;
}

JointVelIneqConstraint::JointVelIneqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                               const Eigen::VectorXd& targets, const Eigen::VectorXd& upper_tols,
                                               const Eigen::VectorXd& lower_tols, int& first_step, int& last_step)
  : IneqConstraint("JointVelIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      sco::AffExpr vel;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      sco::exprDec(vel, targets_[j]);  // offset to center about 0

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, vel);             // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form lower limit expr = (lower_tol-(vel-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, vel);             // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

DblVec JointVelIneqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd vel = diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (vel.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  Eigen::MatrixXd out(diff1.rows(), diff1.cols() + diff2.cols());
  out << diff1, diff2;
  return util::toDblVec(out.cwiseMax(0));
}

sco::ConvexConstraintsPtr JointVelIneqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addIneqCnt(expr);
  }
  return out;
}

//////////////////// Acceleration /////////////////////
JointAccEqCost::JointAccEqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                               int& first_step, int& last_step)
  : Cost("JointAccEq"), vars_(vars), coeffs_(coeffs), targets_(targets), first_step_(first_step), last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // acc = (x3 - 2*x2 + x1) - targ
      sco::AffExpr acc;
      sco::exprInc(acc, sco::exprMult(vars(i, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), -2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 1.0));

      sco::exprDec(acc, targets_[j]);
      // expr_ = coeff * acc^2
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointAccEqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = (getTraj(xvec, vars_));
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())))).rowwise() -
      targets_.transpose();
  // Element-wise square it, multiply it by a diagonal matrix of coefficients, and sums output
  return (diff.array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointAccEqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointAccIneqCost::JointAccIneqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                                   const Eigen::VectorXd& upper_tols, const Eigen::VectorXd& lower_tols,
                                   int& first_step, int& last_step)
  : Cost("JointAccIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // acc = (x3 - 2*x2 + x1) - targ
      sco::AffExpr acc;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(acc, sco::exprMult(vars(i, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), -2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 1.0));
      sco::exprDec(acc, targets_[j]);  // offset to center about 0

      // Form upper limit expr = - (upper_tol-(acc-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, acc);             // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form lower limit expr = (lower_tol-(acc-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, acc);             // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

double JointAccIneqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd acc = diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (acc.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  return diff1.cwiseMax(0).sum() + diff2.cwiseMax(0).sum();
}

sco::ConvexObjectivePtr JointAccIneqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addHinge(expr, 1);
  }
  return out;
}

JointAccEqConstraint::JointAccEqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                           const Eigen::VectorXd& targets, int& first_step, int& last_step)
  : EqConstraint("JointAccEq")
  , vars_(vars)
  , coeffs_(coeffs)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // acc = (x3 - 2*x2 + x1) - targ
      sco::AffExpr acc;
      sco::exprInc(acc, sco::exprMult(vars(i, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), -2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 1.0));

      sco::exprDec(acc, targets_[j]);  // offset to center about 0
      // expr_ = coeff * vel - Not squared b/c QuadExpr cnt not yet supported (TODO)
      expr_vec_.push_back(sco::exprMult(acc, coeffs_[j]));
    }
  }
}

DblVec JointAccEqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())))).rowwise() -
      targets_.transpose();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return util::toDblVec((diff.array().square()).matrix() * coeffs_.asDiagonal());
}
sco::ConvexConstraintsPtr JointAccEqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addEqCnt(expr);
  }
  return out;
}

JointAccIneqConstraint::JointAccIneqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                               const Eigen::VectorXd& targets, const Eigen::VectorXd& upper_tols,
                                               const Eigen::VectorXd& lower_tols, int& first_step, int& last_step)
  : IneqConstraint("JointAccIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = first_step_; i <= last_step_ - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // acc = (x3 - 2*x2 + x1) - targ
      sco::AffExpr acc;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(acc, sco::exprMult(vars(i, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), -2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 1.0));
      sco::exprDec(acc, targets_[j]);  // offset to center about 0

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, acc);             // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, acc);             // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

DblVec JointAccIneqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd acc = diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (acc.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  Eigen::MatrixXd out(diff1.rows(), diff1.cols() + diff2.cols());
  out << diff1, diff2;
  return util::toDblVec(out.cwiseMax(0));
}

sco::ConvexConstraintsPtr JointAccIneqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addIneqCnt(expr);
  }
  return out;
}

//////////////////// Jerk /////////////////////
JointJerkEqCost::JointJerkEqCost(const VarArray& vars, const Eigen::VectorXd& coeffs, const Eigen::VectorXd& targets,
                                 int& first_step, int& last_step)
  : Cost("JointJerkEq"), vars_(vars), coeffs_(coeffs), targets_(targets), first_step_(first_step), last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr jerk;
      sco::exprInc(jerk, sco::exprMult(vars(i, j), -1.0 / 2.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 1, j), 1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 2, j), 0.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 3, j), -1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 4, j), 1.0 / 2.0));

      sco::exprDec(jerk, targets_[j]);
      // expr_ = coeff * jerk^2
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(jerk), coeffs_[j]));
    }
  }
}
double JointJerkEqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = (getTraj(xvec, vars_));
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (diffAxis0(diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())))))
          .rowwise() -
      targets_.transpose();
  // Element-wise square it, multiply it by a diagonal matrix of coefficients, and sums output
  return (diff.array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointJerkEqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointJerkIneqCost::JointJerkIneqCost(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                     const Eigen::VectorXd& targets, const Eigen::VectorXd& upper_tols,
                                     const Eigen::VectorXd& lower_tols, int& first_step, int& last_step)
  : Cost("JointJerkIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr jerk;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(jerk, sco::exprMult(vars(i, j), -1.0 / 2.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 1, j), 1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 2, j), 0.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 3, j), -1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 4, j), 1.0 / 2.0));
      sco::exprDec(jerk, targets_[j]);

      // Form upper limit expr = - (upper_tol-(jerk-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, jerk);            // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form lower limit expr = (lower_tol-(acc-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, jerk);            // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

double JointJerkIneqCost::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd jerk =
      diffAxis0(diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()))));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (jerk.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  return diff1.cwiseMax(0).sum() + diff2.cwiseMax(0).sum();
}

sco::ConvexObjectivePtr JointJerkIneqCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addHinge(expr, 1);
  }
  return out;
}

JointJerkEqConstraint::JointJerkEqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                             const Eigen::VectorXd& targets, int& first_step, int& last_step)
  : EqConstraint("JointJerkEq")
  , vars_(vars)
  , coeffs_(coeffs)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr jerk;
      sco::exprInc(jerk, sco::exprMult(vars(i, j), -1.0 / 2.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 1, j), 1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 2, j), 0.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 3, j), -1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 4, j), 1.0 / 2.0));

      sco::exprDec(jerk, targets_[j]);  // offset to center about 0
      // expr_ = coeff * jerk - Not squared b/c QuadExpr cnt not yet supported (TODO)
      expr_vec_.push_back(sco::exprMult(jerk, coeffs_[j]));
    }
  }
}

DblVec JointJerkEqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targets_ vector
  Eigen::MatrixXd diff =
      (diffAxis0(diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())))))
          .rowwise() -
      targets_.transpose();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return util::toDblVec((diff.array().square()).matrix() * coeffs_.asDiagonal());
}
sco::ConvexConstraintsPtr JointJerkEqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addEqCnt(expr);
  }
  return out;
}

JointJerkIneqConstraint::JointJerkIneqConstraint(const VarArray& vars, const Eigen::VectorXd& coeffs,
                                                 const Eigen::VectorXd& targets, const Eigen::VectorXd& upper_tols,
                                                 const Eigen::VectorXd& lower_tols, int& first_step, int& last_step)
  : IneqConstraint("JointJerkIneq")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targets_(targets)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr jerk;
      sco::AffExpr expr;
      sco::AffExpr expr_neg;
      sco::exprInc(jerk, sco::exprMult(vars(i, j), -1.0 / 2.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 1, j), 1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 2, j), 0.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 3, j), -1.0));
      sco::exprInc(jerk, sco::exprMult(vars(i + 4, j), 1.0 / 2.0));
      sco::exprDec(jerk, targets_[j]);  // offset to center about 0

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      sco::exprDec(expr, jerk);            // expr = upper_tol_- (vel - targets_)
      sco::exprScale(expr, -coeffs[j]);    // expr = - (upper_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr);

      // Form upper limit expr = - (upper_tol-(vel-targ))
      sco::exprInc(expr_neg, lower_tols_[j]);  // expr_ = lower_tol_
      sco::exprDec(expr_neg, jerk);            // expr = lower_tol_- (vel - targets_)
      sco::exprScale(expr_neg, coeffs[j]);     // expr = (lower_tol_- (vel - targets_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

DblVec JointJerkIneqConstraint::value(const DblVec& xvec)
{
  // Convert vector from optimization to trajectory
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  Eigen::MatrixXd acc = diffAxis0(diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols())));
  // Subtract targets_ to center about 0 and then subtract from tolerance
  Eigen::MatrixXd diff0 = (acc.rowwise() - targets_.transpose());
  Eigen::MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  Eigen::MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  Eigen::MatrixXd out(diff1.rows(), diff1.cols() + diff2.cols());
  out << diff1, diff2;
  return util::toDblVec(out.cwiseMax(0));
}

sco::ConvexConstraintsPtr JointJerkIneqConstraint::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (sco::AffExpr& expr : expr_vec_)
  {
    out->addIneqCnt(expr);
  }
  return out;
}

}  // namespace trajopt
