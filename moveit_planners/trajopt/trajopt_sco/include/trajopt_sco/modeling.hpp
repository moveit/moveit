#pragma once

/*
 * Model a non-convex optimization problem by defining Cost and Constraint
 * objects
 * which know how to generate a convex approximation
 *
 *
 */
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
/**
Stores convex terms in a objective
For non-quadratic terms like hinge(x) and abs(x), it needs to add auxilliary
variables and linear constraints to the model
Note: When this object is deleted, the constraints and variables it added to the
model are removed
 */
class ConvexObjective
{
public:
  ConvexObjective(Model* model) : model_(model) {}
  void addAffExpr(const AffExpr&);
  void addQuadExpr(const QuadExpr&);
  void addHinge(const AffExpr&, double coeff);
  void addAbs(const AffExpr&, double coeff);
  void addHinges(const AffExprVector&);
  void addL1Norm(const AffExprVector&);
  void addL2Norm(const AffExprVector&);
  void addMax(const AffExprVector&);

  bool inModel() { return model_ != nullptr; }
  void addConstraintsToModel();
  void removeFromModel();
  double value(const DblVec& x);

  ~ConvexObjective();

  Model* model_;
  QuadExpr quad_;
  VarVector vars_;
  AffExprVector eqs_;
  AffExprVector ineqs_;
  CntVector cnts_;

private:
  ConvexObjective() {}
  ConvexObjective(ConvexObjective&) {}
};

/**
Stores convex inequality constraints and affine equality constraints.
Actually only affine inequality constraints are currently implemented.
*/
class ConvexConstraints
{
public:
  ConvexConstraints(Model* model) : model_(model) {}
  /** Expression that should == 0 */
  void addEqCnt(const AffExpr&);
  /** Expression that should <= 0 */
  void addIneqCnt(const AffExpr&);
  void setModel(Model* model)
  {
    assert(!inModel());
    model_ = model;
  }
  bool inModel() { return model_ != nullptr; }
  void addConstraintsToModel();
  void removeFromModel();

  DblVec violations(const DblVec& x);
  double violation(const DblVec& x);

  ~ConvexConstraints();
  AffExprVector eqs_;
  AffExprVector ineqs_;

private:
  Model* model_;
  CntVector cnts_;
  ConvexConstraints() : model_(nullptr) {}
  ConvexConstraints(ConvexConstraints&) {}
};

/**
Non-convex cost function, which knows how to calculate its convex approximation
(convexify() method)
*/
class Cost
{
public:
  /** Evaluate at solution vector x*/
  virtual double value(const DblVec&) = 0;
  /** Convexify at solution vector x*/
  virtual ConvexObjectivePtr convex(const DblVec& x, Model* model) = 0;
  /** Get problem variables associated with this cost */
  virtual VarVector getVars() = 0;
  std::string name() { return name_; }
  void setName(const std::string& name) { name_ = name; }
  Cost() : name_("unnamed") {}
  Cost(const std::string& name) : name_(name) {}
  virtual ~Cost() = default;

protected:
  std::string name_;
};

/**
Non-convex vector-valued constraint function, which knows how to calculate its
convex approximation
*/
class Constraint
{
public:
  /** inequality vs equality */
  virtual ConstraintType type() = 0;
  /** Evaluate at solution vector x*/
  virtual DblVec value(const DblVec& x) = 0;
  /** Convexify at solution vector x*/
  virtual ConvexConstraintsPtr convex(const DblVec& x, Model* model) = 0;
  /** Calculate constraint violations (positive part for inequality constraint,
   * absolute value for inequality constraint)*/
  DblVec violations(const DblVec& x);
  /** Sum of violations */
  double violation(const DblVec& x);
  /** Get problem variables associated with this constraint */
  virtual VarVector getVars() = 0;
  std::string name() { return name_; }
  void setName(const std::string& name) { name_ = name; }
  Constraint() : name_("unnamed") {}
  Constraint(const std::string& name) : name_(name) {}
  virtual ~Constraint() {}
protected:
  std::string name_;
};

class EqConstraint : public Constraint
{
public:
  ConstraintType type() override { return EQ; }
  EqConstraint() : Constraint() {}
  EqConstraint(const std::string& name) : Constraint(name) {}
};

class IneqConstraint : public Constraint
{
public:
  ConstraintType type() override { return INEQ; }
  IneqConstraint() : Constraint() {}
  IneqConstraint(const std::string& name) : Constraint(name) {}
};

/**
Non-convex optimization problem
*/
class OptProb
{
public:
  OptProb(ModelType convex_solver = ModelType::AUTO_SOLVER);
  virtual ~OptProb() = default;

  /** create variables with bounds [-INFINITY, INFINITY]  */
  VarVector createVariables(const std::vector<std::string>& names);
  /** create variables with bounds [lb[i], ub[i] */
  VarVector createVariables(const std::vector<std::string>& names, const DblVec& lb, const DblVec& ub);
  /** set the lower bounds of all the variables */
  void setLowerBounds(const DblVec& lb);
  /** set the upper bounds of all the variables */
  void setUpperBounds(const DblVec& ub);
  /** set lower bounds of some of the variables */
  void setLowerBounds(const DblVec& lb, const VarVector& vars);
  /** set upper bounds of some of the variables */
  void setUpperBounds(const DblVec& ub, const VarVector& vars);
  /** Note: in the current implementation, this function just adds the
   * constraint to the
   * model. So if you're not careful, you might end up with an infeasible
   * problem. */
  void addLinearConstraint(const AffExpr&, ConstraintType type);
  /** Add nonlinear cost function */
  void addCost(CostPtr);
  /** Add nonlinear constraint function */
  void addConstraint(ConstraintPtr);
  void addEqConstraint(ConstraintPtr);
  void addIneqConstraint(ConstraintPtr);
  /** Find closest point to solution vector x that satisfies linear inequality
   * constraints */
  DblVec getCentralFeasiblePoint(const DblVec& x);
  DblVec getClosestFeasiblePoint(const DblVec& x);

  std::vector<ConstraintPtr> getConstraints() const;
  const std::vector<CostPtr>& getCosts() { return costs_; }
  const std::vector<ConstraintPtr>& getIneqConstraints() { return ineqcnts_; }
  const std::vector<ConstraintPtr>& getEqConstraints() { return eqcnts_; }
  const DblVec& getLowerBounds() { return lower_bounds_; }
  const DblVec& getUpperBounds() { return upper_bounds_; }
  ModelPtr getModel() { return model_; }
  const VarVector& getVars() { return vars_; }
  int getNumCosts() { return static_cast<int>(costs_.size()); }
  int getNumConstraints() { return static_cast<int>(eqcnts_.size() + ineqcnts_.size()); }
  int getNumVars() { return static_cast<int>(vars_.size()); }
protected:
  ModelPtr model_;
  VarVector vars_;
  DblVec lower_bounds_;
  DblVec upper_bounds_;
  std::vector<CostPtr> costs_;
  std::vector<ConstraintPtr> eqcnts_;
  std::vector<ConstraintPtr> ineqcnts_;

  OptProb(OptProb&);
};

template <typename VecType>
inline void setVec(DblVec& x, const VarVector& vars, const VecType& vals)
{
  assert(vars.size() == vals.size());
  for (size_t i = 0; i < vars.size(); ++i)
  {
    x[static_cast<size_t>(vars[i].var_rep->index)] = vals[i];
  }
}
template <typename OutVecType>
inline OutVecType getVec1(const DblVec& x, const VarVector& vars)
{
  OutVecType out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
    out[i] = x[vars[i].var_rep->index];
  return out;
}
}
