#pragma once
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/cache.hxx>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/sco_fwd.hpp>

namespace trajopt
{
struct CollisionEvaluator
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionEvaluator(tesseract::BasicKinConstPtr manip,
                     tesseract::BasicEnvConstPtr env,
                     SafetyMarginDataConstPtr safety_margin_data)
    : env_(env), manip_(manip), safety_margin_data_(safety_margin_data)
  {
  }
  virtual ~CollisionEvaluator() = default;
  virtual void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, tesseract::ContactResultVector&);
  //virtual void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) = 0;
  virtual sco::VarVector GetVars() = 0;

  const SafetyMarginDataConstPtr getSafetyMarginData() const { return safety_margin_data_; }
  
  Cache<size_t, tesseract::ContactResultVector, 10> m_cache;
  // this calss is not dependent to any  tesseract stuff, I just need to figure out ContactResulVector that is passed to it
  // I do not understand what it is doing exactly though

  // what is ContactResultVector?
  // is an aligned vector which works with memory. Basically, it is a vector containing ContactResult
  // now what is ContactResult? it has all the informatin related to contact between two bodies

  // so I need to find the similar type to ContactResult in MoveIt

protected:
  tesseract::BasicEnvConstPtr env_;
  tesseract::BasicKinConstPtr manip_;
  SafetyMarginDataConstPtr safety_margin_data_;

private:
  CollisionEvaluator() {}
};

typedef std::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                                   tesseract::BasicEnvConstPtr env,
                                   SafetyMarginDataConstPtr safety_margin_data,
                                   const sco::VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function
  */
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results) override;
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return m_vars; }
private:
  sco::VarVector m_vars;
  tesseract::DiscreteContactManagerBasePtr contact_manager_;
};

struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  CastCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                         tesseract::BasicEnvConstPtr env,
                         SafetyMarginDataConstPtr safety_margin_data,
                         const sco::VarVector& vars0,
                         const sco::VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results) override;
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return concat(m_vars0, m_vars1); }
private:
  sco::VarVector m_vars0;
  sco::VarVector m_vars1;
  tesseract::ContinuousContactManagerBasePtr contact_manager_;
};

class TRAJOPT_API CollisionCost : public sco::Cost, public Plotter
{
public:
  /* constructor for single timestep */
  CollisionCost(tesseract::BasicKinConstPtr manip,
                tesseract::BasicEnvConstPtr env,
                SafetyMarginDataConstPtr safety_margin_data,
                const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(tesseract::BasicKinConstPtr manip,
                tesseract::BasicEnvConstPtr env,
                SafetyMarginDataConstPtr safety_margin_data,
                const sco::VarVector& vars0,
                const sco::VarVector& vars1);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model) override;
  virtual double value(const DblVec&) override;
  void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) override;
  sco::VarVector getVars() override { return m_calc->GetVars(); }
private:
  CollisionEvaluatorPtr m_calc;
};

class TRAJOPT_API CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(tesseract::BasicKinConstPtr manip,
                      tesseract::BasicEnvConstPtr env,
                      SafetyMarginDataConstPtr safety_margin_data,
                      const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(tesseract::BasicKinConstPtr manip,
                      tesseract::BasicEnvConstPtr env,
                      SafetyMarginDataConstPtr safety_margin_data,
                      const sco::VarVector& vars0,
                      const sco::VarVector& vars1);
  virtual sco::ConvexConstraintsPtr convex(const DblVec& x, sco::Model* model) override;
  virtual DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override { return m_calc->GetVars(); }
private:
  CollisionEvaluatorPtr m_calc;
};
}
