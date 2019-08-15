#pragma once
#include <trajopt_sco/solver_interface.hpp>

/**

@file gurobi_interface.hpp

Gurobi backend

*/

struct _GRBmodel;
typedef struct _GRBmodel GRBmodel;

namespace sco
{
class GurobiModel : public Model
{
public:
  GRBmodel* m_model;
  VarVector m_vars;
  CntVector m_cnts;

  GurobiModel();

  Var addVar(const std::string& name) override;
  Var addVar(const std::string& name, double lower, double upper) override;

  Cnt addEqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const QuadExpr&, const std::string& name) override;

  void removeVars(const VarVector&) override;
  void removeCnts(const CntVector&) override;

  void update() override;
  void setVarBounds(const VarVector&, const DblVec& lower, const DblVec& upper) override;
  DblVec getVarValues(const VarVector&) const override;

  CvxOptStatus optimize() override;
  /** Don't use this function, because it adds constraints that aren't tracked
   */
  CvxOptStatus optimizeFeasRelax();

  void setObjective(const AffExpr&) override;
  void setObjective(const QuadExpr&) override;
  void writeToFile(const std::string& fname) override;

  VarVector getVars() const override;

  ~GurobiModel();
};
}
