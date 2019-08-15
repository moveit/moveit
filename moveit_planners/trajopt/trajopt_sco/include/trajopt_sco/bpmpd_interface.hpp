#pragma once
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
class BPMPDModel : public Model
{
public:
  VarVector m_vars;
  CntVector m_cnts;
  AffExprVector m_cntExprs;
  ConstraintTypeVector m_cntTypes;
  DblVec m_soln;
  DblVec m_lbs, m_ubs;

  QuadExpr m_objective;

  int m_pipeIn, m_pipeOut, m_pid;

  BPMPDModel();
  ~BPMPDModel() override;

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
