#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/format.hpp>
#include <iostream>
#include <map>
#include <sstream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
const std::vector<std::string> ModelType::MODEL_NAMES_ = { "GUROBI", "BPMPD", "OSQP", "QPOASES", "AUTO_SOLVER" };

IntVec vars2inds(const VarVector& vars)
{
  IntVec inds(vars.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = vars[i].var_rep->index;
  return inds;
}
IntVec cnts2inds(const CntVector& cnts)
{
  IntVec inds(cnts.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = cnts[i].cnt_rep->index;
  return inds;
}

void simplify2(IntVec& inds, DblVec& vals)
{
  typedef std::map<int, double> Int2Double;
  Int2Double ind2val;
  for (unsigned i = 0; i < inds.size(); ++i)
  {
    if (vals[i] != 0.0)
      ind2val[inds[i]] += vals[i];
  }
  inds.resize(ind2val.size());
  vals.resize(ind2val.size());
  long unsigned int i_new = 0;
  for (Int2Double::value_type& iv : ind2val)
  {
    inds[i_new] = iv.first;
    vals[i_new] = iv.second;
    ++i_new;
  }
}

double AffExpr::value(const double* x) const
{
  double out = constant;
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double AffExpr::value(const DblVec& x) const
{
  double out = constant;
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double QuadExpr::value(const DblVec& x) const
{
  double out = affexpr.value(x);
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}
double QuadExpr::value(const double* x) const
{
  double out = affexpr.value(x);
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}

Var Model::addVar(const std::string& name, double lb, double ub)
{
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVar(const Var& var)
{
  VarVector vars(1, var);
  removeVars(vars);
}
void Model::removeCnt(const Cnt& cnt)
{
  CntVector cnts(1, cnt);
  removeCnts(cnts);
}

double Model::getVarValue(const Var& var) const
{
  VarVector vars(1, var);
  return getVarValues(vars)[0];
}

void Model::setVarBounds(const Var& var, double lower, double upper)
{
  DblVec lowers(1, lower), uppers(1, upper);
  VarVector vars(1, var);
  setVarBounds(vars, lowers, uppers);
}

std::ostream& operator<<(std::ostream& o, const Var& v)
{
  if (v.var_rep != nullptr)
    o << v.var_rep->name;
  else
    o << "nullvar";
  return o;
}

std::ostream& operator<<(std::ostream& o, const Cnt& c)
{
  o << c.cnt_rep->expr << ((c.cnt_rep->type == EQ) ? " == 0" : " <= 0");
  return o;
}

std::ostream& operator<<(std::ostream& o, const AffExpr& e)
{
  o << e.constant;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars[i];
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, const QuadExpr& e)
{
  o << e.affexpr;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars1[i] << "*" << e.vars2[i];
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, const ModelType& cs)
{
  size_t cs_ivalue_ = static_cast<size_t>(cs.value_);
  if (cs_ivalue_ > cs.MODEL_NAMES_.size())
  {
    std::stringstream conversion_error;
    conversion_error << "Error converting ModelType to string - "
                     << "enum value is " << cs_ivalue_ << std::endl;
    throw std::runtime_error(conversion_error.str());
  }
  o << ModelType::MODEL_NAMES_[cs_ivalue_];
  return o;
}

ModelType::ModelType() { value_ = ModelType::AUTO_SOLVER; }
ModelType::ModelType(const ModelType::Value& v) { value_ = v; }
ModelType::ModelType(const int& v) { value_ = static_cast<Value>(v); }
ModelType::ModelType(const std::string& s)
{
  for (unsigned int i = 0; i < ModelType::MODEL_NAMES_.size(); ++i)
  {
    if (s == ModelType::MODEL_NAMES_[i])
    {
      value_ = static_cast<ModelType::Value>(i);
      return;
    }
  }
  PRINT_AND_THROW(boost::format("invalid solver name:\"%s\"") % s);
}

ModelType::operator int() const { return static_cast<int>(value_); }
bool ModelType::operator==(const ModelType::Value& a) const { return value_ == a; }
bool ModelType::operator==(const ModelType& a) const { return value_ == a.value_; }
bool ModelType::operator!=(const ModelType& a) const { return value_ != a.value_; }
void ModelType::fromJson(const Json::Value& v)
{
  try
  {
    std::string ref = v.asString();
    ModelType cs(ref);
    value_ = cs.value_;
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("string") % (v));
  }
}

std::vector<ModelType> availableSolvers()
{
  std::vector<bool> has_solver(ModelType::AUTO_SOLVER, false);
#ifdef HAVE_GUROBI
  has_solver[ModelType::GUROBI] = true;
#endif
#ifdef HAVE_BPMPD
  has_solver[ModelType::BPMPD] = true;
#endif
#ifdef HAVE_OSQP
  has_solver[ModelType::OSQP] = true;
#endif
#ifdef HAVE_QPOASES
  has_solver[ModelType::QPOASES] = true;
#endif
  size_t n_available_solvers = 0;
  for (auto i = 0; i < ModelType::AUTO_SOLVER; ++i)
    if (has_solver[static_cast<size_t>(i)])
      ++n_available_solvers;
  std::vector<ModelType> available_solvers(n_available_solvers, ModelType::AUTO_SOLVER);

  size_t j = 0;
  for (int i = 0; i < static_cast<int>(ModelType::AUTO_SOLVER); ++i)
    if (has_solver[static_cast<size_t>(i)])
      available_solvers[j++] = static_cast<ModelType>(i);
  return available_solvers;
}

ModelPtr createModel(ModelType model_type)
{
#ifdef HAVE_GUROBI
  extern ModelPtr createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  extern ModelPtr createBPMPDModel();
#endif
#ifdef HAVE_OSQP
  extern ModelPtr createOSQPModel();
#endif
#ifdef HAVE_QPOASES
  extern ModelPtr createqpOASESModel();
#endif

  char* solver_env = getenv("TRAJOPT_CONVEX_SOLVER");

  ModelType solver = model_type;

  if (solver == ModelType::AUTO_SOLVER)
  {
    if (solver_env and std::string(solver_env) != "AUTO_SOLVER")
    {
      try
      {
        solver = ModelType(std::string(solver_env));
      }
      catch (std::runtime_error&)
      {
        PRINT_AND_THROW(boost::format("invalid solver \"%s\"specified by TRAJOPT_CONVEX_SOLVER") % solver_env);
      }
    }
    else
    {
      solver = availableSolvers()[0];
    }
  }

#ifndef HAVE_GUROBI
  if (solver == ModelType::GUROBI)
    PRINT_AND_THROW("you didn't build with GUROBI support");
#endif
#ifndef HAVE_BPMPD
  if (solver == ModelType::BPMPD)
    PRINT_AND_THROW("you don't have BPMPD support on this platform");
#endif
#ifndef HAVE_OSQP
  if (solver == ModelType::OSQP)
    PRINT_AND_THROW("you don't have OSQP support on this platform");
#endif
#ifndef HAVE_QPOASES
  if (solver == ModelType::QPOASES)
    PRINT_AND_THROW("you don't have qpOASES support on this platform");
#endif

#ifdef HAVE_GUROBI
  if (solver == ModelType::GUROBI)
    return createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  if (solver == ModelType::BPMPD)
    return createBPMPDModel();
#endif
#ifdef HAVE_OSQP
  if (solver == ModelType::OSQP)
    return createOSQPModel();
#endif
#ifdef HAVE_QPOASES
  if (solver == ModelType::QPOASES)
    return createqpOASESModel();
#endif
  std::stringstream solver_instatiation_error;
  solver_instatiation_error << "Failed to create solver: unknown solver " << solver << std::endl;
  PRINT_AND_THROW(solver_instatiation_error.str());
  return ModelPtr();
}
}
