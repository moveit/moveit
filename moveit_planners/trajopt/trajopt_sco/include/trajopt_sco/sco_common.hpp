#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cmath>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/sco_fwd.hpp>

namespace sco
{
typedef std::vector<double> DblVec;
typedef std::vector<int> IntVec;
typedef std::vector<Var> VarVector;
typedef std::vector<AffExpr> AffExprVector;
typedef std::vector<QuadExpr> QuadExprVector;
typedef std::vector<Cnt> CntVector;

inline double vecSum(const DblVec& v)
{
  double out = 0;
  for (unsigned i = 0; i < v.size(); ++i)
    out += v[i];
  return out;
}
inline double vecAbsSum(const DblVec& v)
{
  double out = 0;
  for (unsigned i = 0; i < v.size(); ++i)
    out += fabs(v[i]);
  return out;
}
inline double pospart(double x) { return (x > 0) ? x : 0; }
inline double sq(double x) { return x * x; }
inline double vecHingeSum(const DblVec& v)
{
  double out = 0;
  for (unsigned i = 0; i < v.size(); ++i)
    out += pospart(v[i]);
  return out;
}
inline double vecMax(const DblVec& v) { return *std::max_element(v.begin(), v.end()); }
inline double vecDot(const DblVec& a, const DblVec& b)
{
  assert(a.size() == b.size());
  double out = 0;
  for (unsigned i = 0; i < a.size(); ++i)
    out += a[i] * b[i];
  return out;
}
}
