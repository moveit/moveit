#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>

static inline double sq(double x) { return x * x; }
namespace sco
{
QuadExpr exprMult(const AffExpr& affexpr1, const AffExpr& affexpr2)
{
  QuadExpr out;
  size_t naff1 = affexpr1.coeffs.size();
  size_t naff2 = affexpr2.coeffs.size();
  size_t nquad = naff1 * naff2;

  // Multiply the constants of the two expr
  out.affexpr.constant = affexpr1.constant * affexpr2.constant;

  // Account for vars in each expr multiplied by the constant in the other expr
  out.affexpr.vars.reserve(naff1 + naff2);
  out.affexpr.vars.insert(out.affexpr.vars.end(), affexpr1.vars.begin(), affexpr1.vars.end());
  out.affexpr.vars.insert(out.affexpr.vars.end(), affexpr2.vars.begin(), affexpr2.vars.end());
  out.affexpr.coeffs.resize(naff1 + naff2);
  for (size_t i = 0; i < naff1; ++i)
    out.affexpr.coeffs[i] = affexpr2.constant * affexpr1.coeffs[i];
  for (size_t i = 0; i < naff2; ++i)
    out.affexpr.coeffs[i + naff1] = affexpr1.constant * affexpr2.coeffs[i];

  // Account for the vars in each expr that are multipled by another var in the other expr
  out.coeffs.reserve(nquad);
  out.vars1.reserve(nquad);
  out.vars2.reserve(nquad);
  for (size_t i = 0; i < naff1; ++i)
  {
    for (size_t j = 0; j < naff2; ++j)
    {
      out.vars1.push_back(affexpr1.vars[i]);
      out.vars2.push_back(affexpr2.vars[j]);
      out.coeffs.push_back(affexpr1.coeffs[i] * affexpr2.coeffs[j]);
    }
  }
  return out;
}

QuadExpr exprSquare(const Var& a)
{
  QuadExpr out;
  out.coeffs.push_back(1);
  out.vars1.push_back(a);
  out.vars2.push_back(a);
  return out;
}

QuadExpr exprSquare(const AffExpr& affexpr)
{
  QuadExpr out;
  size_t naff = affexpr.coeffs.size();
  size_t nquad = (naff * (naff + 1)) / 2;

  out.affexpr.constant = sq(affexpr.constant);

  out.affexpr.vars = affexpr.vars;
  out.affexpr.coeffs.resize(naff);
  for (size_t i = 0; i < naff; ++i)
    out.affexpr.coeffs[i] = 2 * affexpr.constant * affexpr.coeffs[i];

  out.coeffs.reserve(nquad);
  out.vars1.reserve(nquad);
  out.vars2.reserve(nquad);
  for (size_t i = 0; i < naff; ++i)
  {
    out.vars1.push_back(affexpr.vars[i]);
    out.vars2.push_back(affexpr.vars[i]);
    out.coeffs.push_back(sq(affexpr.coeffs[i]));
    for (size_t j = i + 1; j < naff; ++j)
    {
      out.vars1.push_back(affexpr.vars[i]);
      out.vars2.push_back(affexpr.vars[j]);
      out.coeffs.push_back(2 * affexpr.coeffs[i] * affexpr.coeffs[j]);
    }
  }
  return out;
}

AffExpr cleanupAff(const AffExpr& a)
{
  AffExpr out;
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (fabs(a.coeffs[i]) > 1e-7)
    {
      out.coeffs.push_back(a.coeffs[i]);
      out.vars.push_back(a.vars[i]);
    }
  }
  out.constant = a.constant;
  return out;
}

///////////////////////////////////////////////////////////////
}  // namespace sco
