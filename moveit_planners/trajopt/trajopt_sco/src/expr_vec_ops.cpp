#include <trajopt_sco/expr_vec_ops.hpp>
namespace sco
{
AffExpr varDot(const Eigen::VectorXd& x, const VarVector& v)
{
  AffExpr out;
  out.constant = 0;
  out.vars = v;
  out.coeffs = DblVec(x.data(), x.data() + x.size());
  return out;
}
}
