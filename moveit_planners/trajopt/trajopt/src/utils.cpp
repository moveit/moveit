#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <boost/format.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/utils.hpp>
#include <trajopt_sco/solver_interface.hpp>

namespace trajopt
{
TrajArray getTraj(const DblVec& x, const VarArray& vars)
{
  TrajArray out(vars.rows(), vars.cols());
  for (int i = 0; i < vars.rows(); ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      out(i, j) = vars(i, j).value(x);
    }
  }
  return out;
}

TrajArray getTraj(const DblVec& x, const AffArray& arr)
{
  Eigen::MatrixXd out(arr.rows(), arr.cols());
  for (int i = 0; i < arr.rows(); ++i)
  {
    for (int j = 0; j < arr.cols(); ++j)
    {
      out(i, j) = arr(i, j).value(x);
    }
  }
  return out;
}

void AddVarArrays(sco::OptProb& prob,
                  int rows,
                  const IntVec& cols,
                  const std::vector<std::string>& name_prefix,
                  const std::vector<VarArray*>& newvars)
{
  size_t n_arr = name_prefix.size();
  assert(static_cast<unsigned>(n_arr) == newvars.size());

  std::vector<Eigen::MatrixXi> index(n_arr);
  for (size_t i = 0; i < n_arr; ++i)
  {
    newvars[i]->resize(rows, cols[i]);
    index[i].resize(rows, cols[i]);
  }

  std::vector<std::string> names;
  int var_idx = prob.getNumVars();
  for (int i = 0; i < rows; ++i)
  {
    for (size_t k = 0; k < n_arr; ++k)
    {
      for (int j = 0; j < cols[k]; ++j)
      {
        index[k](i, j) = var_idx;
        names.push_back((boost::format("%s_%i_%i") % name_prefix[k] % i % j).str());
        ++var_idx;
      }
    }
  }
  prob.createVariables(names);  // note that w,r, are both unbounded

  const std::vector<sco::Var>& vars = prob.getVars();
  for (size_t k = 0; k < n_arr; ++k)
  {
    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols[k]; ++j)
      {
        (*newvars[k])(i, j) = vars[static_cast<size_t>(index[k](i, j))];
      }
    }
  }
}

void AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars)
{
  std::vector<VarArray*> arrs(1, &newvars);
  std::vector<std::string> prefixes(1, name_prefix);
  std::vector<int> colss(1, cols);
  AddVarArrays(prob, rows, colss, prefixes, arrs);
}
}
