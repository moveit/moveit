#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <Eigen/SparseCore>
#include <sstream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>

namespace sco
{
void exprToEigen(const AffExpr& expr, Eigen::SparseVector<double>& sparse_vector, const int& n_vars)
{
  sparse_vector.resize(n_vars);
  sparse_vector.reserve(static_cast<long int>(expr.size()));
  for (size_t i = 0; i < expr.size(); ++i)
  {
    int i_var_index = expr.vars[i].var_rep->index;
    if (i_var_index >= n_vars)
    {
      std::stringstream msg;
      msg << "Coefficient " << i << "has index " << i_var_index << " but n_vars is " << n_vars;
      throw std::runtime_error(msg.str());
    }
    if (expr.coeffs[i] != 0.)
      sparse_vector.coeffRef(i_var_index) += expr.coeffs[i];
  }
}

void exprToEigen(const QuadExpr& expr,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars,
                 const bool& matrix_is_halved,
                 const bool& force_diagonal)
{
  IntVec ind1 = vars2inds(expr.vars1);
  IntVec ind2 = vars2inds(expr.vars2);
  sparse_matrix.resize(n_vars, n_vars);
  sparse_matrix.reserve(static_cast<long int>(2 * expr.size()));

  Eigen::SparseVector<double> vector_sparse;
  exprToEigen(expr.affexpr, vector_sparse, n_vars);
  vector = vector_sparse;

  for (size_t i = 0; i < expr.coeffs.size(); ++i)
  {
    if (expr.coeffs[i] != 0.0)
    {
      if (ind1[i] == ind2[i])
        sparse_matrix.coeffRef(ind1[i], ind2[i]) += expr.coeffs[i];
      else
      {
        int c, r;
        if (ind1[i] < ind2[i])
        {
          r = ind1[i];
          c = ind2[i];
        }
        else
        {
          r = ind2[i];
          c = ind1[i];
        }
        sparse_matrix.coeffRef(r, c) += expr.coeffs[i];
      }
    }
  }

  auto sparse_matrix_T = Eigen::SparseMatrix<double>(sparse_matrix.transpose());
  sparse_matrix = sparse_matrix + sparse_matrix_T;

  if (!matrix_is_halved)
    sparse_matrix = 0.5 * sparse_matrix;

  if (force_diagonal)
    for (int k = 0; k < n_vars; ++k)
      sparse_matrix.coeffRef(k, k) += 0.0;
}

void exprToEigen(const AffExprVector& expr_vec,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars)
{
  vector.resize(static_cast<long int>(expr_vec.size()));
  vector.setZero();
  sparse_matrix.resize(static_cast<long int>(expr_vec.size()), n_vars);
  sparse_matrix.reserve(static_cast<long int>(expr_vec.size()) * n_vars);

  for (long int i = 0; i < static_cast<long int>(expr_vec.size()); ++i)
  {
    const AffExpr& expr = expr_vec[static_cast<size_t>(i)];
    Eigen::SparseVector<double> expr_vector;
    exprToEigen(expr, expr_vector, n_vars);
    for (Eigen::SparseVector<double>::InnerIterator it(expr_vector); it; ++it)
      sparse_matrix.coeffRef(i, it.index()) = it.value();
    vector[i] = -expr.constant;
  }
}

void tripletsToEigen(const IntVec& rows_i,
                     const IntVec& cols_j,
                     const DblVec& values_ij,
                     Eigen::SparseMatrix<double>& sparse_matrix)
{
  typedef Eigen::Triplet<double> T;
  std::vector<T, Eigen::aligned_allocator<T>> triplets;
  for (unsigned int i = 0; i < values_ij.size(); ++i)
    triplets.push_back(T(rows_i[i], cols_j[i], values_ij[i]));
  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());
}

void eigenToTriplets(const Eigen::SparseMatrix<double>& sparse_matrix,
                     IntVec& rows_i,
                     IntVec& cols_j,
                     DblVec& values_ij)
{
  auto& sm = sparse_matrix;
  rows_i.reserve(rows_i.size() + static_cast<size_t>(sm.nonZeros()));
  cols_j.reserve(cols_j.size() + static_cast<size_t>(sm.nonZeros()));
  values_ij.reserve(values_ij.size() + static_cast<size_t>(sm.nonZeros()));
  for (int k = 0; k < sm.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sm, k); it; ++it)
    {
      rows_i.push_back(static_cast<int>(it.row()));
      cols_j.push_back(static_cast<int>(it.col()));
      values_ij.push_back(it.value());
    }
  }
}
}
