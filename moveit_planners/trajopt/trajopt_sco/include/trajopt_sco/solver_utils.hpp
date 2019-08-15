#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <iostream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
/**
 * @brief transform an `AffExpr` to an `Eigen::SparseVector`
 *
 * @param [in] expr an `AffExpr` expression
 * @param [out] sparse_vector vector where to store results. It will be resized
 *                            to the correct size.
 * @param [in] n_vars the number of variables in expr. It is usually equal
 *                    to `expr.size()`, but it might be larger.
 */
void exprToEigen(const AffExpr& expr, Eigen::SparseVector<double>& sparse_vector, const int& n_vars);

/**
 * @brief transform a `QuadExpr` to an `Eigen::SparseMatrix` plus
 *        `Eigen::VectorXd`
 *
 * @param [in] expr a `QuadExpr` expression
 * @param [out] sparse_matrix matrix where to store results. It will be resized
 *                            to the correct size.
 * @param [out] vector        vector where to store the affine part of the
 *                            `QuadExpr`. It will be resized to the correct size.
 * @param [in] n_vars the number of variables in expr. It is usually equal
 *                    to `expr.size()`, but it might be larger.
 * @param [in] matrix_is_halved if `true`, sparse_matrix will be premultiplied
 *                              by the coefficient `2`.
 * @param [in] force_diagonal if true, we will forcibly add elements to the
 *                            diagonal of the sparse matrix, adding `0.` if needed
 */
void exprToEigen(const QuadExpr& expr,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars,
                 const bool& matrix_is_halved = false,
                 const bool& force_diagonal = false);

/**
 * @brief transform a vector of `AffExpr` to an `Eigen::SparseMatrix` plus an
 *        `Eigen::VectorXd`.
 *        Notice that the underlying transformation is so that the the affine
 *        expressions of the type `ax + b` will be stacked in matrix form into
 *        `Ax + b` and then transformed into the expression `Ax <= -b` so that
 *        `vector[i] = -expr_vec[i].constant`
 * @param [in] expr_vec a an `AffExprVector`
 * @param [out] sparse_matrix matrix where to store results. It will be resized
 *                            to the correct size.
 * @param [out] vector vector where to store all the constants of each `AffExpr`,
 *                     ordered by their appearance order in `expr_vec`.
 *                     It will be resized to the correct size.
 * @param [in] n_vars the larger number of variables in expr.
 *                    It is usually the same for each `expr` in `expr_vec`,
 *                    and equal to `expr.size()`, but it might be larger.
 */
void exprToEigen(const AffExprVector& expr_vec,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars = -1);
/**
 * @brief Converts triplets to an `Eigen::SparseMatrix`.
 * @param [in] rows_i a vector of row indices
 * @param [in] cols_j a vector of columns indices
 * @param [in] values_ij a vector of values, so that:
 *                       `M[rows_i[k], cols_j[k]] = values_ij[k]`
 * @param [in,out] sparse_matrix must be of the right size, as we should not
 *                               be guessing the right size of sparse_matrix from
 *                               a sparse triplet representation.
 */
void tripletsToEigen(const IntVec& rows_i,
                     const IntVec& cols_j,
                     const DblVec& values_ij,
                     Eigen::SparseMatrix<double>& sparse_matrix);

/**
 * @brief Converts an `Eigen::SparseMatrix` into triplets format
 * @param [in] sparse_matrix an `Eigen::SparseMatrix`
 * @param [out] rows_i a vector of row indices
 * @param [out] cols_j a vector of columns indices
 * @param [out] values_ij a vector of values, so that:
 *                       `M[rows_i[k], cols_j[k]] = values_ij[k]`
 */
void eigenToTriplets(const Eigen::SparseMatrix<double>& sparse_matrix,
                     IntVec& rows_i,
                     IntVec& cols_j,
                     DblVec& values_ij);

/**
 * @brief converts a sparse matrix into compressed
 *        sparse column representation (CSC).
 *
 * @param [out] row_indices row indices for a CSC matrix
 * @param [out] column_pointers column pointer for a CSC matrix
 * @param [out] values pointer to non-zero elements in CSC representation
 * @param [in,out] sparse_matrix input matrix: will be compressed
 */
template <int eigenUpLoType = 0, typename T>
void eigenToCSC(Eigen::SparseMatrix<double>& sparse_matrix,
                std::vector<T>& row_indices,
                std::vector<T>& column_pointers,
                DblVec& values)
{
  Eigen::SparseMatrix<double> sm_t;
  auto sm_ref = std::ref(sparse_matrix);

  if (eigenUpLoType > 0)
  {
    sm_t = sparse_matrix.triangularView<eigenUpLoType>();
    sm_ref = std::ref(sm_t);
  }
  sm_ref.get().makeCompressed();

  if (sizeof(T) != sizeof(int))
  {
    IntVec row_indices_int, column_pointers_int;
    auto si_p = sm_ref.get().innerIndexPtr();
    row_indices_int.assign(si_p, si_p + sm_ref.get().nonZeros());
    row_indices.clear();
    row_indices.reserve(row_indices_int.size());
    for (const auto& v : row_indices_int)
      row_indices.push_back(static_cast<T>(v));

    si_p = sm_ref.get().outerIndexPtr();
    column_pointers_int.assign(si_p, si_p + sm_ref.get().outerSize());
    column_pointers.clear();
    column_pointers.reserve(column_pointers_int.size());
    for (const auto& v : column_pointers_int)
      column_pointers.push_back(static_cast<T>(v));
  }
  else
  {
    auto si_p = sm_ref.get().innerIndexPtr();
    row_indices.assign(si_p, si_p + sm_ref.get().nonZeros());

    si_p = sm_ref.get().outerIndexPtr();
    column_pointers.assign(si_p, si_p + sm_ref.get().outerSize());
  }

  // while Eigen does not enforce this, CSC format requires that column
  // pointers ends with the number of non-zero elements
  column_pointers.push_back(sm_ref.get().nonZeros());

  auto csc_v = sm_ref.get().valuePtr();
  values.assign(csc_v, csc_v + sm_ref.get().nonZeros());
}
}
