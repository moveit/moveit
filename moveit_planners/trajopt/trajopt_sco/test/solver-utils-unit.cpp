#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cstdio>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace sco;

TEST(solver_utils, exprToEigen)
{
  int n_vars = 2;
  std::vector<VarRepPtr> x_info;
  VarVector x;
  for (int i = 0; i < n_vars; ++i)
  {
    std::stringstream var_name;
    var_name << "x_" << i;
    VarRepPtr x_el(new VarRep(i, var_name.str(), nullptr));
    x_info.push_back(x_el);
    x.push_back(Var(x_el.get()));
  }

  // x_affine = [3, 2]*x + 1
  AffExpr x_affine;
  x_affine.vars = x;
  x_affine.coeffs = DblVec{ 3, 2 };
  x_affine.constant = 1;

  std::cout << "x_affine=  " << x_affine << std::endl;
  std::cout << "expecting A = [3, 2];" << std::endl << "          u = [1]" << std::endl;
  Eigen::MatrixXd m_A_expected(1, n_vars);
  m_A_expected << 3, 2;
  Eigen::VectorXd v_u_expected(1);
  v_u_expected << -1;

  Eigen::SparseVector<double> v_A;
  exprToEigen(x_affine, v_A, n_vars);
  ASSERT_EQ(v_A.size(), m_A_expected.cols());
  Eigen::VectorXd v_A_d = v_A;
  Eigen::VectorXd m_A_r = m_A_expected.row(0);
  EXPECT_TRUE(v_A_d.isApprox(m_A_r)) << "error converting x_affine"
                                     << " to Eigen::SparseVector. "
                                     << "v_A :" << std::endl
                                     << v_A << std::endl;

  Eigen::SparseMatrix<double> m_A;
  Eigen::VectorXd v_u;
  AffExprVector x_affine_vector(1, x_affine);
  exprToEigen(x_affine_vector, m_A, v_u, n_vars);
  ASSERT_EQ(v_u_expected.size(), v_u.size());
  EXPECT_TRUE(v_u_expected == v_u) << "v_u_expected != v_u" << std::endl << "v_u:" << std::endl << v_u << std::endl;
  EXPECT_EQ(m_A.nonZeros(), 2) << "m_A.nonZeros() != 2" << std::endl;
  ASSERT_EQ(m_A.rows(), m_A_expected.rows());
  ASSERT_EQ(m_A.cols(), m_A_expected.cols());
  EXPECT_TRUE(m_A.isApprox(m_A_expected)) << "error converting x_affine to "
                                          << "Eigen::SparseMatrix. m_A :" << std::endl
                                          << m_A << std::endl;

  QuadExpr x_squared = exprSquare(x_affine);
  std::cout << "x_squared= " << x_squared << std::endl;
  std::cout << "expecting Q = [9, 6;" << std::endl
            << "               6, 4]" << std::endl
            << "          q = [6, 4]" << std::endl;
  Eigen::MatrixXd m_Q_expected(2, 2);
  m_Q_expected << 9, 6, 6, 4;
  DblVec v_q_expected{ 6, 4 };

  Eigen::SparseMatrix<double> m_Q;
  Eigen::VectorXd v_q;
  exprToEigen(x_squared, m_Q, v_q, n_vars);
  {
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> v_q_e_eig(v_q_expected.data(), v_q_expected.size());
    ASSERT_EQ(v_q.size(), v_q_e_eig.size());
    EXPECT_TRUE(v_q_e_eig.isApprox(v_q)) << "v_q_expected != v_q" << std::endl
                                         << "v_q:" << std::endl
                                         << v_q << std::endl;
  }
  EXPECT_TRUE(m_Q.isApprox(m_Q_expected)) << "error converting x_squared to "
                                          << "Eigen::SparseMatrix. m_Q :" << std::endl
                                          << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 4) << "m_Q.nonZeros() != 4" << std::endl;

  exprToEigen(x_squared, m_Q, v_q, n_vars, true);
  {
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> v_q_e_eig(v_q_expected.data(), v_q_expected.size());
    EXPECT_TRUE(v_q_e_eig.isApprox(v_q)) << "v_q_expected != v_q" << std::endl
                                         << "v_q:" << std::endl
                                         << v_q << std::endl;
  }
  EXPECT_TRUE(m_Q.isApprox(2 * m_Q_expected)) << "error converting x_squared to"
                                              << " Eigen::SparseMatrix. m_Q :" << std::endl
                                              << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 4) << "m_Q.nonZeros() != 4" << std::endl;

  x_affine.coeffs = DblVec{ 0, 2 };
  std::cout << "x_affine=  " << x_affine << std::endl;
  std::cout << "expecting A = [0, 2];" << std::endl;
  x_squared = exprSquare(x_affine);
  std::cout << "x_squared= " << x_squared << std::endl;
  std::cout << "expecting Q = [0, 0;" << std::endl << "               0, 4]" << std::endl;
  m_Q_expected.setZero();
  m_Q_expected << 0, 0, 0, 4;
  exprToEigen(x_squared, m_Q, v_q, n_vars, false, false);
  EXPECT_TRUE(m_Q.isApprox(m_Q_expected)) << "error converting x_squared to "
                                          << "Eigen::SparseMatrix. m_Q :" << std::endl
                                          << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 1) << "m_Q.nonZeros() != 1" << std::endl;

  exprToEigen(x_squared, m_Q, v_q, n_vars, true, false);
  EXPECT_TRUE(m_Q.isApprox(2 * m_Q_expected)) << "error converting x_squared to"
                                              << " Eigen::SparseMatrix. m_Q :" << std::endl
                                              << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 1) << "m_Q.nonZeros() != 1" << std::endl;

  exprToEigen(x_squared, m_Q, v_q, n_vars, false, true);
  EXPECT_TRUE(m_Q.isApprox(m_Q_expected)) << "error converting x_squared to "
                                          << "Eigen::SparseMatrix. m_Q :" << std::endl
                                          << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 2) << "m_Q.nonZeros() != 2" << std::endl;

  exprToEigen(x_squared, m_Q, v_q, n_vars, true, true);
  EXPECT_TRUE(m_Q.isApprox(2 * m_Q_expected)) << "error converting x_squared to"
                                              << "Eigen::SparseMatrix. m_Q :" << std::endl
                                              << m_Q << std::endl;
  EXPECT_EQ(m_Q.nonZeros(), 2) << "m_Q.nonZeros() != 2" << std::endl;
}

TEST(solver_utils, eigenToTriplets)
{
  Eigen::MatrixXd m_Q(2, 2);
  m_Q << 9, 0, 6, 4;
  Eigen::SparseMatrix<double> m_Q_sparse_expected = m_Q.sparseView();
  IntVec m_Q_i, m_Q_j;
  DblVec m_Q_ij;
  eigenToTriplets(m_Q_sparse_expected, m_Q_i, m_Q_j, m_Q_ij);
  Eigen::SparseMatrix<double> m_Q_sparse(2, 2);
  tripletsToEigen(m_Q_i, m_Q_j, m_Q_ij, m_Q_sparse);
  EXPECT_TRUE(m_Q_sparse.isApprox(m_Q)) << "m_Q != m_Q_sparse when converting "
                                        << "m_Q -> triplets -> m_Q_sparse. m_Q:" << std::endl
                                        << m_Q << std::endl;
  EXPECT_EQ(m_Q_sparse.nonZeros(), 3) << "m_Q.nonZeros() != 3" << std::endl;
}

TEST(solver_utils, eigenToCSC)
{
  DblVec P;
  IntVec rows_i;
  IntVec cols_p;
  {
    /*
     * M = [ 1, 2, 3,
     *       1, 0, 9,
     *       1, 8, 0]
     */
    Eigen::MatrixXd M(3, 3);
    M << 1, 2, 3, 1, 0, 9, 1, 8, 0;
    Eigen::SparseMatrix<double> Ms = M.sparseView();

    eigenToCSC(Ms, rows_i, cols_p, P);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{ 1, 1, 1, 2, 8, 3, 9 })) << "bad P:\n" << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{ 0, 1, 2, 0, 2, 0, 1 })) << "bad rows_i:\n" << CSTR(rows_i);
    EXPECT_TRUE((cols_p == IntVec{ 0, 3, 5, 7 })) << "cols_p not in "
                                                  << "CRC form:\n"
                                                  << CSTR(cols_p);
  }
  {
    /*
     * M = [ 0, 2, 0,
     *       7, 0, 0,
     *       0, 0, 0]
     */
    Eigen::SparseMatrix<double> M(3, 3);
    M.coeffRef(0, 1) = 2;
    M.coeffRef(1, 0) = 7;

    eigenToCSC(M, rows_i, cols_p, P);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{ 7, 2 })) << "bad P:\n" << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{ 1, 0 })) << "rows_i != data_j:\n"
                                            << CSTR(rows_i) << " vs\n"
                                            << CSTR((IntVec{ 1, 0 }));
    EXPECT_TRUE((cols_p == IntVec{ 0, 1, 2, 2 })) << "cols_p not in "
                                                  << "CRC form:\n"
                                                  << CSTR(cols_p);

    std::vector<long long int> rows_i_ll, cols_p_ll;
    std::vector<long long int> rows_i_ll_exp{ 1, 0 };
    std::vector<long long int> cols_p_ll_exp{ 0, 1, 2, 2 };

    eigenToCSC(M, rows_i_ll, cols_p_ll, P);
    EXPECT_TRUE(rows_i_ll.size() == P.size()) << "rows_i_ll.size() != P.size()";
    EXPECT_TRUE((rows_i_ll == rows_i_ll_exp));
    EXPECT_TRUE((cols_p_ll == cols_p_ll_exp));

    std::vector<unsigned long long int> rows_i_ull, cols_p_ull;
    std::vector<unsigned long long int> rows_i_ull_exp{ 1, 0 };
    std::vector<unsigned long long int> cols_p_ull_exp{ 0, 1, 2, 2 };
    eigenToCSC(M, rows_i_ull, cols_p_ull, P);
    EXPECT_TRUE(rows_i_ull.size() == P.size()) << "rows_i_ll.size() != P.size()";
    EXPECT_TRUE((rows_i_ull == rows_i_ull_exp));
    EXPECT_TRUE((cols_p_ull == cols_p_ull_exp));
  }
  {
    /*
     * M = [ 0, 0, 0,
     *       0, 0, 0,
     *       0, 6, 0]
     */
    Eigen::SparseMatrix<double> M(3, 3);
    M.coeffRef(2, 1) = 6;

    eigenToCSC(M, rows_i, cols_p, P);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{ 6 })) << "bad P:\n" << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{ 2 })) << "rows_i != data_j:\n" << CSTR(rows_i) << " vs\n" << CSTR((IntVec{ 1, 0 }));
    EXPECT_TRUE((cols_p == IntVec{ 0, 0, 1, 1 })) << "cols_p not in "
                                                  << "CRC form:\n"
                                                  << CSTR(cols_p);
  }
}

TEST(solver_utils, eigenToCSC_upper_triangular)
{
  /*
   * M = [ 1, 2, 0,
   *       2, 4, 0,
   *       0, 0, 9]
   */
  Eigen::MatrixXd M(3, 3);
  M << 1, 2, 0, 2, 4, 0, 0, 0, 9;
  Eigen::SparseMatrix<double> Ms = M.sparseView();

  DblVec P;
  IntVec rows_i;
  IntVec cols_p;
  eigenToCSC<Eigen::Upper>(Ms, rows_i, cols_p, P);

  EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
  EXPECT_TRUE((P == DblVec{ 1, 2, 4, 9 })) << "bad P:\n" << CSTR(P);
  EXPECT_TRUE((rows_i == IntVec{ 0, 0, 1, 2 })) << "rows_i != expected"
                                                << ":\n"
                                                << CSTR(rows_i) << " vs\n"
                                                << CSTR((IntVec{ 0, 0, 1, 2 }));
  EXPECT_TRUE((cols_p == IntVec{ 0, 1, 3, 4 })) << "cols_p not in "
                                                << "CRC form:\n"
                                                << CSTR(cols_p);
}
