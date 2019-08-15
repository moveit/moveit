#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cstdio>
#include <gtest/gtest.h>
#include <iostream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace sco
{
extern void simplify2(IntVec& inds, DblVec& vals);
}

using namespace sco;

class SolverInterface : public testing::TestWithParam<ModelType>
{
protected:
  SolverInterface() {}
};

TEST(SolverInterface, simplify2)
{
  IntVec indices = { 0, 1, 3 };
  DblVec values = { 1e-7, 1e3, 0., 0., 0. };
  simplify2(indices, values);

  EXPECT_EQ(indices.size(), 2);
  EXPECT_EQ(values.size(), 2);
  EXPECT_TRUE((indices == IntVec{ 0, 1 }));
  EXPECT_TRUE((values == DblVec{ 1e-7, 1e3 }));
}

TEST_P(SolverInterface, setup_problem)
{
  ModelPtr solver = createModel(GetParam());
  VarVector vars;
  for (int i = 0; i < 3; ++i)
  {
    char namebuf[5];
    sprintf(namebuf, "v%i", i);
    vars.push_back(solver->addVar(namebuf));
  }
  solver->update();

  AffExpr aff;
  std::cout << aff << std::endl;
  for (int i = 0; i < 3; ++i)
  {
    exprInc(aff, vars[i]);
    solver->setVarBounds(vars[i], 0, 10);
  }
  aff.constant -= 3;
  std::cout << aff << std::endl;
  QuadExpr affsquared = exprSquare(aff);
  solver->setObjective(affsquared);
  solver->update();
  LOG_INFO("objective: %s", CSTR(affsquared));
  LOG_INFO("please manually check that /tmp/solver-interface-test.lp matches this");
  solver->writeToFile("/tmp/solver-interface-test.lp");

  solver->optimize();

  DblVec soln(3);
  for (int i = 0; i < 3; ++i)
  {
    soln[i] = solver->getVarValue(vars[i]);
  }
  EXPECT_NEAR(aff.value(soln), 0, 1e-6);

  solver->removeVar(vars[2]);
  solver->update();
  EXPECT_EQ(solver->getVars().size(), 2);
}

// Tests multiplying larger terms
TEST_P(SolverInterface, DISABLED_ExprMult_test1)  // QuadExpr not PSD
{
  ModelPtr solver = createModel(GetParam());
  VarVector vars;
  for (int i = 0; i < 3; ++i)
  {
    char namebuf[5];
    sprintf(namebuf, "v%i", i);
    vars.push_back(solver->addVar(namebuf));
  }
  for (int i = 3; i < 6; ++i)
  {
    char namebuf[5];
    sprintf(namebuf, "v%i", i);
    vars.push_back(solver->addVar(namebuf));
  }
  solver->update();

  AffExpr aff1;
  std::cout << aff1 << std::endl;
  for (int i = 0; i < 3; ++i)
  {
    exprInc(aff1, vars[i]);
    solver->setVarBounds(vars[i], 0, 10);
  }
  aff1.constant -= 3;
  std::cout << aff1 << std::endl;

  AffExpr aff2;
  std::cout << aff2 << std::endl;
  for (int i = 3; i < 6; ++i)
  {
    exprInc(aff2, vars[i]);
    solver->setVarBounds(vars[i], 0, 10);
  }
  aff2.constant -= 5;

  std::cout << aff2 << std::endl;
  QuadExpr aff12 = exprMult(aff1, aff2);
  solver->setObjective(aff12);
  solver->update();
  LOG_INFO("objective: %s", CSTR(aff12));
  LOG_INFO("please manually check that /tmp/solver-interface-test.lp matches this");
  solver->writeToFile("/tmp/solver-interface-test.lp");

  solver->optimize();

  DblVec soln(3);
  for (int i = 0; i < 3; ++i)
  {
    soln[i] = solver->getVarValue(vars[i]);
  }
  EXPECT_NEAR(aff1.value(soln), 0, 1e-6);

  solver->removeVar(vars[2]);
  solver->update();
  EXPECT_EQ(solver->getVars().size(), 5);
}

// Tests multiplication with 2 variables: v1=10, v2=20 => (2*v1)(v2) = 400
TEST_P(SolverInterface, ExprMult_test2)
{
  const double v1_val = 10;
  const double v2_val = 20;
  const double v1_coeff = 2;
  const double v2_coeff = 1;
  const double aff1_const = 0;
  const double aff2_const = 0;

  ModelPtr solver = createModel(GetParam());
  VarVector vars;
  vars.push_back(solver->addVar("v1"));
  vars.push_back(solver->addVar("v2"));

  solver->update();

  AffExpr aff1;
  AffExpr aff2;

  exprInc(aff1, vars[0]);
  solver->setVarBounds(vars[0], v1_val, v1_val);
  aff1.constant = aff1_const;
  aff1.coeffs[0] = v1_coeff;

  exprInc(aff2, vars[1]);
  solver->setVarBounds(vars[1], v2_val, v2_val);
  aff2.constant = aff2_const;
  aff2.coeffs[0] = v2_coeff;

  std::cout << "aff1: " << aff1 << std::endl;
  std::cout << "aff2: " << aff2 << std::endl;
  QuadExpr aff12 = exprMult(aff1, aff2);
  solver->setObjective(aff12);
  solver->update();
  LOG_INFO("objective: %s", CSTR(aff12));
  LOG_INFO("please manually check that /tmp/solver-interface-test.lp matches this");
  solver->writeToFile("/tmp/solver-interface-test.lp");

  solver->optimize();

  DblVec soln(2);
  for (int i = 0; i < 2; ++i)
  {
    soln[i] = solver->getVarValue(vars[i]);
    std::cout << soln[i] << std::endl;
  }
  std::cout << "Result: " << aff12.value(soln) << std::endl;
  double answer = (v1_coeff * v1_val + aff1_const) * (v2_coeff * v2_val + aff2_const);
  EXPECT_NEAR(aff12.value(soln), answer, 1e-6);
}

// Tests multiplication with a constant: v1=10, v2=20 => (3*v1-3)(2*v2-5) = 945
TEST_P(SolverInterface, ExprMult_test3)
{
  const double v1_val = 10;
  const double v2_val = 20;
  const double v1_coeff = 3;
  const double v2_coeff = 2;
  const double aff1_const = -3;
  const double aff2_const = -5;

  ModelPtr solver = createModel(GetParam());
  VarVector vars;
  vars.push_back(solver->addVar("v1"));
  vars.push_back(solver->addVar("v2"));

  solver->update();

  AffExpr aff1;
  AffExpr aff2;

  exprInc(aff1, vars[0]);
  solver->setVarBounds(vars[0], v1_val, v1_val);
  aff1.constant = aff1_const;
  aff1.coeffs[0] = v1_coeff;

  exprInc(aff2, vars[1]);
  solver->setVarBounds(vars[1], v2_val, v2_val);
  aff2.constant = aff2_const;
  aff2.coeffs[0] = v2_coeff;

  std::cout << "aff1: " << aff1 << std::endl;
  std::cout << "aff2: " << aff2 << std::endl;
  QuadExpr aff12 = exprMult(aff1, aff2);
  solver->setObjective(aff12);
  solver->update();
  LOG_INFO("objective: %s", CSTR(aff12));
  LOG_INFO("please manually check that /tmp/solver-interface-test.lp matches this");
  solver->writeToFile("/tmp/solver-interface-test.lp");

  solver->optimize();

  DblVec soln(2);
  for (int i = 0; i < 2; ++i)
  {
    soln[i] = solver->getVarValue(vars[i]);
    std::cout << soln[i] << std::endl;
  }
  std::cout << "Result: " << aff12.value(soln) << std::endl;
  double answer = (v1_coeff * v1_val + aff1_const) * (v2_coeff * v2_val + aff2_const);
  EXPECT_NEAR(aff12.value(soln), answer, 1e-6);
}

INSTANTIATE_TEST_CASE_P(AllSolvers, SolverInterface, testing::ValuesIn(availableSolvers()));
