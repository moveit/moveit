/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <gtest/gtest.h>
#include <trajectory_processing/trajectory_processing_utils.h>
#include <stdlib.h>

static double getRandomNumber(double min, double max)
{
  return ((double)rand() / RAND_MAX)*(max-min) + min;
}

TEST(TestUtils, testTridiagonalSolver)
{
  // seed the random number generator:
  srand(2);

  // generate a random tridiagonal matrix:
  int n=10;

  std::vector<double> a(n);
  std::vector<double> b(n);
  std::vector<double> c(n);
  std::vector<double> d(n);
  std::vector<double> x(n);
  std::vector<double> solved_x(n);

  for (int i=0; i<n; i++)
  {
    a[i] = getRandomNumber(10.0, 20.0);
    b[i] = getRandomNumber(1.0, 4.0);
    c[i] = getRandomNumber(1.0, 4.0);
    x[i] = getRandomNumber(-1.0, 1.0);
  }
  a[0] = 0.0;
  c[n-1] = 0.0;

  // multiply to get values for d:
  for (int i=1; i<n-1; i++)
  {
    d[i] = a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1];
  }
  d[0] = b[0]*x[0] + c[0]*x[1];
  d[n-1] = a[n-1]*x[n-2] + b[n-1]*x[n-1];

  // solve it:
  trajectory_processing::tridiagonalSolve(a, b, c, d, solved_x);

  double tolerance = 1e-8;

  // check it:
  for (int i=0; i<n; i++)
  {
    EXPECT_NEAR(x[i], solved_x[i], tolerance);
  }
}

TEST(TestUtils, testTridiagonalSolver2)
{
  // seed the random number generator:
  srand(2);

  // generate a tridiagonal matrix for cubic splines
  int n=10;

  std::vector<double> a(n);
  std::vector<double> b(n);
  std::vector<double> c(n);
  std::vector<double> d(n);
  std::vector<double> x(n);
  std::vector<double> solved_x(n);

  for (int i=0; i<n; i++)
  {
    a[i] = 4.0;
    b[i] = 1.0;
    c[i] = 1.0;
    x[i] = getRandomNumber(-1.0, 1.0);
  }
  a[0] = 0.0;
  c[n-1] = 0.0;
  b[0] = 2.0;
  b[n-1] = 2.0;

  // multiply to get values for d:
  for (int i=1; i<n-1; i++)
  {
    d[i] = a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1];
  }
  d[0] = b[0]*x[0] + c[0]*x[1];
  d[n-1] = a[n-1]*x[n-2] + b[n-1]*x[n-1];

  // solve it:
  trajectory_processing::tridiagonalSolve(a, b, c, d, solved_x);

  double tolerance = 1e-8;

  // check it:
  for (int i=0; i<n; i++)
  {
    EXPECT_NEAR(x[i], solved_x[i], tolerance);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_utils");
  return RUN_ALL_TESTS();
}
