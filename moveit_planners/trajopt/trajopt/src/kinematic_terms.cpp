/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari */

#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>

#include "trajopt/kinematic_terms.h"

using namespace std;
using namespace sco;
using namespace Eigen;

namespace trajopt
{
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  // TODO: create the actual error function from information in planning scene
  VectorXd err;
  return err;
}

VectorXd JointVelErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  // var_vals = (theta_t1, theta_t2, theta_t3 ... 1/dt_1, 1/dt_2, 1/dt_3 ...)
  int half = static_cast<int>(var_vals.rows() / 2);
  int num_vels = half - 1;
  // (x1-x0)*(1/dt)
  VectorXd vel = (var_vals.segment(1, num_vels) - var_vals.segment(0, num_vels)).array() *
                 var_vals.segment(half + 1, num_vels).array();

  // Note that for equality terms tols are 0, so error is effectively doubled
  VectorXd result(vel.rows() * 2);
  result.topRows(vel.rows()) = -(upper_tol_ - (vel.array() - target_));
  result.bottomRows(vel.rows()) = lower_tol_ - (vel.array() - target_);
  return result;
}

MatrixXd JointVelJacobianCalculator::operator()(const VectorXd& var_vals) const
{
  // var_vals = (theta_t1, theta_t2, theta_t3 ... 1/dt_1, 1/dt_2, 1/dt_3 ...)
  int num_vals = static_cast<int>(var_vals.rows());
  int half = num_vals / 2;
  int num_vels = half - 1;
  MatrixXd jac = MatrixXd::Zero(num_vels * 2, num_vals);

  for (int i = 0; i < num_vels; i++)
  {
    // v = (j_i+1 - j_i)*(1/dt)
    // We calculate v with the dt from the second pt
    int time_index = i + half + 1;
    jac(i, i) = -1.0 * var_vals(time_index);
    jac(i, i + 1) = 1.0 * var_vals(time_index);
    jac(i, time_index) = var_vals(i + 1) - var_vals(i);
    // All others are 0
  }

  // bottom half is negative velocities
  jac.bottomRows(num_vels) = -jac.topRows(num_vels);

  return jac;
}

}  // namespace trajopt
