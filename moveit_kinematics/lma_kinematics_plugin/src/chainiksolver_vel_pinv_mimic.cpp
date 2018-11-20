/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CRI group, NTU, Singapore
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
 *   * Neither the name of CRI group nor the names of its
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

/* Author: Francisco Suarez-Ruiz */

#include <moveit/lma_kinematics_plugin/chainiksolver_vel_pinv_mimic.h>
#include <ros/console.h>

namespace KDL
{
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& _chain, int _num_mimic_joints,
                                                         int _num_redundant_joints, bool _position_ik, double _eps,
                                                         int _maxiter)
  : chain(_chain)
  , jnt2jac(chain)
  , jac(chain.getNrOfJoints())
  , U(6, JntArray(chain.getNrOfJoints() - _num_mimic_joints))
  , S(chain.getNrOfJoints() - _num_mimic_joints)
  , V(chain.getNrOfJoints() - _num_mimic_joints, JntArray(chain.getNrOfJoints() - _num_mimic_joints))
  , tmp(chain.getNrOfJoints() - _num_mimic_joints)
  , jac_reduced(chain.getNrOfJoints() - _num_mimic_joints)
  , qdot_out_reduced(chain.getNrOfJoints() - _num_mimic_joints)
  , U_translate(MatrixXd::Zero(3, chain.getNrOfJoints() - _num_mimic_joints))
  , S_translate(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints))
  , V_translate(MatrixXd::Zero(chain.getNrOfJoints() - _num_mimic_joints, chain.getNrOfJoints() - _num_mimic_joints))
  , tmp_translate(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints))
  , jac_locked(chain.getNrOfJoints() - _num_redundant_joints - _num_mimic_joints)
  , qdot_out_reduced_locked(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints)
  , qdot_out_locked(chain.getNrOfJoints() - _num_redundant_joints)
  , svd(jac_reduced)
  , eps(_eps)
  , maxiter(_maxiter)
  , num_mimic_joints(_num_mimic_joints)
  , position_ik(_position_ik)
  , U_locked(MatrixXd::Zero(6, chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , S_locked(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , V_locked(MatrixXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints,
                            chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , tmp_locked(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , U_translate_locked(MatrixXd::Zero(3, chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , S_translate_locked(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , V_translate_locked(MatrixXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints,
                                      chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , tmp_translate_locked(VectorXd::Zero(chain.getNrOfJoints() - _num_mimic_joints - _num_redundant_joints))
  , num_redundant_joints(_num_redundant_joints)
  , redundant_joints_locked(false)
{
  mimic_joints_.resize(chain.getNrOfJoints());
  for (std::size_t i = 0; i < mimic_joints_.size(); ++i)
    mimic_joints_[i].reset(i);
}

ChainIkSolverVel_pinv_mimic::~ChainIkSolverVel_pinv_mimic()
{
}

bool ChainIkSolverVel_pinv_mimic::setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& mimic_joints)
{
  if (mimic_joints.size() != chain.getNrOfJoints())
    return false;

  for (std::size_t i = 0; i < mimic_joints.size(); ++i)
  {
    if (mimic_joints[i].map_index >= chain.getNrOfJoints())
      return false;
  }
  mimic_joints_ = mimic_joints;
  return true;
}

bool ChainIkSolverVel_pinv_mimic::setRedundantJointsMapIndex(
    const std::vector<unsigned int>& redundant_joints_map_index)
{
  if (redundant_joints_map_index.size() != chain.getNrOfJoints() - num_mimic_joints - num_redundant_joints)
  {
    ROS_ERROR("Map index size: %d does not match expected size. "
              "No. of joints: %d, num_mimic_joints: %d, num_redundant_joints: %d",
              (int)redundant_joints_map_index.size(), (int)chain.getNrOfJoints(), (int)num_mimic_joints,
              (int)num_redundant_joints);
    return false;
  }

  for (std::size_t i = 0; i < redundant_joints_map_index.size(); ++i)
  {
    if (redundant_joints_map_index[i] >= chain.getNrOfJoints() - num_mimic_joints)
      return false;
  }
  locked_joints_map_index = redundant_joints_map_index;
  return true;
}

bool ChainIkSolverVel_pinv_mimic::jacToJacReduced(const Jacobian& jac, Jacobian& jac_reduced_l)
{
  jac_reduced_l.data.setZero();
  for (std::size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    Twist vel1 = jac_reduced_l.getColumn(mimic_joints_[i].map_index);
    Twist vel2 = jac.getColumn(i);
    Twist result = vel1 + (mimic_joints_[i].multiplier * vel2);
    jac_reduced_l.setColumn(mimic_joints_[i].map_index, result);
  }
  return true;
}

bool ChainIkSolverVel_pinv_mimic::jacToJacLocked(const Jacobian& jac, Jacobian& jac_locked)
{
  jac_locked.data.setZero();
  for (std::size_t i = 0; i < chain.getNrOfJoints() - num_mimic_joints - num_redundant_joints; ++i)
  {
    jac_locked.setColumn(i, jac.getColumn(locked_joints_map_index[i]));
  }
  return true;
}

int ChainIkSolverVel_pinv_mimic::CartToJntRedundant(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  qdot_out.data.setZero();
  // Let the ChainJntToJacSolver calculate the jacobian "jac" for
  // the current joint positions "q_in". This will include the mimic joints
  if (num_mimic_joints > 0)
  {
    jnt2jac.JntToJac(q_in, jac);
    // Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac, jac_reduced);
  }
  else
    jnt2jac.JntToJac(q_in, jac_reduced);

  // Now compute the jacobian with redundant joints locked
  jacToJacLocked(jac_reduced, jac_locked);

  // Do a singular value decomposition of "jac" with maximum
  // iterations "maxiter", put the results in "U", "S" and "V"
  // jac = U*S*Vt

  int ret;
  if (!position_ik)
    ret = svd_eigen_HH(jac_locked.data, U_locked, S_locked, V_locked, tmp_locked, maxiter);
  else
    ret =
        svd_eigen_HH(jac_locked.data.topLeftCorner(3, chain.getNrOfJoints() - num_mimic_joints - num_redundant_joints),
                     U_translate_locked, S_translate_locked, V_translate_locked, tmp_translate_locked, maxiter);

  double sum;
  unsigned int i, j;

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*v_in

  unsigned int rows;
  if (!position_ik)
    rows = jac_locked.rows();
  else
    rows = 3;

  // first we calculate Ut*v_in
  for (i = 0; i < jac_locked.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < rows; j++)
    {
      if (!position_ik)
        sum += U_locked(j, i) * v_in(j);
      else
        sum += U_translate_locked(j, i) * v_in(j);
    }
    // If the singular value is too small (<eps), don't invert it but
    // set the inverted singular value to zero (truncated svd)
    if (!position_ik)
      tmp(i) = sum * (fabs(S_locked(i)) < eps ? 0.0 : 1.0 / S_locked(i));
    else
      tmp(i) = sum * (fabs(S_translate_locked(i)) < eps ? 0.0 : 1.0 / S_translate_locked(i));
  }
  // tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
  // it with V to get qdot_out
  for (i = 0; i < jac_locked.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < jac_locked.columns(); j++)
    {
      if (!position_ik)
        sum += V_locked(i, j) * tmp(j);
      else
        sum += V_translate_locked(i, j) * tmp(j);
    }
    // Put the result in qdot_out_reduced if mimic joints exist, otherwise in qdot_out
    if (num_mimic_joints > 0)
      qdot_out_reduced_locked(i) = sum;
    else
      qdot_out_locked(i) = sum;
  }
  ROS_DEBUG_STREAM_NAMED("lma", "Solution:");

  if (num_mimic_joints > 0)
  {
    for (i = 0; i < chain.getNrOfJoints() - num_mimic_joints - num_redundant_joints; ++i)
    {
      qdot_out_reduced(locked_joints_map_index[i]) = qdot_out_reduced_locked(i);
    }
    for (i = 0; i < chain.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  else
  {
    for (i = 0; i < chain.getNrOfJoints() - num_redundant_joints; ++i)
    {
      qdot_out(locked_joints_map_index[i]) = qdot_out_locked(i);
    }
  }
  // Reset the flag
  // redundant_joints_locked = false;
  // return the return value of the svd decomposition
  return ret;
}

int ChainIkSolverVel_pinv_mimic::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  if (redundant_joints_locked)
    return CartToJntRedundant(q_in, v_in, qdot_out);

  // Let the ChainJntToJacSolver calculate the jacobian "jac" for
  // the current joint positions "q_in". This will include the mimic joints
  if (num_mimic_joints > 0)
  {
    jnt2jac.JntToJac(q_in, jac);
    // Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac, jac_reduced);
  }
  else
    jnt2jac.JntToJac(q_in, jac_reduced);

  // Do a singular value decomposition of "jac" with maximum
  // iterations "maxiter", put the results in "U", "S" and "V"
  // jac = U*S*Vt

  int ret;
  if (!position_ik)
    ret = svd.calculate(jac_reduced, U, S, V, maxiter);
  else
    ret = svd_eigen_HH(jac_reduced.data.topLeftCorner(3, chain.getNrOfJoints() - num_mimic_joints), U_translate,
                       S_translate, V_translate, tmp_translate, maxiter);

  double sum;
  unsigned int i, j;

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*v_in

  unsigned int rows;
  if (!position_ik)
    rows = jac_reduced.rows();
  else
    rows = 3;

  // first we calculate Ut*v_in
  for (i = 0; i < jac_reduced.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < rows; j++)
    {
      if (!position_ik)
        sum += U[j](i) * v_in(j);
      else
        sum += U_translate(j, i) * v_in(j);
    }
    // If the singular value is too small (<eps), don't invert it but
    // set the inverted singular value to zero (truncated svd)
    if (!position_ik)
      tmp(i) = sum * (fabs(S(i)) < eps ? 0.0 : 1.0 / S(i));
    else
      tmp(i) = sum * (fabs(S_translate(i)) < eps ? 0.0 : 1.0 / S_translate(i));
  }
  // tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
  // it with V to get qdot_out
  for (i = 0; i < jac_reduced.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < jac_reduced.columns(); j++)
    {
      if (!position_ik)
        sum += V[i](j) * tmp(j);
      else
        sum += V_translate(i, j) * tmp(j);
    }
    // Put the result in qdot_out_reduced if mimic joints exist, otherwise in qdot_out
    if (num_mimic_joints > 0)
      qdot_out_reduced(i) = sum;
    else
      qdot_out(i) = sum;
  }
  ROS_DEBUG_STREAM_NAMED("lma", "Solution:");
  if (num_mimic_joints > 0)
  {
    for (i = 0; i < chain.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  // return the return value of the svd decomposition
  return ret;
}
}
