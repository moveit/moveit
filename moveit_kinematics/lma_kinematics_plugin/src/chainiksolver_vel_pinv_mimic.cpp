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
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& chain, int num_mimic_joints,
                                                         int num_redundant_joints, bool position_ik, double eps,
                                                         int maxiter)
  : chain_(chain)
  , jnt2jac_(chain_)
  , jac_(chain_.getNrOfJoints())
  , U_(6, JntArray(chain_.getNrOfJoints() - num_mimic_joints))
  , S_(chain_.getNrOfJoints() - num_mimic_joints)
  , V_(chain_.getNrOfJoints() - num_mimic_joints, JntArray(chain_.getNrOfJoints() - num_mimic_joints))
  , tmp_(chain_.getNrOfJoints() - num_mimic_joints)
  , jac_reduced_(chain_.getNrOfJoints() - num_mimic_joints)
  , qdot_out_reduced_(chain_.getNrOfJoints() - num_mimic_joints)
  , U_translate_(MatrixXd::Zero(3, chain_.getNrOfJoints() - num_mimic_joints))
  , S_translate_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints))
  , V_translate_(MatrixXd::Zero(chain_.getNrOfJoints() - num_mimic_joints, chain_.getNrOfJoints() - num_mimic_joints))
  , tmp_translate_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints))
  , jac_locked_(chain_.getNrOfJoints() - num_redundant_joints - num_mimic_joints)
  , qdot_out_reduced_locked_(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints)
  , qdot_out_locked_(chain_.getNrOfJoints() - num_redundant_joints)
  , svd_(jac_reduced_)
  , eps_(eps)
  , maxiter_(maxiter)
  , num_mimic_joints_(num_mimic_joints)
  , position_ik_(position_ik)
  , U_locked_(MatrixXd::Zero(6, chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , S_locked_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , V_locked_(MatrixXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints,
                             chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , tmp_locked_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , U_translate_locked_(MatrixXd::Zero(3, chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , S_translate_locked_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , V_translate_locked_(MatrixXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints,
                                       chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , tmp_translate_locked_(VectorXd::Zero(chain_.getNrOfJoints() - num_mimic_joints - num_redundant_joints))
  , num_redundant_joints_(num_redundant_joints)
  , redundant_joints_locked_(false)
{
  mimic_joints_.resize(chain_.getNrOfJoints());
  for (std::size_t i = 0; i < mimic_joints_.size(); ++i)
    mimic_joints_[i].reset(i);
}

ChainIkSolverVel_pinv_mimic::~ChainIkSolverVel_pinv_mimic() = default;

bool ChainIkSolverVel_pinv_mimic::setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& mimic_joints)
{
  if (mimic_joints.size() != chain_.getNrOfJoints())
    return false;

  for (const lma_kinematics_plugin::JointMimic& mimic_joint : mimic_joints)
  {
    if (mimic_joint.map_index >= chain_.getNrOfJoints())
      return false;
  }
  mimic_joints_ = mimic_joints;
  return true;
}

bool ChainIkSolverVel_pinv_mimic::setRedundantJointsMapIndex(
    const std::vector<unsigned int>& redundant_joints_map_index)
{
  if (redundant_joints_map_index.size() != chain_.getNrOfJoints() - num_mimic_joints_ - num_redundant_joints_)
  {
    ROS_ERROR("Map index size: %d does not match expected size. "
              "No. of joints: %d, num_mimic_joints_: %d, num_redundant_joints_: %d",
              (int)redundant_joints_map_index.size(), (int)chain_.getNrOfJoints(), (int)num_mimic_joints_,
              (int)num_redundant_joints_);
    return false;
  }

  for (unsigned int i : redundant_joints_map_index)
  {
    if (i >= chain_.getNrOfJoints() - num_mimic_joints_)
      return false;
  }
  locked_joints_map_index_ = redundant_joints_map_index;
  return true;
}

bool ChainIkSolverVel_pinv_mimic::jacToJacReduced(const Jacobian& jac, Jacobian& jac_reduced_l)
{
  jac_reduced_l.data.setZero();
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
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
  for (std::size_t i = 0; i < chain_.getNrOfJoints() - num_mimic_joints_ - num_redundant_joints_; ++i)
  {
    jac_locked.setColumn(i, jac.getColumn(locked_joints_map_index_[i]));
  }
  return true;
}

int ChainIkSolverVel_pinv_mimic::cartToJntRedundant(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  qdot_out.data.setZero();
  // Let the ChainJntToJacSolver calculate the jacobian "jac_" for
  // the current joint positions "q_in". This will include the mimic joints
  if (num_mimic_joints_ > 0)
  {
    jnt2jac_.JntToJac(q_in, jac_);
    // Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac_, jac_reduced_);
  }
  else
    jnt2jac_.JntToJac(q_in, jac_reduced_);

  // Now compute the jacobian with redundant joints locked
  jacToJacLocked(jac_reduced_, jac_locked_);

  // Do a singular value decomposition of "jac_" with maximum
  // iterations "maxiter_", put the results in "U_", "S_" and "V_"
  // jac_ = U_*S_*Vt

  int ret;
  if (!position_ik_)
    ret = svd_eigen_HH(jac_locked_.data, U_locked_, S_locked_, V_locked_, tmp_locked_, maxiter_);
  else
    ret = svd_eigen_HH(
        jac_locked_.data.topLeftCorner(3, chain_.getNrOfJoints() - num_mimic_joints_ - num_redundant_joints_),
        U_translate_locked_, S_translate_locked_, V_translate_locked_, tmp_translate_locked_, maxiter_);

  double sum;
  unsigned int i, j;

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V_*S_pinv*Ut):
  // qdot_out = V_*S_pinv*Ut*v_in

  unsigned int rows;
  if (!position_ik_)
    rows = jac_locked_.rows();
  else
    rows = 3;

  // first we calculate Ut*v_in
  for (i = 0; i < jac_locked_.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < rows; j++)
    {
      if (!position_ik_)
        sum += U_locked_(j, i) * v_in(j);
      else
        sum += U_translate_locked_(j, i) * v_in(j);
    }
    // If the singular value is too small (<eps_), don't invert it but
    // set the inverted singular value to zero (truncated svd)
    if (!position_ik_)
      tmp_(i) = sum * (fabs(S_locked_(i)) < eps_ ? 0.0 : 1.0 / S_locked_(i));
    else
      tmp_(i) = sum * (fabs(S_translate_locked_(i)) < eps_ ? 0.0 : 1.0 / S_translate_locked_(i));
  }
  // tmp_ is now: tmp_=S_pinv*Ut*v_in, we still have to premultiply
  // it with V_ to get qdot_out
  for (i = 0; i < jac_locked_.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < jac_locked_.columns(); j++)
    {
      if (!position_ik_)
        sum += V_locked_(i, j) * tmp_(j);
      else
        sum += V_translate_locked_(i, j) * tmp_(j);
    }
    // Put the result in qdot_out_reduced_ if mimic joints exist, otherwise in qdot_out
    if (num_mimic_joints_ > 0)
      qdot_out_reduced_locked_(i) = sum;
    else
      qdot_out_locked_(i) = sum;
  }
  ROS_DEBUG_STREAM_NAMED("lma", "Solution:");

  if (num_mimic_joints_ > 0)
  {
    for (i = 0; i < chain_.getNrOfJoints() - num_mimic_joints_ - num_redundant_joints_; ++i)
    {
      qdot_out_reduced_(locked_joints_map_index_[i]) = qdot_out_reduced_locked_(i);
    }
    for (i = 0; i < chain_.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced_(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  else
  {
    for (i = 0; i < chain_.getNrOfJoints() - num_redundant_joints_; ++i)
    {
      qdot_out(locked_joints_map_index_[i]) = qdot_out_locked_(i);
    }
  }
  // Reset the flag
  // redundant_joints_locked_ = false;
  // return the return value of the svd decomposition
  return ret;
}

int ChainIkSolverVel_pinv_mimic::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  if (redundant_joints_locked_)
    return cartToJntRedundant(q_in, v_in, qdot_out);

  // Let the ChainJntToJacSolver calculate the jacobian "jac_" for
  // the current joint positions "q_in". This will include the mimic joints
  if (num_mimic_joints_ > 0)
  {
    jnt2jac_.JntToJac(q_in, jac_);
    // Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac_, jac_reduced_);
  }
  else
    jnt2jac_.JntToJac(q_in, jac_reduced_);

  // Do a singular value decomposition of "jac_" with maximum
  // iterations "maxiter_", put the results in "U_", "S_" and "V_"
  // jac_ = U_*S_*Vt

  int ret;
  if (!position_ik_)
    ret = svd_.calculate(jac_reduced_, U_, S_, V_, maxiter_);
  else
    ret = svd_eigen_HH(jac_reduced_.data.topLeftCorner(3, chain_.getNrOfJoints() - num_mimic_joints_), U_translate_,
                       S_translate_, V_translate_, tmp_translate_, maxiter_);

  double sum;
  unsigned int i, j;

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V_*S_pinv*Ut):
  // qdot_out = V_*S_pinv*Ut*v_in

  unsigned int rows;
  if (!position_ik_)
    rows = jac_reduced_.rows();
  else
    rows = 3;

  // first we calculate Ut*v_in
  for (i = 0; i < jac_reduced_.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < rows; j++)
    {
      if (!position_ik_)
        sum += U_[j](i) * v_in(j);
      else
        sum += U_translate_(j, i) * v_in(j);
    }
    // If the singular value is too small (<eps_), don't invert it but
    // set the inverted singular value to zero (truncated svd)
    if (!position_ik_)
      tmp_(i) = sum * (fabs(S_(i)) < eps_ ? 0.0 : 1.0 / S_(i));
    else
      tmp_(i) = sum * (fabs(S_translate_(i)) < eps_ ? 0.0 : 1.0 / S_translate_(i));
  }
  // tmp_ is now: tmp_=S_pinv*Ut*v_in, we still have to premultiply
  // it with V_ to get qdot_out
  for (i = 0; i < jac_reduced_.columns(); i++)
  {
    sum = 0.0;
    for (j = 0; j < jac_reduced_.columns(); j++)
    {
      if (!position_ik_)
        sum += V_[i](j) * tmp_(j);
      else
        sum += V_translate_(i, j) * tmp_(j);
    }
    // Put the result in qdot_out_reduced_ if mimic joints exist, otherwise in qdot_out
    if (num_mimic_joints_ > 0)
      qdot_out_reduced_(i) = sum;
    else
      qdot_out(i) = sum;
  }
  ROS_DEBUG_STREAM_NAMED("lma", "Solution:");
  if (num_mimic_joints_ > 0)
  {
    for (i = 0; i < chain_.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced_(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  // return the return value of the svd decomposition
  return ret;
}
}  // namespace KDL
