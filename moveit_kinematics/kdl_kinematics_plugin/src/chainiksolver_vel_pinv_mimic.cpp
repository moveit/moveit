// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified to account for "mimic" joints, i.e. joints whose motion has a
// linear relationship to that of another joint.
// Copyright  (C)  2013  Sachin Chitta, Willow Garage

#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_pinv_mimic.hpp>
#include <ros/console.h>

namespace KDL
{
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& _chain, int _num_mimic_joints, int _num_redundant_joints,
                                                         const Eigen::VectorXd& _cartesian_weights,
                                                         bool _position_ik, double _eps, int _maxiter)
  : chain(_chain)
  , jnt2jac(chain)
  , cartesian_weights(_cartesian_weights)
  , jac(chain.getNrOfJoints())
  , jac_reduced(chain.getNrOfJoints() - _num_mimic_joints)
  , jac_locked(chain.getNrOfJoints() - _num_redundant_joints - _num_mimic_joints)
  , eps(_eps)
  , maxiter(_maxiter)
  , num_mimic_joints(_num_mimic_joints)
  , position_ik(_position_ik)
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

bool ChainIkSolverVel_pinv_mimic::setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints)
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

bool ChainIkSolverVel_pinv_mimic::weightJac(Eigen::MatrixXd& jac)
{
  for (int j=0; j < 6; ++j)
      jac.row(j) *= cartesian_weights[j];
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

// compute q_out = W_j * (W_x * J * W_j)^# * W_x * v_in
// where W_j and W_x are joint and task-level weights
int ChainIkSolverVel_pinv_mimic::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
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

  // transform v_in to 6D Eigen::Vector and apply weigthing
  Matrix<double, 6, 1> vin;
  vin.topRows<3>() = Eigen::Map<const Eigen::Vector3d>(v_in.vel.data, 3);
  vin.bottomRows<3>() = Eigen::Map<const Eigen::Vector3d>(v_in.rot.data, 3);
  vin.array() *= cartesian_weights.array();

  // Remove columns of locked redundant joints from Jacobian
  bool locked = (redundant_joints_locked && num_redundant_joints > 0);
  if (locked)
    jacToJacLocked(jac_reduced, jac_locked);

  // use jac_reduced or jac_locked in the following
  const Eigen::MatrixXd& J = locked ? jac_locked.data : jac_reduced.data;
  weightJac(const_cast<Eigen::MatrixXd&>(J));

  unsigned int columns = J.cols();
  unsigned int rows = position_ik ? 3 : J.rows();

  // resize is a no-op if size already matches (from last call)
  U.resize(rows, columns);
  S.resize(columns);
  V.resize(columns, columns);
  tmp.resize(columns);

  // Do a singular value decomposition of "jac" with maximum
  // iterations "maxiter", put the results in "U", "S" and "V"
  // jac = U*S*Vt

  int ret;
  ret = svd_eigen_HH(J.topRows(rows), U, S, V, tmp, maxiter);

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv = V * S^-1 * U^t):
  // qdot_out = V * S^-1 * U^t * v_in

  // invert singular values
  for (int i = 0; i < columns; i++)
  {
    S(i) = fabs(S(i)) < eps ? 0.0 : 1.0 / S(i);
  }
  // first we calculate S^-1 * U^t * v_in
  tmp = S.array() * (U.transpose() * vin.topRows(rows)).array();
  // pre-multiply tmp with V
  qdot_out.data.block(0,0, V.rows(),1) = V * tmp;

  ROS_DEBUG_STREAM_NAMED("kdl", "Solution:");
  if (num_mimic_joints > 0)
  {
    for (int i = columns; i < chain.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  // return the return value of the svd decomposition
  return ret;
}
}
