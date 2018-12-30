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
ChainIkSolverVel_pinv_mimic::ChainIkSolverVel_pinv_mimic(const Chain& _chain, int _num_mimic_joints, bool _position_ik,
                                                         double _eps, int _maxiter)
  : chain(_chain)
  , jnt2jac(chain)
  , jac(chain.getNrOfJoints())
  , jac_reduced(chain.getNrOfJoints() - _num_mimic_joints)
  , eps(_eps)
  , maxiter(_maxiter)
  , num_mimic_joints(_num_mimic_joints)
  , position_ik(_position_ik)
{
  mimic_joints_.resize(chain.getNrOfJoints());
  for (std::size_t i = 0; i < mimic_joints_.size(); ++i)
    mimic_joints_[i].reset(i);
}

void ChainIkSolverVel_pinv_mimic::updateInternalDataStructures()
{
  // TODO: move (re)allocation of any internal data structures here
  // to react to changes in chain
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

  // transform v_in to 6D Eigen::Vector
  Matrix<double, 6, 1> vin;
  vin.topRows<3>() = Eigen::Map<const Eigen::Vector3d>(v_in.vel.data, 3);
  vin.bottomRows<3>() = Eigen::Map<const Eigen::Vector3d>(v_in.rot.data, 3);

  const Eigen::MatrixXd& J = jac_reduced.data;
  unsigned int columns = J.cols();
  unsigned int rows = position_ik ? 3 : J.rows();

  // resize is a no-op if size already matches (from last call)
  U.resize(rows, columns);
  S.resize(columns);
  V.resize(columns, columns);
  tmp.resize(columns);

  // Do a singular value decomposition of "jac" with maximum iterations "maxiter",
  // put the results in "U", "S" and "V", jac = U*S*V^t

  int ret;
  ret = svd_eigen_HH(J.topRows(rows), U, S, V, tmp, maxiter);

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv = V * S^-1 * U^t):
  // qdot_out = V * S^-1 * U^t * v_in

  // invert singular values
  for (unsigned int i = 0; i < columns; ++i)
    S(i) = fabs(S(i)) < eps ? 0.0 : 1.0 / S(i);

  // first we calculate S^-1 * U^t * v_in
  tmp = S.array() * (U.transpose() * vin.topRows(rows)).array();
  // pre-multiply tmp with V
  if (num_mimic_joints > 0)
  {
    qdot_out_reduced.noalias() = V * tmp;
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
      qdot_out(i) = qdot_out_reduced[mimic_joints_[i].map_index] * mimic_joints_[i].multiplier;
  }
  else
    qdot_out.data.noalias() = V * tmp;

  // return the return value of the svd decomposition
  return ret;
}
}
