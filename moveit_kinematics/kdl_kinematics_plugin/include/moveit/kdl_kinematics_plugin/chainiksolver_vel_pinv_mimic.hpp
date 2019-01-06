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

#ifndef KDL_CHAIN_IKSOLVERVEL_PINV_Mimic_HPP
#define KDL_CHAIN_IKSOLVERVEL_PINV_Mimic_HPP

#include <kdl/config.h>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <Eigen/SVD>

namespace KDL
{
/**
 * Implementation of a inverse velocity kinematics algorithm based
 * on the generalize pseudo inverse to calculate the velocity
 * transformation from Cartesian to joint space of a general
 * KDL::Chain. It uses a svd-calculation based on householders
 * rotations.
 *
 * @ingroup KinematicFamily
 */
class ChainIkSolverVel_pinv_mimic : public ChainIkSolverVel
{
public:
  /**
   * Constructor of the solver
   *
   * @param chain the chain to calculate the inverse velocity kinematics for
   * @param num_mimic_joints The number of joints that are setup to follow other joints
   * @param position_ik false if you want to solve for the full 6 dof end-effector pose,
   *        true if you want to solve only for the 3 dof end-effector position.
   * @param eps if a singular value is below this value, its inverse is set to zero, default: 0.00001
   */
  explicit ChainIkSolverVel_pinv_mimic(const Chain& chain, int num_mimic_joints = 0, bool position_ik = false,
                                       double eps = 0.00001);

// TODO: simplify after kinetic support is dropped
#define KDL_VERSION_LESS(a, b, c) (KDL_VERSION < ((a << 16) | (b << 8) | c))
#if KDL_VERSION_LESS(1, 4, 0)
  void updateInternalDataStructures();
#else
  void updateInternalDataStructures() override;
#endif
#undef KDL_VERSION_LESS

  ~ChainIkSolverVel_pinv_mimic() override;

  int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out) override;

  /**
   * not (yet) implemented.
   *
   */
  int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out) override
  {
    return -1;
  };

  /**
   * @brief Set a vector of indices that map each (and every) joint in the chain onto the corresponding joint in a
   * reduced set of joints that do not include the mimic joints. This vector must be of size chain.getNrOfJoints().
   * E.g. if an arm has 7 joints: j0 to j6. Say j2 mimics (follows) j0. Then, mimic_joints should be:
   * [0 1 0 3 4 5 6]
   * @param mimic_joints Vector of size chain.getNrOfJoints() that maps every joint in the chain onto (a) itself
   * if its not a mimic joint or (b) onto the active dof that it is mimicking
   */
  bool setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& _mimic_joints);

private:
  bool jacToJacReduced(const Jacobian& jac, Jacobian& jac_mimic);

  const Chain chain;
  ChainJntToJacSolver jnt2jac;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  Eigen::VectorXd qdot_out_reduced;

  Jacobian jac;          // full Jacobian
  Jacobian jac_reduced;  // reduced Jacobian with contributions of mimic joints mapped onto active DoFs

  // Mimic joint specific
  std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints_;
  int num_mimic_joints;

  bool position_ik;
};
}
#endif
