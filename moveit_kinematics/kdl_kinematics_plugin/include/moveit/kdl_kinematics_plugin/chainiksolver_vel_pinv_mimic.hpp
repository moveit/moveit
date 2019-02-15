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

#include "kdl/chainiksolver.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/utilities/svd_HH.hpp"
#include "kdl/utilities/svd_eigen_HH.hpp"

#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>

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
   * @param chain the chain to calculate the inverse velocity
   * kinematics for
   * @param num_mimic_joints The number of joints that are setup to
   * follow other joints
   * @param num_redundant_joints The number of redundant dofs
   * @param position_ik false if you want to solve for the full 6 dof
   * end-effector pose, true if you want to solve only for the 3 dof
   * end-effector position.
   * @param eps if a singular value is below this value, its
   * inverse is set to zero, default: 0.00001
   * @param maxiter maximum iterations for the svd calculation,
   * default: 150
   */
  explicit ChainIkSolverVel_pinv_mimic(const Chain& chain, int num_mimic_joints = 0, int num_redundant_joints = 0,
                                       bool position_ik = false, double eps = 0.00001, int maxiter = 150);

  ~ChainIkSolverVel_pinv_mimic();

  virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

  virtual int cartToJntRedundant(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

  /**
   * not (yet) implemented.
   *
   */
  virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out)
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
  bool setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints);

  /**
   * @brief Set a mapping between a reduced set of joints (numbering either 6 or 3) and the full set of active (i.e
   * excluding the mimic joints) DOFs in the robot.
   * As an example, consider an arm with 7 joints: j0 to j6. If j2 represents the redundancy, then
   * redundant_joints_map_index
   * will be a 6 dimensional vector - [0 1 3 4 5 6],
   * i.e. joint_value_full(redundant_joints_map_index[i]) = joint_value_reduced(i), for i=0,...5
   */
  bool setRedundantJointsMapIndex(const std::vector<unsigned int>& redundant_joints_map_index);

  void lockRedundantJoints()
  {
    redundant_joints_locked_ = true;
  }

  void unlockRedundantJoints()
  {
    redundant_joints_locked_ = false;
  }

private:
  bool jacToJacReduced(const Jacobian& jac, Jacobian& jac_reduced_l);
  bool jacToJacLocked(const Jacobian& jac, Jacobian& jac_locked);

  const Chain chain_;
  ChainJntToJacSolver jnt2jac_;

  // This set of variables are all used in the default case, i.e. where we are solving for the
  // full end-effector pose
  Jacobian jac_;
  std::vector<JntArray> U_;
  JntArray S_;
  std::vector<JntArray> V_;
  JntArray tmp_;

  // This is the "reduced" jacobian, i.e. the contributions of the mimic joints have been mapped onto
  // the active DOFs here
  Jacobian jac_reduced_;
  JntArray qdot_out_reduced_;

  // This is the set of variable used when solving for position only inverse kinematics
  Eigen::MatrixXd U_translate_;
  Eigen::VectorXd S_translate_;
  Eigen::MatrixXd V_translate_;
  Eigen::VectorXd tmp_translate_;

  // This is the jacobian when the redundant joint is "locked" and plays no part
  Jacobian jac_locked_;
  JntArray qdot_out_reduced_locked_, qdot_out_locked_;

  SVD_HH svd_;
  double eps_;
  int maxiter_;

  // Mimic joint specific
  std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints_;
  int num_mimic_joints_;

  bool position_ik_;

  // This is the set of variable used when solving for inverse kinematics
  // for the case where the redundant joint is "locked" and plays no part
  Eigen::MatrixXd U_locked_;
  Eigen::VectorXd S_locked_;
  Eigen::MatrixXd V_locked_;
  Eigen::VectorXd tmp_locked_;

  // This is the set of variable used when solving for position only inverse kinematics
  // for the case where the redundant joint is "locked" and plays no part
  Eigen::MatrixXd U_translate_locked_;
  Eigen::VectorXd S_translate_locked_;
  Eigen::MatrixXd V_translate_locked_;
  Eigen::VectorXd tmp_translate_locked_;

  // Internal storage for a map from the "locked" state to the full active state
  std::vector<unsigned int> locked_joints_map_index_;
  unsigned int num_redundant_joints_;
  bool redundant_joints_locked_;
};
}
#endif
