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

#ifndef KDL_CHAIN_IKSOLVERVEL_PINV_MIMIC_H
#define KDL_CHAIN_IKSOLVERVEL_PINV_MIMIC_H

#include "kdl/chainiksolver.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/utilities/svd_HH.hpp"
#include "kdl/utilities/svd_eigen_HH.hpp"

#include <moveit/lma_kinematics_plugin/joint_mimic.h>

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

  virtual int CartToJntRedundant(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

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
  bool setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& _mimic_joints);

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
    redundant_joints_locked = true;
  }

  void unlockRedundantJoints()
  {
    redundant_joints_locked = false;
  }

private:
  bool jacToJacReduced(const Jacobian& jac, Jacobian& jac_mimic);
  bool jacToJacLocked(const Jacobian& jac, Jacobian& jac_locked);

  const Chain chain;
  ChainJntToJacSolver jnt2jac;

  // This set of variables are all used in the default case, i.e. where we are solving for the
  // full end-effector pose
  Jacobian jac;
  std::vector<JntArray> U;
  JntArray S;
  std::vector<JntArray> V;
  JntArray tmp;

  // This is the "reduced" jacobian, i.e. the contributions of the mimic joints have been mapped onto
  // the active DOFs here
  Jacobian jac_reduced;
  JntArray qdot_out_reduced;

  // This is the set of variable used when solving for position only inverse kinematics
  Eigen::MatrixXd U_translate;
  Eigen::VectorXd S_translate;
  Eigen::MatrixXd V_translate;
  Eigen::VectorXd tmp_translate;

  // This is the jacobian when the redundant joint is "locked" and plays no part
  Jacobian jac_locked;
  JntArray qdot_out_reduced_locked, qdot_out_locked;

  SVD_HH svd;
  double eps;
  int maxiter;

  // Mimic joint specific
  std::vector<lma_kinematics_plugin::JointMimic> mimic_joints_;
  int num_mimic_joints;

  bool position_ik;

  // This is the set of variable used when solving for inverse kinematics
  // for the case where the redundant joint is "locked" and plays no part
  Eigen::MatrixXd U_locked;
  Eigen::VectorXd S_locked;
  Eigen::MatrixXd V_locked;
  Eigen::VectorXd tmp_locked;

  // This is the set of variable used when solving for position only inverse kinematics
  // for the case where the redundant joint is "locked" and plays no part
  Eigen::MatrixXd U_translate_locked;
  Eigen::VectorXd S_translate_locked;
  Eigen::MatrixXd V_translate_locked;
  Eigen::VectorXd tmp_translate_locked;

  // Internal storage for a map from the "locked" state to the full active state
  std::vector<unsigned int> locked_joints_map_index;
  unsigned int num_redundant_joints;
  bool redundant_joints_locked;
};
}
#endif
