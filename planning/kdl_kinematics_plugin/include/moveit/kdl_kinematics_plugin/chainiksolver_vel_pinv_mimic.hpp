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
   * @param eps if a singular value is below this value, its
   * inverse is set to zero, default: 0.00001
   * @param maxiter maximum iterations for the svd calculation,
   * default: 150
   *
   */
  explicit ChainIkSolverVel_pinv_mimic(const Chain& chain,int num_mimic_joints =0, bool position_ik=false, double eps=0.00001, int maxiter=150);

  ~ChainIkSolverVel_pinv_mimic();
  
  virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

  /**
   * not (yet) implemented.
   *
   */
  virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out){return -1;};

  bool setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic> &_mimic_joints);

private:
  
  bool jacToJacReduced(const Jacobian &jac, Jacobian &jac_mimic);
  const Chain chain;
  ChainJntToJacSolver jnt2jac;
  Jacobian jac;
  Jacobian jac_reduced;
  SVD_HH svd;
  std::vector<JntArray> U;
  JntArray S;
  std::vector<JntArray> V;
  JntArray tmp;
  double eps;
  int maxiter;
  std::vector<unsigned int> mimic_column;
  JntArray qdot_out_reduced;  
  std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints_;
  Eigen::MatrixXd task_space_weight;  
  int num_mimic_joints;
  bool position_ik;  
  Eigen::MatrixXd U_translate;
  Eigen::VectorXd S_translate;
  Eigen::MatrixXd V_translate;
  Eigen::VectorXd tmp_translate;
};
}
#endif

