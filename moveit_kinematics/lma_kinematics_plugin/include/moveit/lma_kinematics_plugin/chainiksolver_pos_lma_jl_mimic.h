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

#ifndef KDLCHAINIKSOLVERPOS_LMA_JL_MIMIC_H
#define KDLCHAINIKSOLVERPOS_LMA_JL_MIMIC_H

#include "kdl/chainiksolverpos_lma.hpp"  // Solver for the inverse position kinematics that uses Levenberg-Marquardt.
#include "kdl/chainfksolver.hpp"

#include <moveit/lma_kinematics_plugin/joint_mimic.h>

namespace KDL
{
/**
 * Implementation of a general inverse position kinematics
 * algorithm based on Levenberg-Marquardt method to calculate the
 * position transformation from Cartesian to joint space of a general
 * KDL::Chain. Takes joint limits into account.
 *
 * @ingroup KinematicFamily
 */
class ChainIkSolverPos_LMA_JL_Mimic : public ChainIkSolverPos
{
public:
  /**
   * Constructor of the solver, it needs the chain, a forward
   * position kinematics solver and an inverse velocity
   * kinematics solver for that chain.
   *
   * @param chain the chain to calculate the inverse position for
   * @param q_max the maximum joint positions
   * @param q_min the minimum joint positions
   * @param fksolver a forward position kinematics solver
   * @param iksolver an inverse velocity kinematics solver
   * @param maxiter the maximum Levenberg-Marquardt iterations,
   * default: 100
   * @param eps the precision for the position, used to end the
   * iterations, default: epsilon (defined in kdl.hpp)
   *
   * @return
   */
  ChainIkSolverPos_LMA_JL_Mimic(const Chain& chain, const JntArray& q_min, const JntArray& q_max,
                                ChainFkSolverPos& fksolver, ChainIkSolverPos_LMA& iksolver, unsigned int maxiter = 100,
                                double eps = 1e-6, bool position_ik = false);

  ~ChainIkSolverPos_LMA_JL_Mimic();

  virtual int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out);

  virtual int CartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out, bool lock_redundant_joints);

  bool setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& mimic_joints);

private:
  const Chain chain;
  JntArray q_min;        // These are the limits for the "reduced" state consisting of only active DOFs
  JntArray q_min_mimic;  // These are the limits for the full state
  JntArray q_max;        // These are the limits for the "reduced" state consisting of only active DOFs
  JntArray q_max_mimic;  // These are the limits for the full state
  JntArray q_temp;
  ChainFkSolverPos& fksolver;
  ChainIkSolverPos_LMA& iksolver;
  JntArray delta_q;
  Frame f;
  Twist delta_twist;
  unsigned int maxiter;
  double eps;
  std::vector<lma_kinematics_plugin::JointMimic> mimic_joints;
  void qToqMimic(const JntArray& q,
                 JntArray& q_result);  // Convert from the "reduced" state (only active DOFs) to the "full" state
  void qMimicToq(const JntArray& q, JntArray& q_result);  // Convert from the "full" state to the "reduced" state
  void harmonize(JntArray& q_out);                        // Puts the angles within [-2PI, 2PI]
  bool obeysLimits(const KDL::JntArray& q_out);           // Checks that a set of joint angles obey the urdf limits
  bool position_ik;
};
}

#endif
