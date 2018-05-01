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

#include "moveit/lma_kinematics_plugin/chainiksolver_pos_lma_jl_mimic.h"
#include <ros/console.h>

namespace KDL
{
ChainIkSolverPos_LMA_JL_Mimic::ChainIkSolverPos_LMA_JL_Mimic(const Chain& _chain, const JntArray& _q_min,
                                                             const JntArray& _q_max, ChainFkSolverPos& _fksolver,
                                                             ChainIkSolverPos_LMA& _iksolver, unsigned int _maxiter,
                                                             double _eps, bool _position_ik)
  : chain(_chain)
  , q_min(_q_min)
  , q_min_mimic(chain.getNrOfJoints())
  , q_max(_q_max)
  , q_max_mimic(chain.getNrOfJoints())
  , q_temp(chain.getNrOfJoints())
  , fksolver(_fksolver)
  , iksolver(_iksolver)
  , delta_q(_chain.getNrOfJoints())
  , maxiter(_maxiter)
  , eps(_eps)
  , position_ik(_position_ik)
{
  mimic_joints.resize(chain.getNrOfJoints());
  for (std::size_t i = 0; i < mimic_joints.size(); ++i)
  {
    mimic_joints[i].reset(i);
  }
  ROS_DEBUG_NAMED("lma", "Limits");
  for (std::size_t i = 0; i < q_min.rows(); ++i)
  {
    ROS_DEBUG_NAMED("lma", "%ld: Min: %f, Max: %f", long(i), q_min(i), q_max(i));
  }
  ROS_DEBUG_NAMED("lma", " ");
}

bool ChainIkSolverPos_LMA_JL_Mimic::setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& _mimic_joints)
{
  if (_mimic_joints.size() != chain.getNrOfJoints())
  {
    ROS_ERROR_NAMED("lma", "Mimic Joint info should be same size as number of joints in chain: %d",
                    chain.getNrOfJoints());
    return false;
  }

  for (std::size_t i = 0; i < _mimic_joints.size(); ++i)
  {
    if (_mimic_joints[i].map_index >= chain.getNrOfJoints())
    {
      ROS_ERROR_NAMED("lma", "Mimic Joint index should be less than number of joints in chain: %d",
                      chain.getNrOfJoints());
      return false;
    }
  }
  mimic_joints = _mimic_joints;

  // Note that q_min and q_max will be of size chain.getNrOfJoints() - num_mimic_joints
  //  qToqMimic(q_min,q_min_mimic);
  //  qToqMimic(q_max,q_max_mimic);

  ROS_DEBUG_NAMED("lma", "Set mimic joints");
  return true;
}

void ChainIkSolverPos_LMA_JL_Mimic::qToqMimic(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    q_result(i) = mimic_joints[i].offset + mimic_joints[i].multiplier * q(mimic_joints[i].map_index);
  }
}

void ChainIkSolverPos_LMA_JL_Mimic::qMimicToq(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    if (mimic_joints[i].active)  // This is not a mimic joint
    {
      q_result(mimic_joints[i].map_index) = q(i);
    }
  }
}

int ChainIkSolverPos_LMA_JL_Mimic::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
{
  return CartToJntAdvanced(q_init, p_in, q_out, false);
}

void ChainIkSolverPos_LMA_JL_Mimic::harmonize(JntArray& q_out)
{
  for (size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    // Harmonize
    while (q_out(i) > 2 * M_PI)
      q_out(i) -= 2 * M_PI;

    while (q_out(i) < -2 * M_PI)
      q_out(i) += 2 * M_PI;
  }
}

bool ChainIkSolverPos_LMA_JL_Mimic::obeysLimits(const KDL::JntArray& q_out)
{
  bool obeys_limits = true;
  for (size_t i = 0; i < chain.getNrOfJoints(); i++)
  {
    if ((q_out(i) < (q_min(i) - 0.0001)) || (q_out(i) > (q_max(i) + 0.0001)))
    {
      // One element of solution is not within limits
      obeys_limits = false;
      ROS_DEBUG_STREAM_NAMED("lma", "Not in limits! " << i << " value " << q_out(i) << " has limit being  " << q_min(i)
                                                      << " to " << q_max(i));
      break;
    }
  }
  return obeys_limits;
}

int ChainIkSolverPos_LMA_JL_Mimic::CartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out,
                                                     bool lock_redundant_joints)
{
  int ik_valid = iksolver.CartToJnt(q_init, p_in, q_out);
  harmonize(q_out);

  if (!obeysLimits(q_out))
    ik_valid = -4;  // Doesn't obey the joint limits

  return ik_valid;
}

ChainIkSolverPos_LMA_JL_Mimic::~ChainIkSolverPos_LMA_JL_Mimic()
{
}

}  // namespace
