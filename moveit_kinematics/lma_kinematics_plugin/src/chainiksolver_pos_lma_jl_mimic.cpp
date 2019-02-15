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
ChainIkSolverPos_LMA_JL_Mimic::ChainIkSolverPos_LMA_JL_Mimic(const Chain& chain, const JntArray& q_min,
                                                             const JntArray& q_max, ChainFkSolverPos& fksolver,
                                                             ChainIkSolverPos_LMA& iksolver, unsigned int maxiter,
                                                             double eps, bool position_ik)
  : chain_(chain)
  , q_min_(q_min)
  , q_min_mimic_(chain_.getNrOfJoints())
  , q_max_(q_max)
  , q_max_mimic_(chain_.getNrOfJoints())
  , q_temp_(chain_.getNrOfJoints())
  , fksolver_(fksolver)
  , iksolver_(iksolver)
  , delta_q_(chain.getNrOfJoints())
  , maxiter_(maxiter)
  , eps_(eps)
  , position_ik_(position_ik)
{
  mimic_joints_.resize(chain_.getNrOfJoints());
  for (std::size_t i = 0; i < mimic_joints_.size(); ++i)
  {
    mimic_joints_[i].reset(i);
  }
  ROS_DEBUG_NAMED("lma", "Limits");
  for (std::size_t i = 0; i < q_min_.rows(); ++i)
  {
    ROS_DEBUG_NAMED("lma", "%ld: Min: %f, Max: %f", long(i), q_min_(i), q_max_(i));
  }
  ROS_DEBUG_NAMED("lma", " ");
}

bool ChainIkSolverPos_LMA_JL_Mimic::setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& mimic_joints)
{
  if (mimic_joints.size() != chain_.getNrOfJoints())
  {
    ROS_ERROR_NAMED("lma", "Mimic Joint info should be same size as number of joints in chain_: %d",
                    chain_.getNrOfJoints());
    return false;
  }

  for (const lma_kinematics_plugin::JointMimic& _mimic_joint : mimic_joints)
  {
    if (_mimic_joint.map_index >= chain_.getNrOfJoints())
    {
      ROS_ERROR_NAMED("lma", "Mimic Joint index should be less than number of joints in chain_: %d",
                      chain_.getNrOfJoints());
      return false;
    }
  }
  mimic_joints_ = mimic_joints;

  // Note that q_min_ and q_max_ will be of size chain_.getNrOfJoints() - num_mimic_joints
  //  qToqMimic(q_min_,q_min_mimic_);
  //  qToqMimic(q_max_,q_max_mimic_);

  ROS_DEBUG_NAMED("lma", "Set mimic joints");
  return true;
}

void ChainIkSolverPos_LMA_JL_Mimic::qToqMimic(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    q_result(i) = mimic_joints_[i].offset + mimic_joints_[i].multiplier * q(mimic_joints_[i].map_index);
  }
}

void ChainIkSolverPos_LMA_JL_Mimic::qMimicToq(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    if (mimic_joints_[i].active)  // This is not a mimic joint
    {
      q_result(mimic_joints_[i].map_index) = q(i);
    }
  }
}

int ChainIkSolverPos_LMA_JL_Mimic::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
{
  return cartToJntAdvanced(q_init, p_in, q_out, false);
}

void ChainIkSolverPos_LMA_JL_Mimic::harmonize(JntArray& q_out)
{
  for (size_t i = 0; i < chain_.getNrOfJoints(); ++i)
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
  for (size_t i = 0; i < chain_.getNrOfJoints(); i++)
  {
    if ((q_out(i) < (q_min_(i) - 0.0001)) || (q_out(i) > (q_max_(i) + 0.0001)))
    {
      // One element of solution is not within limits
      obeys_limits = false;
      ROS_DEBUG_STREAM_NAMED("lma", "Not in limits! " << i << " value " << q_out(i) << " has limit being  " << q_min_(i)
                                                      << " to " << q_max_(i));
      break;
    }
  }
  return obeys_limits;
}

int ChainIkSolverPos_LMA_JL_Mimic::cartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out,
                                                     bool lock_redundant_joints)
{
  int ik_valid = iksolver_.CartToJnt(q_init, p_in, q_out);
  harmonize(q_out);

  if (!obeysLimits(q_out))
    ik_valid = -4;  // Doesn't obey the joint limits

  return ik_valid;
}

ChainIkSolverPos_LMA_JL_Mimic::~ChainIkSolverPos_LMA_JL_Mimic()
= default;

}  // namespace KDL
