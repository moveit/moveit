// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

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

#include "moveit/kdl_kinematics_plugin/chainiksolver_pos_nr_jl_mimic.hpp"
#include <ros/console.h>

namespace KDL
{
ChainIkSolverPos_NR_JL_Mimic::ChainIkSolverPos_NR_JL_Mimic(const Chain& chain, const JntArray& q_min,
                                                           const JntArray& q_max, ChainFkSolverPos& fksolver,
                                                           ChainIkSolverVel& iksolver, unsigned int maxiter, double eps,
                                                           bool position_ik)
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
  ROS_DEBUG_NAMED("kdl", "Limits");
  for (std::size_t i = 0; i < q_min_.rows(); ++i)
  {
    ROS_DEBUG_NAMED("kdl", "%ld: Min: %f, Max: %f", long(i), q_min_(i), q_max_(i));
  }
  ROS_DEBUG_NAMED("kdl", " ");
}

bool ChainIkSolverPos_NR_JL_Mimic::setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints)
{
  if (mimic_joints.size() != chain_.getNrOfJoints())
  {
    ROS_ERROR_NAMED("kdl", "Mimic Joint info should be same size as number of joints in chain_: %d",
                    chain_.getNrOfJoints());
    return false;
  }

  for (const kdl_kinematics_plugin::JointMimic& mimic_joint : mimic_joints)
  {
    if (mimic_joint.map_index >= chain_.getNrOfJoints())
    {
      ROS_ERROR_NAMED("kdl", "Mimic Joint index should be less than number of joints in chain_: %d",
                      chain_.getNrOfJoints());
      return false;
    }
  }
  mimic_joints_ = mimic_joints;

  // Note that q_min_ and q_max_ will be of size chain_.getNrOfJoints() - num_mimic_joints
  //  qToqMimic(q_min_,q_min_mimic_);
  //  qToqMimic(q_max_,q_max_mimic_);

  ROS_DEBUG_NAMED("kdl", "Set mimic joints");
  return true;
}

void ChainIkSolverPos_NR_JL_Mimic::qToqMimic(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    q_result(i) = mimic_joints_[i].offset + mimic_joints_[i].multiplier * q(mimic_joints_[i].map_index);
  }
}

void ChainIkSolverPos_NR_JL_Mimic::qMimicToq(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    if (mimic_joints_[i].active)  // This is not a mimic joint
    {
      q_result(mimic_joints_[i].map_index) = q(i);
    }
  }
}

int ChainIkSolverPos_NR_JL_Mimic::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
{
  return cartToJntAdvanced(q_init, p_in, q_out, false);
}

int ChainIkSolverPos_NR_JL_Mimic::cartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out,
                                                    bool lock_redundant_joints)
{
  //  Note that q_init and q_out will be of size chain_.getNrOfJoints()
  //  qToqMimic(q_init,q_temp_);

  q_temp_ = q_init;
  ROS_DEBUG_STREAM_NAMED("kdl", "Input:");
  for (std::size_t i = 0; i < q_out.rows(); ++i)
    ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_out(i));

  unsigned int i;
  for (i = 0; i < maxiter_; ++i)
  {
    fksolver_.JntToCart(q_temp_, f_);
    delta_twist_ = diff(f_, p_in);

    if (position_ik_)
    {
      if (fabs(delta_twist_(0)) < eps_ && fabs(delta_twist_(1)) < eps_ && fabs(delta_twist_(2)) < eps_)
        break;
    }
    else
    {
      if (Equal(delta_twist_, Twist::Zero(), eps_))
        break;
    }

    ROS_DEBUG_STREAM_NAMED("kdl", "delta_twist_");
    for (std::size_t i = 0; i < 6; ++i)
      ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, delta_twist_(i));

    iksolver_.CartToJnt(q_temp_, delta_twist_, delta_q_);

    Add(q_temp_, delta_q_, q_temp_);

    ROS_DEBUG_STREAM_NAMED("kdl", "delta_q_");
    for (std::size_t i = 0; i < delta_q_.rows(); ++i)
      ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, delta_q_(i));

    ROS_DEBUG_STREAM_NAMED("kdl", "q_temp_");
    for (std::size_t i = 0; i < q_temp_.rows(); ++i)
      ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_temp_(i));

    for (std::size_t j = 0; j < q_min_.rows(); ++j)
    {
      //      if(mimic_joints_[j].active)
      if (q_temp_(j) < q_min_(j))
        q_temp_(j) = q_min_(j);
    }
    for (std::size_t j = 0; j < q_max_.rows(); ++j)
    {
      //      if(mimic_joints_[j].active)
      if (q_temp_(j) > q_max_(j))
        q_temp_(j) = q_max_(j);
    }

    //    q_out = q_temp_;
    // Make sure limits are applied on the mimic joints to
    //    qMimicToq(q_temp_,q_out);
    //    qToqMimic(q_out,q_temp_);
  }

  //  qMimicToq(q_temp_, q_out);
  q_out = q_temp_;
  ROS_DEBUG_STREAM_NAMED("kdl", "Full Solution:");
  for (std::size_t i = 0; i < q_temp_.rows(); ++i)
    ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_temp_(i));

  ROS_DEBUG_STREAM_NAMED("kdl", "Actual Solution:");
  for (std::size_t i = 0; i < q_out.rows(); ++i)
    ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_out(i));

  if (i != maxiter_)
    return 0;
  else
    return -3;
}

ChainIkSolverPos_NR_JL_Mimic::~ChainIkSolverPos_NR_JL_Mimic() = default;
}  // namespace KDL
