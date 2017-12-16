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
ChainIkSolverPos_NR_JL_Mimic::ChainIkSolverPos_NR_JL_Mimic(const Chain& _chain, const JntArray& _q_min,
                                                           const JntArray& _q_max, ChainFkSolverPos& _fksolver,
                                                           ChainIkSolverVel& _iksolver, unsigned int _maxiter,
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
  ROS_DEBUG_NAMED("kdl", "Limits");
  for (std::size_t i = 0; i < q_min.rows(); ++i)
  {
    ROS_DEBUG_NAMED("kdl", "%ld: Min: %f, Max: %f", long(i), q_min(i), q_max(i));
  }
  ROS_DEBUG_NAMED("kdl", " ");
}

bool ChainIkSolverPos_NR_JL_Mimic::setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& _mimic_joints)
{
  if (_mimic_joints.size() != chain.getNrOfJoints())
  {
    ROS_ERROR_NAMED("kdl", "Mimic Joint info should be same size as number of joints in chain: %d",
                    chain.getNrOfJoints());
    return false;
  }

  for (std::size_t i = 0; i < _mimic_joints.size(); ++i)
  {
    if (_mimic_joints[i].map_index >= chain.getNrOfJoints())
    {
      ROS_ERROR_NAMED("kdl", "Mimic Joint index should be less than number of joints in chain: %d",
                      chain.getNrOfJoints());
      return false;
    }
  }
  mimic_joints = _mimic_joints;

  // Note that q_min and q_max will be of size chain.getNrOfJoints() - num_mimic_joints
  //  qToqMimic(q_min,q_min_mimic);
  //  qToqMimic(q_max,q_max_mimic);

  ROS_DEBUG_NAMED("kdl", "Set mimic joints");
  return true;
}

void ChainIkSolverPos_NR_JL_Mimic::qToqMimic(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    q_result(i) = mimic_joints[i].offset + mimic_joints[i].multiplier * q(mimic_joints[i].map_index);
  }
}

void ChainIkSolverPos_NR_JL_Mimic::qMimicToq(const JntArray& q, JntArray& q_result)
{
  for (std::size_t i = 0; i < chain.getNrOfJoints(); ++i)
  {
    if (mimic_joints[i].active)  // This is not a mimic joint
    {
      q_result(mimic_joints[i].map_index) = q(i);
    }
  }
}

int ChainIkSolverPos_NR_JL_Mimic::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
{
  //  Note that q_init and q_out will be of size chain.getNrOfJoints()
  double last_delta_twist_err = DBL_MAX;
  double step_size = 1.0;
  q_out = q_init;
  ROS_DEBUG_STREAM_NAMED("kdl", "Input:");
  for (std::size_t i = 0; i < q_out.rows(); ++i)
    ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_init(i));

  unsigned int i;
  bool success = false;
  for (i = 0; i < maxiter; ++i)
  {
    fksolver.JntToCart(q_out, f);
    delta_twist = diff(f, p_in);

    // check norms of position and orientation errors
    double position_error = delta_twist.vel.Norm();
    double orientation_error = position_ik ? 0 : delta_twist.rot.Norm();
    double delta_twist_err = std::max(position_error, orientation_error);
    if (delta_twist_err <= eps)
    {
      success = true;
      break;
    }

    if (delta_twist_err >= last_delta_twist_err)
    {
      // if the error increased, we are close to a singularity -> reduce step size
      double old_step_size = step_size;
      step_size *= 0.5;  // reduce scale;
      Multiply(delta_q, step_size / old_step_size, delta_q);
      ROS_INFO_NAMED("kdl", "error increased: %f -> %f, scale: %f", last_delta_twist_err, delta_twist_err, step_size);
      q_out = q_temp;  // restore previous unclipped joint values
      if (step_size < eps)
        break;  // cannot reach target
    }
    else
    {
      q_temp = q_out;  // remember joint values of last successful step
      last_delta_twist_err = delta_twist_err;

      delta_twist = delta_twist * step_size;
      iksolver.CartToJnt(q_out, delta_twist, delta_q);
      double delta_q_norm = delta_q.data.array().abs().sum();

      ROS_INFO_NAMED("kdl", "pos err: %f  rot err: %f  delta_q: %f", position_error, orientation_error, delta_q_norm);
      for (std::size_t i = 0; i < 6; ++i)
        ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, delta_twist(i));

      if (delta_q_norm < eps)
        break;  // cannot reach target
    }
    Add(q_out, delta_q, q_out);

    ROS_DEBUG_STREAM_NAMED("kdl", "delta_q");
    for (std::size_t i = 0; i < delta_q.rows(); ++i)
      ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, delta_q(i));

    ROS_DEBUG_STREAM_NAMED("kdl", "q_out");
    for (std::size_t i = 0; i < q_out.rows(); ++i)
      ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_out(i));

    for (std::size_t j = 0; j < q_min.rows(); ++j)
    {
      if (q_out(j) < q_min(j))
        q_out(j) = q_min(j);
    }
    for (std::size_t j = 0; j < q_max.rows(); ++j)
    {
      if (q_out(j) > q_max(j))
        q_out(j) = q_max(j);
    }
  }

  int result = (i == maxiter) ? -3 : (success ? 0 : -2);
  ROS_INFO_NAMED("kdl", "Result %d after %d iterations:", result, i);
  for (std::size_t i = 0; i < q_out.rows(); ++i)
    ROS_DEBUG_NAMED("kdl", "%d: %f", (int)i, q_out(i));

  return result;
}

int ChainIkSolverPos_NR_JL_Mimic::CartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out,
                                                    bool lock_redundant_joints)
{
  return CartToJnt(q_init, p_in, q_out);
}

ChainIkSolverPos_NR_JL_Mimic::~ChainIkSolverPos_NR_JL_Mimic()
{
}
}
