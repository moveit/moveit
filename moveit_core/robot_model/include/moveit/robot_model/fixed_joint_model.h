/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan */

#ifndef MOVEIT_CORE_ROBOT_MODEL_FIXED_JOINT_MODEL_
#define MOVEIT_CORE_ROBOT_MODEL_FIXED_JOINT_MODEL_

#include <moveit/robot_model/joint_model.h>

namespace moveit
{
namespace core
{
/** \brief A fixed joint */
class FixedJointModel : public JointModel
{
public:
  FixedJointModel(const std::string& name);

  virtual void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const;
  virtual void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                          const Bounds& other_bounds) const;
  virtual void getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                const Bounds& other_bounds, const double* near,
                                                const double distance) const;
  virtual bool enforcePositionBounds(double* values, const Bounds& other_bounds) const;
  virtual bool satisfiesPositionBounds(const double* values, const Bounds& other_bounds, double margin) const;

  virtual void interpolate(const double* from, const double* to, const double t, double* state) const;
  virtual unsigned int getStateSpaceDimension() const;
  virtual double getMaximumExtent(const Bounds& other_bounds) const;
  virtual double distance(const double* values1, const double* values2) const;

  virtual void computeTransform(const double* joint_values, Eigen::Affine3d& transf) const;
  virtual void computeVariablePositions(const Eigen::Affine3d& transf, double* joint_values) const;
};
}
}

#endif
