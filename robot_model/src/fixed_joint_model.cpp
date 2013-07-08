/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/robot_model/fixed_joint_model.h>

robot_model::FixedJointModel::FixedJointModel(const std::string& name) : JointModel(name)
{
  type_ = FIXED;
}

unsigned int robot_model::FixedJointModel::getStateSpaceDimension() const
{
  return 0;
}

void robot_model::FixedJointModel::getVariableDefaultValues(std::vector<double>& values, const Bounds &bounds) const
{
}

void robot_model::FixedJointModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds) const
{
}

void robot_model::FixedJointModel::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds,
                                                                     const std::vector<double> &near, const double distance) const
{
}

void robot_model::FixedJointModel::enforceBounds(std::vector<double> &values, const Bounds &bounds) const
{
}

bool robot_model::FixedJointModel::satisfiesBounds(const std::vector<double> &values, const Bounds &bounds, double margin) const
{
  return true;
}

double robot_model::FixedJointModel::distance(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  return 0.0;
}

double robot_model::FixedJointModel::getMaximumExtent(const Bounds &other_bounds) const
{
  return 0.0;
}

void robot_model::FixedJointModel::interpolate(const std::vector<double> &from, const std::vector<double> &to, const double t, std::vector<double> &state) const
{
}

void robot_model::FixedJointModel::computeTransform(const std::vector<double>& /* joint_values */, Eigen::Affine3d &transf) const
{
  transf.setIdentity();
}

void robot_model::FixedJointModel::updateTransform(const std::vector<double>& /* joint_values */, Eigen::Affine3d &transf) const
{
}

void robot_model::FixedJointModel::computeJointStateValues(const Eigen::Affine3d& /* transform */, std::vector<double>& joint_values) const
{
  joint_values.clear();
}
