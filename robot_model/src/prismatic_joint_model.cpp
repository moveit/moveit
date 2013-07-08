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

#include <moveit/robot_model/prismatic_joint_model.h>
#include <limits>

robot_model::PrismaticJointModel::PrismaticJointModel(const std::string& name) : JointModel(name), axis_(0.0, 0.0, 0.0)
{
  type_ = PRISMATIC;
  variable_bounds_.push_back(std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
  variable_names_.push_back(name_);
}

unsigned int robot_model::PrismaticJointModel::getStateSpaceDimension() const
{
  return 1;
}

double robot_model::PrismaticJointModel::getMaximumExtent(const Bounds &other_bounds) const
{
  return other_bounds[0].second - other_bounds[0].first;
}

void robot_model::PrismaticJointModel::getVariableDefaultValues(std::vector<double> &values, const Bounds &bounds) const
{
  // if zero is a valid value
  if (bounds[0].first <= 0.0 && bounds[0].second >= 0.0)
    values.push_back(0.0);
  else
    values.push_back((bounds[0].first + bounds[0].second)/2.0);
}

bool robot_model::PrismaticJointModel::satisfiesBounds(const std::vector<double> &values, const Bounds &bounds, double margin) const
{
  assert(bounds.size() > 0);
  if (values[0] < bounds[0].first - margin || values[0] > bounds[0].second + margin)
    return false;
  return true;
}

void robot_model::PrismaticJointModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds) const
{
  values.push_back(rng.uniformReal(bounds[0].first, bounds[0].second));
}

void robot_model::PrismaticJointModel::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds,
                                                                         const std::vector<double> &near, const double distance) const
{
  values.push_back(rng.uniformReal(std::max(bounds[0].first, near[values.size()] - distance),
                                   std::min(bounds[0].second, near[values.size()] + distance)));
}

void robot_model::PrismaticJointModel::enforceBounds(std::vector<double> &values, const Bounds &bounds) const
{
  const std::pair<double, double> &b = bounds[0];
  if (values[0] < b.first)
    values[0] = b.first;
  else
    if (values[0] > b.second)
      values[0] = b.second;
}

double robot_model::PrismaticJointModel::distance(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  assert(values1.size() == 1);
  assert(values2.size() == 1);
  return fabs(values1[0] - values2[0]);
}

void robot_model::PrismaticJointModel::interpolate(const std::vector<double> &from, const std::vector<double> &to, const double t, std::vector<double> &state) const
{
  state[0] = from[0] + (to[0] - from[0]) * t;
}

void robot_model::PrismaticJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.setIdentity();
  updateTransform(joint_values, transf);
}

void robot_model::PrismaticJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.translation() = Eigen::Vector3d(axis_ * joint_values[0]);
}

void robot_model::PrismaticJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(1);
  joint_values[0] = transf.translation().dot(axis_);
}
