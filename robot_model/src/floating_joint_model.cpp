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

#include <moveit/robot_model/floating_joint_model.h>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <cmath>

robot_model::FloatingJointModel::FloatingJointModel(const std::string& name) : JointModel(name), angular_distance_weight_(1.0)
{
  type_ = FLOATING;
  local_variable_names_.push_back("trans_x");
  local_variable_names_.push_back("trans_y");
  local_variable_names_.push_back("trans_z");
  local_variable_names_.push_back("rot_x");
  local_variable_names_.push_back("rot_y");
  local_variable_names_.push_back("rot_z");
  local_variable_names_.push_back("rot_w");
  for (int i = 0 ; i < 7 ; ++i)
    variable_names_.push_back(name_ + "/" + local_variable_names_[i]);
  variable_bounds_.resize(7);
  variable_bounds_[0] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[1] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[2] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[3] = std::make_pair(-1.0, 1.0);
  variable_bounds_[4] = std::make_pair(-1.0, 1.0);
  variable_bounds_[5] = std::make_pair(-1.0, 1.0);
  variable_bounds_[6] = std::make_pair(-1.0, 1.0);
}

double robot_model::FloatingJointModel::getMaximumExtent(const Bounds &other_bounds) const
{
  double dx = other_bounds[0].first - other_bounds[0].second;
  double dy = other_bounds[1].first - other_bounds[1].second;
  double dz = other_bounds[2].first - other_bounds[2].second;
  return sqrt(dx*dx + dy*dy + dz*dz) + boost::math::constants::pi<double>() * 0.5 * angular_distance_weight_;
}

double robot_model::FloatingJointModel::distance(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  assert(values1.size() == 7);
  assert(values2.size() == 7);
  double dx = values1[0] - values2[0];
  double dy = values1[1] - values2[1];
  double dz = values1[2] - values2[2];
  double dq = fabs(values1[3] * values2[3] + values1[4] * values2[4] + values1[5] * values2[5] + values1[6] * values2[6]);
  if (dq + std::numeric_limits<double>::epsilon() >= 1.0)
    return sqrt(dx*dx + dy*dy + dz * dz);
  else
    return angular_distance_weight_ * acos(dq) + sqrt(dx*dx + dy*dy + dz * dz);
}

double robot_model::FloatingJointModel::distanceTranslation(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  assert(values1.size() == 7);
  assert(values2.size() == 7);
  double dx = values1[0] - values2[0];
  double dy = values1[1] - values2[1];
  double dz = values1[2] - values2[2];
  return sqrt(dx*dx + dy*dy + dz * dz);
}

double robot_model::FloatingJointModel::distanceRotation(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  double dq = fabs(values1[3] * values2[3] + values1[4] * values2[4] + values1[5] * values2[5] + values1[6] * values2[6]);
  if (dq + std::numeric_limits<double>::epsilon() >= 1.0)
    return 0.0;
  else
    return acos(dq);
}


void robot_model::FloatingJointModel::interpolate(const std::vector<double> &from, const std::vector<double> &to, const double t, std::vector<double> &state) const
{
  // interpolate position
  state[0] = from[0] + (to[0] - from[0]) * t;
  state[1] = from[1] + (to[1] - from[1]) * t;
  state[2] = from[2] + (to[2] - from[2]) * t;

  double dq = fabs(from[3] * to[3] + from[4] * to[4] + from[5] * to[5] + from[6] * to[6]);
  double theta = (dq + std::numeric_limits<double>::epsilon() >= 1.0) ? 0.0 : acos(dq);
  if (theta > std::numeric_limits<double>::epsilon())
  {
    double d = 1.0 / sin(theta);
    double s0 = sin((1.0 - t) * theta);
    double s1 = sin(t * theta);
    if (dq < 0)  // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
      s1 = -s1;
    state[3] = (from[3] * s0 + to[3] * s1) * d;
    state[4] = (from[4] * s0 + to[4] * s1) * d;
    state[5] = (from[5] * s0 + to[5] * s1) * d;
    state[6] = (from[6] * s0 + to[6] * s1) * d;
  }
  else
  {
    state[3] = from[3];
    state[4] = from[4];
    state[5] = from[5];
    state[6] = from[6];
  }
}

bool robot_model::FloatingJointModel::satisfiesBounds(const std::vector<double> &values, const Bounds &bounds, double margin) const
{
  assert(bounds.size() > 2);

  if (values[0] < bounds[0].first - margin || values[0] > bounds[0].second + margin)
    return false;
  if (values[1] < bounds[1].first - margin || values[1] > bounds[1].second + margin)
    return false;
  if (values[2] < bounds[2].first - margin || values[2] > bounds[2].second + margin)
    return false;
  double normSqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  if (fabs(normSqr - 1.0) > std::numeric_limits<float>::epsilon() * 10.0)
    return false;
  return true;
}

bool robot_model::FloatingJointModel::normalizeRotation(std::vector<double> &values) const
{
  // normalize the quaternion if we need to
  double normSqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  if (fabs(normSqr - 1.0) > std::numeric_limits<double>::epsilon() * 100.0)
  {
    double norm = sqrt(normSqr);
    if (norm < std::numeric_limits<double>::epsilon() * 100.0)
    {
      logWarn("Quaternion is zero in RobotState *representation. Setting to identity");
      values[3] = 0.0;
      values[4] = 0.0;
      values[5] = 0.0;
      values[6] = 1.0;
    }
    else
    {
      values[3] /= norm;
      values[4] /= norm;
      values[5] /= norm;
      values[6] /= norm;
    }
    return true;
  }
  else
    return false;
}

unsigned int robot_model::FloatingJointModel::getStateSpaceDimension() const
{
  return 6;
}

void robot_model::FloatingJointModel::enforceBounds(std::vector<double> &values, const Bounds &bounds) const
{
  normalizeRotation(values);
  for (unsigned int i = 0 ; i < 3 ; ++i)
  {
    const std::pair<double, double> &b = bounds[i];
    if (values[i] < b.first)
      values[i] = b.first;
    else
      if (values[i] > b.second)
        values[i] = b.second;
  }
}

void robot_model::FloatingJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  updateTransform(joint_values, transf);
}

void robot_model::FloatingJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf = Eigen::Affine3d(Eigen::Translation3d(joint_values[0], joint_values[1], joint_values[2])
                           *Eigen::Quaterniond(joint_values[6],joint_values[3], joint_values[4], joint_values[5]).toRotationMatrix());
}

void robot_model::FloatingJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(7);
  joint_values[0] = transf.translation().x();
  joint_values[1] = transf.translation().y();
  joint_values[2] = transf.translation().z();
  Eigen::Quaterniond q(transf.rotation());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}

void robot_model::FloatingJointModel::getVariableDefaultValues(std::vector<double>& values, const Bounds &bounds) const
{
  assert(bounds.size() > 2);
  for (unsigned int i = 0 ; i < 3 ; ++i)
  {
    // if zero is a valid value
    if (bounds[i].first <= 0.0 && bounds[i].second >= 0.0)
      values.push_back(0.0);
    else
      values.push_back((bounds[i].first + bounds[i].second)/2.0);
  }

  values.push_back(0.0);
  values.push_back(0.0);
  values.push_back(0.0);
  values.push_back(1.0);
}

void robot_model::FloatingJointModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds) const
{
  std::size_t s = values.size();
  values.resize(s + 7);

  if (bounds[0].second >= std::numeric_limits<double>::max() || bounds[0].first <= -std::numeric_limits<double>::max())
    values[s] = 0.0;
  else
    values[s] = rng.uniformReal(bounds[0].first, bounds[0].second);
  if (bounds[1].second >= std::numeric_limits<double>::max() || bounds[1].first <= -std::numeric_limits<double>::max())
    values[s + 1] = 0.0;
  else
    values[s + 1] = rng.uniformReal(bounds[1].first, bounds[1].second);
  if (bounds[2].second >= std::numeric_limits<double>::max() || bounds[2].first <= -std::numeric_limits<double>::max())
    values[s + 2] = 0.0;
  else
    values[s + 2] = rng.uniformReal(bounds[2].first, bounds[2].second);

  double q[4]; rng.quaternion(q);
  values[s + 3] = q[0];
  values[s + 4] = q[1];
  values[s + 5] = q[2];
  values[s + 6] = q[3];
}

void robot_model::FloatingJointModel::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds,
                                                                        const std::vector<double> &near, const double distance) const
{
  std::size_t s = values.size();
  values.resize(s + 7);

  if (bounds[0].second >= std::numeric_limits<double>::max() || bounds[0].first <= -std::numeric_limits<double>::max())
    values[s] = 0.0;
  else
    values[s] = rng.uniformReal(std::max(bounds[0].first, near[s] - distance),
                                std::min(bounds[0].second, near[s] + distance));
  if (bounds[1].second >= std::numeric_limits<double>::max() || bounds[1].first <= -std::numeric_limits<double>::max())
    values[s + 1] = 0.0;
  else
    values[s + 1] = rng.uniformReal(std::max(bounds[1].first, near[s + 1] - distance),
                                    std::min(bounds[1].second, near[s + 1] + distance));
  if (bounds[2].second >= std::numeric_limits<double>::max() || bounds[2].first <= -std::numeric_limits<double>::max())
    values[s + 2] = 0.0;
  else
    values[s + 2] = rng.uniformReal(std::max(bounds[2].first, near[s + 2] - distance),
                                    std::min(bounds[2].second, near[s + 2] + distance));

  double da = angular_distance_weight_ * distance;
  if (da >= .25 * boost::math::constants::pi<double>())
  {
    double q[4]; rng.quaternion(q);
    values[s + 3] = q[0];
    values[s + 4] = q[1];
    values[s + 5] = q[2];
    values[s + 6] = q[3];
  }
  else
  {
    //taken from OMPL
    // sample angle & axis
    double ax = rng.gaussian01();
    double ay = rng.gaussian01();
    double az = rng.gaussian01();
    double angle = 2.0 * pow(rng.uniform01(), 1.0/3.0) * da;
    // convert to quaternion
    double q[4];
    double norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6)
    {
      q[0] = q[1] = q[2] = 0.0;
      q[3] = 1.0;
    }
    else
    {
      double s = sin(angle / 2.0);
      q[0] = s * ax / norm;
      q[1] = s * ay / norm;
      q[2] = s * az / norm;
      q[3] = cos(angle / 2.0);
    }
    // multiply quaternions: near * q
    values[s + 3] = near[s + 6]*q[0] + near[s + 3]*q[3] + near[s + 4]*q[2] - near[s + 5]*q[1];
    values[s + 4] = near[s + 6]*q[1] + near[s + 4]*q[3] + near[s + 5]*q[0] - near[s + 3]*q[2];
    values[s + 5] = near[s + 6]*q[2] + near[s + 5]*q[3] + near[s + 3]*q[1] - near[s + 4]*q[0];
    values[s + 6] = near[s + 6]*q[3] - near[s + 3]*q[0] - near[s + 4]*q[1] - near[s + 5]*q[2];
  }
}
