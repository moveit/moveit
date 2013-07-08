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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/robot_state/joint_state.h>

robot_state::JointState::JointState(const robot_model::JointModel *jm) : joint_model_(jm)
{
  joint_state_values_.resize(getVariableCount());
  variable_transform_.setIdentity();
  std::vector<double> values;
  joint_model_->getVariableDefaultValues(values);
  setVariableValues(values);

  horrible_velocity_placeholder_.resize(getVariableCount()); // not actually true; topology can be different
  horrible_acceleration_placeholder_.resize(getVariableCount()); // not actually true; topology can be different
}

robot_state::JointState::JointState(const JointState &other) :
  joint_model_(other.joint_model_), variable_transform_(other.variable_transform_), joint_state_values_(other.joint_state_values_), mimic_requests_(other.mimic_requests_)
{
}

robot_state::JointState::~JointState()
{
}

robot_state::JointState& robot_state::JointState::operator=(const robot_state::JointState &other)
{
  if (this != &other)
  {
    assert(joint_state_values_.size() == other.joint_state_values_.size());
    joint_state_values_ = other.joint_state_values_;
    variable_transform_ = other.variable_transform_;
    mimic_requests_ = other.mimic_requests_;
  }
  return *this;
}

bool robot_state::JointState::setVariableValue(const std::string &variable, double value)
{
  std::map<std::string, unsigned int>::const_iterator it = getVariableIndexMap().find(variable);
  if (it != getVariableIndexMap().end())
  {
    joint_state_values_[it->second] = value;
    joint_model_->updateTransform(joint_state_values_, variable_transform_);
    updateMimicJoints();
    return true;
  }
  else
  {
    logError("Cannot set variable %s to %lf for joint %s (variable not found).", variable.c_str(), value, getName().c_str());
    return false;
  }
}

bool robot_state::JointState::setVariableValues(const std::vector<double>& joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    logError("Joint %s expects %u variables but values for %u variables were specified.",
             getName().c_str(), getVariableCount(), (unsigned int)joint_state_values.size());
    return false;
  }
  joint_state_values_ = joint_state_values;
  joint_model_->updateTransform(joint_state_values, variable_transform_);
  updateMimicJoints();
  return true;
}

void robot_state::JointState::setVariableValues(const double *joint_state_values)
{
  std::copy(joint_state_values, joint_state_values + joint_state_values_.size(), joint_state_values_.begin());
  joint_model_->updateTransform(joint_state_values_, variable_transform_);
  updateMimicJoints();
}

void robot_state::JointState::setVariableValues(const std::map<std::string, double>& joint_value_map, std::vector<std::string>& missing)
{
  bool has_any = false;
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  for (std::map<std::string, unsigned int>::const_iterator it = vim.begin(); it != vim.end(); ++it)
  {
    std::map<std::string, double>::const_iterator it2 = joint_value_map.find(it->first);
    if (it2 == joint_value_map.end())
      missing.push_back(it->first);
    else
    {
      has_any = true;
      joint_state_values_[it->second] = it2->second;
    }
  }

  if (has_any)
  {
    joint_model_->updateTransform(joint_state_values_, variable_transform_);
    updateMimicJoints();
  }
}

void robot_state::JointState::setVariableValues(const std::map<std::string, double>& joint_value_map)
{
  bool update = false;
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  // iterate over the shorter list, for efficiency reasons
  if (joint_value_map.size() <= vim.size())
    for (std::map<std::string, double>::const_iterator it = joint_value_map.begin() ; it != joint_value_map.end() ; ++it)
    {
      std::map<std::string, unsigned int>::const_iterator it2 = vim.find(it->first);
      if (it2 != vim.end())
      {
        joint_state_values_[it2->second] = it->second;
        update = true;
      }
    }
  else
    for (std::map<std::string, unsigned int>::const_iterator it = vim.begin(); it != vim.end(); ++it)
    {
      std::map<std::string, double>::const_iterator it2 = joint_value_map.find(it->first);
      if (it2 != joint_value_map.end())
      {
        update = true;
        joint_state_values_[it->second] = it2->second;
      }
    }
  if (update)
  {
    joint_model_->updateTransform(joint_state_values_, variable_transform_);
    updateMimicJoints();
  }
}

void robot_state::JointState::setVariableValues(const Eigen::Affine3d& transform)
{
  joint_model_->computeJointStateValues(transform, joint_state_values_);
  joint_model_->updateTransform(joint_state_values_, variable_transform_);
  updateMimicJoints();
}

void robot_state::JointState::updateMimicJoints()
{
  for (std::size_t i = 0 ; i < mimic_requests_.size() ; ++i)
  {
    std::vector<double> mim_val(joint_state_values_.size());
    for (std::size_t j = 0 ; j < mim_val.size() ; ++j)
      mim_val[j] = joint_state_values_[j] * mimic_requests_[i]->getJointModel()->getMimicFactor() + mimic_requests_[i]->getJointModel()->getMimicOffset();
    mimic_requests_[i]->setVariableValues(&mim_val[0]);
  }
}

void robot_state::JointState::enforceBounds()
{
  joint_model_->enforceBounds(joint_state_values_);
  joint_model_->updateTransform(joint_state_values_, variable_transform_);
  updateMimicJoints();
}

void robot_state::JointState::interpolate(const JointState *to, const double t, JointState *dest) const
{
  joint_model_->interpolate(joint_state_values_, to->joint_state_values_, t, dest->joint_state_values_);
  dest->joint_model_->updateTransform(dest->joint_state_values_, dest->variable_transform_);
  dest->updateMimicJoints();
}

bool robot_state::JointState::allVariablesAreDefined(const std::map<std::string, double>& joint_value_map) const
{
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  for (std::map<std::string, unsigned int>::const_iterator it = vim.begin() ; it != vim.end() ; ++it)
    if (joint_value_map.find(it->first) == joint_value_map.end())
      return false;
  return true;
}
