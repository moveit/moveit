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

#include "planning_models/kinematic_state.h"

planning_models::KinematicState::JointStateGroup::JointStateGroup(planning_models::KinematicState *state,
                                                                  const planning_models::KinematicModel::JointModelGroup *jmg) :
  kinematic_state_(state), joint_model_group_(jmg)
{
  const std::vector<const KinematicModel::JointModel*>& joint_model_vector = jmg->getJointModels();
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    if (!kinematic_state_->hasJointState(joint_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No joint state for group joint name " << joint_model_vector[i]->getName());
      continue;
    }
    JointState* js = kinematic_state_->getJointState(joint_model_vector[i]->getName());
    joint_state_vector_.push_back(js);
    joint_state_map_[joint_model_vector[i]->getName()] = js;
  }
  const std::vector<const KinematicModel::LinkModel*>& link_model_vector = jmg->getUpdatedLinkModels();
  for (unsigned int i = 0; i < link_model_vector.size(); i++)
  {
    if (!kinematic_state_->hasLinkState(link_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No link state for link joint name " << link_model_vector[i]->getName());
      continue;
    }
    LinkState* ls = kinematic_state_->getLinkState(link_model_vector[i]->getName());
    updated_links_.push_back(ls);
  }
  
  const std::vector<const KinematicModel::JointModel*>& joint_root_vector = jmg->getJointRoots();
  for (std::size_t i = 0; i < joint_root_vector.size(); ++i)
  {
    JointState* js = kinematic_state_->getJointState(joint_root_vector[i]->getName());
    if (js)
      joint_roots_.push_back(js);
  }
}

planning_models::KinematicState::JointStateGroup::~JointStateGroup(void)
{
}

random_numbers::RandomNumberGenerator& planning_models::KinematicState::JointStateGroup::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

bool planning_models::KinematicState::JointStateGroup::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool planning_models::KinematicState::JointStateGroup::updatesLinkState(const std::string& link) const
{
  for (std::size_t i = 0 ; i < updated_links_.size() ; ++i)
    if (updated_links_[i]->getName() == link)
      return true;
  return false;
}

bool planning_models::KinematicState::JointStateGroup::setStateValues(const std::vector<double> &joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    ROS_ERROR("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
              getVariableCount(), (int)joint_state_values.size());
    return false;
  }
  
  unsigned int value_counter = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    unsigned int dim = joint_state_vector_[i]->getVariableCount();
    if (dim != 0)
    {
      joint_state_vector_[i]->setVariableValues(&joint_state_values[value_counter]);
      value_counter += dim;
    }
  }
  updateLinkTransforms();
  return true;
}

void planning_models::KinematicState::JointStateGroup::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < updated_links_.size(); ++i)
    updated_links_[i]->computeTransform();
}

void planning_models::KinematicState::JointStateGroup::copyFrom(const JointStateGroup *other_jsg)
{
  const std::vector<JointState*> &ojsv = other_jsg->getJointStateVector();
  for (std::size_t i = 0 ; i < ojsv.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(ojsv[i]->getVariableValues());
  updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::setToDefaultValues(void)
{
  std::map<std::string, double> default_joint_values;
  for (std::size_t i = 0  ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->getJointModel()->getDefaultValues(default_joint_values);
  setStateValues(default_joint_values);
}

bool planning_models::KinematicState::JointStateGroup::setToDefaultState(const std::string &name)
{
  std::map<std::string, double> default_joint_values;
  if (!joint_model_group_->getDefaultValues(name, default_joint_values))
    return false;
  setStateValues(default_joint_values);
  return true;
}

void planning_models::KinematicState::JointStateGroup::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  joint_model_group_->getRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::map<std::string,double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<double> &jsv = joint_state_vector_[i]->getVariableValues();
    const std::vector<std::string> &jsn = joint_state_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < jsv.size(); ++j)
      joint_state_values[jsn[j]] = jsv[j];
  }
}

planning_models::KinematicState::JointState* planning_models::KinematicState::JointStateGroup::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}
