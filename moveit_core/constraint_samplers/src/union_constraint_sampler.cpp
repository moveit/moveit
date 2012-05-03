/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include "constraint_samplers/union_constraint_sampler.h"
#include <ros/console.h>
#include <algorithm>

namespace constraint_samplers
{
struct OrderSamplersByFrameDependency
{
  bool operator()(const ConstraintSamplerPtr &a, const ConstraintSamplerPtr &b) const
  {
    bool a_depends_on_b = false;  
    bool b_depends_on_a = false;
    const std::vector<std::string> &fda = a->getFrameDependency();
    const std::vector<std::string> &fdb = b->getFrameDependency();
    const std::vector<std::string> &alinks = a->getJointModelGroup()->getLinkModelNames();
    const std::vector<std::string> &blinks = b->getJointModelGroup()->getLinkModelNames();
    for (std::size_t i = 0 ; i < fda.size() && !a_depends_on_b ; ++i)
      for (std::size_t j = 0 ; j < blinks.size() ; ++j)
        if (blinks[j] == fda[i])
        {
          a_depends_on_b = true;
          break;
        }
    for (std::size_t i = 0 ; i < fdb.size() && !b_depends_on_a ; ++i)
      for (std::size_t j = 0 ; j < alinks.size() ; ++j)
        if (alinks[j] == fdb[i])
        {
          b_depends_on_a = true;
          break;
        }
    if (b_depends_on_a && a_depends_on_b)
      ROS_WARN("Circular frame dependency! Sampling will likely produce invalid results (sampling for groups '%s' and '%s')",
               a->getJointModelGroup()->getName().c_str(), b->getJointModelGroup()->getName().c_str());
    return b_depends_on_a && !a_depends_on_b;
  }  
};
}

constraint_samplers::UnionConstraintSampler::UnionConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg, std::vector<ConstraintSamplerPtr> &samplers) :
  ConstraintSampler(), samplers_(samplers)
{
  jmg_ = jmg;  
  std::sort(samplers_.begin(), samplers_.end(), OrderSamplersByFrameDependency());

  const std::map<std::string, unsigned int> &gi = jmg->getJointVariablesIndexMap();
  bijection_.resize(samplers_.size());
  for (std::size_t i = 0 ; i < samplers_.size() ; ++i)
  { 
    const std::vector<std::string> &fd = samplers_[i]->getFrameDependency();
    for (std::size_t j = 0 ; j < fd.size() ; ++j)
      frame_depends_.push_back(fd[j]);
    
    ROS_DEBUG_STREAM("Union sampler for group '" << jmg->getName() << "' includes sampler for group '" << samplers_[i]->getJointModelGroup()->getName() << "'");
    bijection_[i].resize(gi.size(), -1);
    const std::map<std::string, unsigned int> &sgi = samplers_[i]->getJointModelGroup()->getJointVariablesIndexMap();
    for (std::map<std::string, unsigned int>::const_iterator it = sgi.begin() ; it != sgi.end() ; ++it)
    {
      std::map<std::string, unsigned int>::const_iterator jt = gi.find(it->first);
      if (jt == gi.end())
        ROS_FATAL("Subgroups do not match group");
      bijection_[i][it->second] = jt->second;
    }
  }
}

bool constraint_samplers::UnionConstraintSampler::sample(std::vector<double> &values, const planning_models::KinematicState &ks, unsigned int max_attempts)
{
  values.clear();
  jmg_->getRandomValues(random_number_generator_, values);

  std::vector<double> v;
  if (samplers_.size() >= 1)
  {
    if (!samplers_[0]->sample(v, ks, max_attempts))
      return false;
    for (std::size_t j = 0 ; j < v.size() ; ++j)
      values[bijection_[0][j]] = v[j];
  }
  if (samplers_.size() > 1)
  {
    planning_models::KinematicState temp = ks;  
    for (std::size_t i = 1 ; i < samplers_.size() ; ++i)
    {
      temp.getJointStateGroup(samplers_[i-1]->getJointModelGroup()->getName())->setStateValues(v);
      if (!samplers_[i]->sample(v, temp, max_attempts))
        return false;
      for (std::size_t j = 0 ; j < v.size() ; ++j)
        values[bijection_[i][j]] = v[j];
    }
  }
  return true;
}
