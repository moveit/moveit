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

#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <algorithm>

namespace constraint_samplers
{
struct OrderSamplers
{
  bool operator()(const ConstraintSamplerPtr &a, const ConstraintSamplerPtr &b) const
  {
    const std::vector<std::string> &alinks = a->getJointModelGroup()->getUpdatedLinkModelNames();
    const std::vector<std::string> &blinks = b->getJointModelGroup()->getUpdatedLinkModelNames();
    std::set<std::string> a_updates(alinks.begin(), alinks.end());
    std::set<std::string> b_updates(blinks.begin(), blinks.end());

    bool a_contains_b = std::includes(a_updates.begin(), a_updates.end(),
                                      b_updates.begin(), b_updates.end());

    bool b_contains_a = std::includes(b_updates.begin(), b_updates.end(),
                                      a_updates.begin(), a_updates.end());

    //a contains b and sets are not equal
    if (a_contains_b && !b_contains_a)
      return true;
    if (b_contains_a && !a_contains_b)
      return false;

    //sets are equal or disjoint
    bool a_depends_on_b = false;
    bool b_depends_on_a = false;
    const std::vector<std::string> &fda = a->getFrameDependency();
    const std::vector<std::string> &fdb = b->getFrameDependency();
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
    {
      logWarn("Circular frame dependency! Sampling will likely produce invalid results (sampling for groups '%s' and '%s')",
              a->getJointModelGroup()->getName().c_str(), b->getJointModelGroup()->getName().c_str());
      return true;
    }
    if (b_depends_on_a && !a_depends_on_b)
      return true;
    if(a_depends_on_b && !b_depends_on_a)
      return false;

    // prefer sampling JointConstraints first
    JointConstraintSampler *ja = dynamic_cast<JointConstraintSampler*>(a.get());
    JointConstraintSampler *jb = dynamic_cast<JointConstraintSampler*>(b.get());
    if (ja && jb == NULL)
      return true;
    if (jb && ja == NULL)
      return false;

    // neither depends on either, so break ties based on group name
    return (a->getJointModelGroup()->getName() < b->getJointModelGroup()->getName());
  }
};
}

constraint_samplers::UnionConstraintSampler::UnionConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                                                                    const std::vector<ConstraintSamplerPtr> &samplers) :
  ConstraintSampler(scene, group_name), samplers_(samplers)
{
  // using stable sort to preserve order of equivalents
  std::stable_sort(samplers_.begin(), samplers_.end(), OrderSamplers());

  for (std::size_t i = 0 ; i < samplers_.size() ; ++i)
  {
    const std::vector<std::string> &fd = samplers_[i]->getFrameDependency();
    for (std::size_t j = 0 ; j < fd.size() ; ++j)
      frame_depends_.push_back(fd[j]);

    logDebug("Union sampler for group '%s' includes sampler for group '%s'", jmg_->getName().c_str(), samplers_[i]->getJointModelGroup()->getName().c_str());
  }
}

bool constraint_samplers::UnionConstraintSampler::sample(robot_state::JointStateGroup *jsg, const robot_state::RobotState &ks, unsigned int max_attempts)
{
  jsg->setToRandomValues();

  if (samplers_.size() >= 1)
  {
    if (!samplers_[0]->sample(jsg->getRobotState()->getJointStateGroup(samplers_[0]->getJointModelGroup()->getName()), ks, max_attempts))
      return false;
  }

  if (samplers_.size() > 1)
  {
    robot_state::RobotState temp = ks;
    *(temp.getJointStateGroup(jsg->getName())) = *jsg;

    for (std::size_t i = 1 ; i < samplers_.size() ; ++i)
    {
      robot_state::JointStateGroup *x = jsg->getRobotState()->getJointStateGroup(samplers_[i]->getJointModelGroup()->getName());
      if (samplers_[i]->sample(x, temp, max_attempts))
      {
        if (i + 1 < samplers_.size())
          *(temp.getJointStateGroup(samplers_[i]->getJointModelGroup()->getName())) = *x;
      }
      else
        return false;
    }
  }

  return true;
}

bool constraint_samplers::UnionConstraintSampler::project(robot_state::JointStateGroup *jsg, const robot_state::RobotState &ks, unsigned int max_attempts)
{
  if (samplers_.size() >= 1)
  {
    if (!samplers_[0]->project(jsg->getRobotState()->getJointStateGroup(samplers_[0]->getJointModelGroup()->getName()), ks, max_attempts))
      return false;
  }

  if (samplers_.size() > 1)
  {
    robot_state::RobotState temp = ks;
    *(temp.getJointStateGroup(jsg->getName())) = *jsg;

    for (std::size_t i = 1 ; i < samplers_.size() ; ++i)
    {
      robot_state::JointStateGroup *x = jsg->getRobotState()->getJointStateGroup(samplers_[i]->getJointModelGroup()->getName());
      if (samplers_[i]->project(x, temp, max_attempts))
      {
        if (i + 1 < samplers_.size())
          *(temp.getJointStateGroup(samplers_[i]->getJointModelGroup()->getName())) = *x;
      }
      else
        return false;
    }
  }

  return true;
}
