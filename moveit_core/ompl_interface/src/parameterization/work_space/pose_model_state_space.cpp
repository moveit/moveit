/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface/parameterization/work_space/pose_model_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ros/console.h>

namespace ompl_interface
{

static unsigned int getJointDimension(const pm::KinematicModel::JointModel *joint)
{
  if (joint->getType() == pm::KinematicModel::JointModel::FIXED)
    return 0;
  if (joint->getType() == pm::KinematicModel::JointModel::PLANAR)
    return 3;
  if (joint->getType() == pm::KinematicModel::JointModel::FLOATING)
    return 6;
  return 1;
}

static unsigned int getGroupDimension(const std::vector<const planning_models::KinematicModel::JointModel*> &joints)
{
  unsigned int sum = 0;
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
    sum += getJointDimension(joints[i]);
  return sum;
}

static void generateBits(const std::vector<const planning_models::KinematicModel::JointModel*> &joints,
                         unsigned int r, std::vector<bool> &bits, unsigned int start,
                         std::vector<const planning_models::KinematicModel::JointModel*> &res)
{
  if (start >= bits.size())
  {
    unsigned int sum = 0;
    for (unsigned int i = 0 ; i < bits.size() ; ++i)
      if (bits[i])
        sum += getJointDimension(joints[i]);
    if (sum == r)
    {
      res.clear();
      for (unsigned int i = 0 ; i < bits.size() ; ++i)
        if (bits[i])
          res.push_back(joints[i]);
    }
  }
  else
  {
    bits[start] = false;
    generateBits(joints, r, bits, start + 1, res);
    if (res.empty())
    {
      bits[start] = true;
      generateBits(joints, r, bits, start + 1, res);
    }
  }
}

// given the joints in the group, identify a subset of them such that the sum of their dimensions is eqyal to r
// we thus need to solve the knapsack problem; this is NP-hard, but the dimension is typically low, so we are fine
static std::vector<const planning_models::KinematicModel::JointModel*>
identifyRedundancy(const std::vector<const planning_models::KinematicModel::JointModel*> &joints, unsigned int r)
{
  std::vector<bool> bits(joints.size());
  std::vector<const planning_models::KinematicModel::JointModel*> res;
  generateBits(joints, r, bits, 0, res);
  return res;
}

}

void ompl_interface::PoseModelStateSpace::configure(void)
{ 
  // construct the state space representation
  int redundancy = getGroupDimension(spec_.joint_model_group_->getJointModels()) - count_ * 6;
  for (unsigned int i = 0 ; i < count_ ; ++i)
    addSubSpace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.0);
  ROS_DEBUG("Adding %u SE3 instances", count_);
  
  // add redundancy space
  if (redundancy > 0)
  {
    const std::vector<const planning_models::KinematicModel::JointModel*> &rjoints = identifyRedundancy(spec_.joint_model_group_->getJointModels(), redundancy);
    if (rjoints.empty())
      ROS_WARN("Unable to identify redundancy space");
    else
    {
      redundancy_.reset(new JointModelStateSpaceHelper(rjoints));
      const ob::StateSpacePtr &rspace = redundancy_->getStateSpace();
      if (rspace)
      {
        ROS_DEBUG("Redundancy space is '%s', of dimension %u", rspace->getName().c_str(), rspace->getDimension());
        addSubSpace(rspace, rspace->getDimension() / 6.0);
      }
      else
        ROS_ERROR("Unable to construct redundancy space");
    }
  }
  
  setName(getJointModelGroupName() + "_PoseModel");
  lock();
}

void ompl_interface::PoseModelStateSpace::copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const
{

}

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<double> &values) const
{

}

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const
{

}

void ompl_interface::PoseModelStateSpace::setup(void)
{
  KinematicModelStateSpace::setup();
  
}
