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

#include "ompl_interface/parameterization/model_based_state_space.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/console.h>

namespace ompl_interface
{
static void setPlanningVolumeAux(ob::StateSpace *space, double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  if (space->getType() == ob::STATE_SPACE_SE3)
  {
    ob::RealVectorBounds b(3);
    b.setLow(0, minX); b.setLow(1, minY); b.setLow(2, minZ);
    b.setHigh(0, maxX); b.setHigh(1, maxY); b.setHigh(2, maxZ);
    space->as<ob::SE3StateSpace>()->setBounds(b);
  }
  else
    if (space->getType() == ob::STATE_SPACE_SE2)
    {
      ob::RealVectorBounds b(2);
      b.setLow(0, minX); b.setLow(1, minY);
      b.setHigh(0, maxX); b.setHigh(1, maxY);
      space->as<ob::SE2StateSpace>()->setBounds(b);
    }
  if (space->isCompound())
  {
    const std::vector<ob::StateSpacePtr> &c = space->as<ob::CompoundStateSpace>()->getSubSpaces();
    for (std::size_t i = 0 ; i < c.size() ; ++i)
      setPlanningVolumeAux(c[i].get(), minX, maxX, minY, maxY, minZ, maxZ);
  }
}
}

void ompl_interface::KinematicModelStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  setPlanningVolumeAux(this, minX, maxX, minY, maxY, minZ, maxZ);
}

ompl::base::State* ompl_interface::KinematicModelStateSpace::allocState(void) const
{
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl_interface::KinematicModelStateSpace::freeState(ob::State *state) const
{
  CompoundStateSpace::freeState(state);
}

void ompl_interface::KinematicModelStateSpace::copyState(ob::State *destination, const ob::State *source) const
{
  CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->tag = source->as<StateType>()->tag;  
}

void ompl_interface::KinematicModelStateSpace::interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{
  CompoundStateSpace::interpolate(from, to, t, state);
  if (from->as<StateType>()->tag >= 0 && to->as<StateType>()->tag >= 0)
    state->as<StateType>()->tag = t < 0.5 ? from->as<StateType>()->tag : to->as<StateType>()->tag;
  else
    state->as<StateType>()->tag = std::max(from->as<StateType>()->tag, to->as<StateType>()->tag);
}

void ompl_interface::KinematicModelStateSpace::copyToKinematicState(pm::KinematicState &kstate, const ob::State *state) const
{
  copyToKinematicState(kstate.getJointStateGroup(getJointModelGroupName())->getJointStateVector(), state);
}

void ompl_interface::KinematicModelStateSpace::copyToOMPLState(ob::State *state, const pm::KinematicState &kstate) const
{
  copyToOMPLState(state, kstate.getJointStateGroup(getJointModelGroupName())->getJointStateVector());
}

void ompl_interface::KinematicModelStateSpace::useIKAllocators(const std::map<std::string, kinematic_constraints::IKAllocator> &ik_allocators)
{
  std::map<std::string, kinematic_constraints::IKAllocator>::const_iterator jt = ik_allocators.find(getJointModelGroupName());
  if (jt == ik_allocators.end())
  {
    const pm::KinematicModel::JointModelGroup *jmg = spec_.joint_model_group_;
    
    // if an IK allocator is NOT available for this group, we try to see if we can use subgroups for IK
    std::set<const pm::KinematicModel::JointModel*> joints;
    joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());
    
    std::vector<const pm::KinematicModel::JointModelGroup*> subs;
    
    // go through the groups that we know have IK allocators and see if they are included in the group that does not; fi so, put that group in sub
    for (std::map<std::string, kinematic_constraints::IKAllocator>::const_iterator kt = ik_allocators.begin() ;
         kt != ik_allocators.end() ; ++kt)
    {
      const pm::KinematicModel::JointModelGroup *sub = spec_.kmodel_->getJointModelGroup(kt->first);
      std::set<const pm::KinematicModel::JointModel*> sub_joints;
      sub_joints.insert(sub->getJointModels().begin(), sub->getJointModels().end());
      
      if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
      {
        std::set<const pm::KinematicModel::JointModel*> result;
        std::set_difference(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end(),
                            std::inserter(result, result.end()));
        subs.push_back(sub);
        joints = result;
      }
    }
    
    // if we found subgroups, pass that information to the planning group
    if (!subs.empty())
    {
      ik_subgroup_allocators_.clear();
      std::stringstream ss;
      for (std::size_t i = 0 ; i < subs.size() ; ++i)
      {
        ss << subs[i]->getName() << " ";
        ik_subgroup_allocators_[subs[i]] = ik_allocators.find(subs[i]->getName())->second;
      }
      ROS_INFO("Added sub-group IK allocators for group '%s': [ %s]", jmg->getName().c_str(), ss.str().c_str());
    }
  }
  else
    // if the IK allocator is for this group, we use it
    ik_allocator_ = jt->second;
}
