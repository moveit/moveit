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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <moveit_msgs/JointLimits.h>
#include <kinematic_constraints/constraint_samplers.h>

namespace ompl_interface
{

namespace ob = ompl::base;
namespace pm = planning_models;
namespace kc = kinematic_constraints;

class KinematicModelStateSpace;
typedef boost::shared_ptr<KinematicModelStateSpace> KinematicModelStateSpacePtr;

struct KinematicModelStateSpaceSpecification
{
  KinematicModelStateSpaceSpecification(const pm::KinematicModelConstPtr &kmodel,
                                        const pm::KinematicModel::JointModelGroup *jmg) :
    kmodel_(kmodel), joint_model_group_(jmg)
  {
  }

  KinematicModelStateSpaceSpecification(const pm::KinematicModelConstPtr &kmodel,
                                        const std::string &group_name) :
    kmodel_(kmodel), joint_model_group_(kmodel_->getJointModelGroup(group_name))
  {
    if (!joint_model_group_)
      throw std::runtime_error("Group '" + group_name + "'  was not found");
  }
  
  pm::KinematicModelConstPtr                 kmodel_;
  const pm::KinematicModel::JointModelGroup *joint_model_group_;
};

class KinematicModelStateSpace : public ob::CompoundStateSpace
{
public:
  
  /// Add a tag to the state representation
  class StateType : public ob::CompoundStateSpace::StateType
  {
  public:
    StateType(void) : ob::CompoundStateSpace::StateType(), tag(-1)
    {
    }    
    int tag;
  };
  
  KinematicModelStateSpace(const KinematicModelStateSpaceSpecification &spec) : 
    ob::CompoundStateSpace(), spec_(spec)
  {
  }
  
  virtual ~KinematicModelStateSpace(void)
  {
  }
    
  virtual ob::State* allocState(void) const;
  virtual void freeState(ob::State *state) const;
  virtual void copyState(ob::State *destination, const ob::State *source) const;
  virtual void interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const;
  
  const pm::KinematicModelConstPtr& getKinematicModel(void) const
  {
    return spec_.kmodel_;
  }

  const pm::KinematicModel::JointModelGroup* getJointModelGroup(void) const
  {
    return spec_.joint_model_group_;
  }  
  
  const std::string& getJointModelGroupName(void) const
  {
    return getJointModelGroup()->getName();
  }
  
  const KinematicModelStateSpaceSpecification& getSpecification(void) const
  {
    return spec_;
  }
  
  /// Copy the data from an OMPL state to a set of joint states. The join states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const = 0;
  
  /// Copy the data from an OMPL state to a kinematic state. The join states \b must be specified in the same order as the joint models in the constructor. This function is implemented in terms of the previous definition with the same name.
  void copyToKinematicState(pm::KinematicState &kstate, const ob::State *state) const;

  /// Copy the data from a value vector that corresponds to the state of the considered joint model group (or array of joints)
  virtual void copyToOMPLState(ob::State *state, const std::vector<double> &values) const = 0;
    
  /// Copy the data from a set of joint states to an OMPL state. The join states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const = 0;
  
  /// Copy the data from a kinematic state to an OMPL state. Only needed joint states are copied. This function is implemented in terms of the previous definition with the same name.
  void copyToOMPLState(ob::State *state, const pm::KinematicState &kstate) const;
    
  /// Set the planning volume for the possible SE2 and/or SE3 components of the state space
  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);

  void useIKAllocators(const std::map<std::string, kc::IKAllocator> &ik_allocators);
  
  const kc::IKAllocator& getIKAllocator(void) const
  {
    return ik_allocator_;
  }

  const kc::IKSubgroupAllocator& getIKSubgroupAllocators(void) const
  {
    return ik_subgroup_allocators_;
  }
  
protected:
  
  KinematicModelStateSpaceSpecification      spec_; 

  /// a function pointer that returns an IK solver for this group; this is useful for sampling states using IK
  kc::IKAllocator         ik_allocator_;
  kc::IKSubgroupAllocator ik_subgroup_allocators_;
  
};
}

#endif
