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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_WORK_SPACE_POSE_MODEL_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_WORK_SPACE_POSE_MODEL_STATE_SPACE_

#include "ompl_interface/parameterization/model_based_state_space.h"
#include "ompl_interface/parameterization/joint_space/joint_model_state_space_helper.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl_interface
{

class PoseModelStateSpace : public ModelBasedStateSpace
{
public:

  /// Add a tag to the state representation
  class StateType : public ModelBasedStateSpace::StateType
  {
  public:
    StateType(void) : ModelBasedStateSpace::StateType(), joints_computed(true), pose_computed(false)
    {
    }    
    bool joints_computed;
    bool pose_computed;
  };
  
  PoseModelStateSpace(const ModelBasedStateSpaceSpecification &spec);

  virtual ~PoseModelStateSpace(void)
  {
  }
  
  virtual ob::State* allocState(void) const;
  virtual void freeState(ob::State *state) const;  
  virtual void copyState(ob::State *destination, const ob::State *source) const;
  virtual void interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const;
  
  virtual void copyToKinematicState(const std::vector<pm::KinematicState::JointState*> &js, const ob::State *state) const;
  virtual void copyToOMPLState(ob::State *state, const std::vector<pm::KinematicState::JointState*> &js) const;
  virtual void copyToOMPLState(ob::State *state, const std::vector<double> &values) const;
  
private:
  
  struct PoseComponent
  {
    PoseComponent(const pm::KinematicModel::JointModelGroup *subgroup, 
                  const kc::IKAllocator &kinematics_allocator);
    
    bool computeStateFK(ob::State *state) const;
    bool computeStateIK(ob::State *state) const;

    const pm::KinematicModel::JointModelGroup *subgroup_;    
    boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
    JointModelStateSpaceHelper joint_model_;
    ob::SE3StateSpace *se3_component_;
    ob::StateSpacePtr state_space_;
    std::vector<std::string> fk_link_;
    std::vector<std::string> joint_names_;
    std::vector<unsigned int> joint_val_count_;
    unsigned int variable_count_;
  };
  
  void constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                      const kc::IKAllocator &ik_allocator);
  void constructSpace(const pm::KinematicModel::JointModelGroup *group, 
                      const kc::IKSubgroupAllocator &ik_allocator);
  void constructSpaceFromPoses(void);

  bool computeStateFK(ob::State *state) const;
  bool computeStateIK(ob::State *state) const;
  bool computeStateK(ob::State *state) const;

  
  std::vector<PoseComponent> poses_;
};

}

#endif
