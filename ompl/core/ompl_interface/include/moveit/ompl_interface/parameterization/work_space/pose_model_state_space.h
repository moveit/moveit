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

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

namespace ompl_interface
{

class PoseModelStateSpace : public ModelBasedStateSpace
{
public:

  static const std::string PARAMETERIZATION_TYPE;

  /// Add a tag to the state representation
  class StateType : public ModelBasedStateSpace::StateType
  {
  public:
    enum 
      {
        JOINTS_COMPUTED = 256,
        POSE_COMPUTED = 512
      };
    
    StateType() : ModelBasedStateSpace::StateType()
    {
      flags |= JOINTS_COMPUTED;
    }    

    bool jointsComputed() const
    {
      return flags & JOINTS_COMPUTED;
    }
    
    bool poseComputed() const
    {
      return flags & POSE_COMPUTED;
    }
    
    void setJointsComputed(bool value)
    {
      if (value)
        flags |= JOINTS_COMPUTED;
      else
        flags &= ~JOINTS_COMPUTED;
    }
    
    void setPoseComputed(bool value)
    {
      if (value)
        flags |= POSE_COMPUTED;
      else
        flags &= ~POSE_COMPUTED;
    }
    
  };
  
  PoseModelStateSpace(const ModelBasedStateSpaceSpecification &spec);
  virtual ~PoseModelStateSpace();
  
  virtual ompl::base::State* allocState() const;
  virtual void freeState(ompl::base::State *state) const;  
  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
  virtual double getMaximumExtent() const;

  bool computeStateFK(ompl::base::State *state) const;
  bool computeStateIK(ompl::base::State *state) const;
  bool computeStateK(ompl::base::State *state) const;

  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
  virtual void copyToOMPLState(ompl::base::State *state, const robot_state::JointStateGroup* jsg) const;
  virtual void sanityChecks() const;
  
private:

  struct PoseComponent
  {
    PoseComponent(const robot_model::JointModelGroup *subgroup);
    
    bool computeStateFK(const ompl::base::StateSpace *full_state_space, ompl::base::State *full_state, ompl::base::State *state) const;
    bool computeStateIK(const ompl::base::StateSpace *full_state_space, ompl::base::State *full_state, ompl::base::State *state) const;
    
    bool operator<(const PoseComponent &o) const
    {
      return subgroup_->getName() < o.subgroup_->getName();
    }
    
    const robot_model::JointModelGroup *subgroup_;    
    boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
    ompl::base::StateSpacePtr state_space_;
    std::vector<std::string> fk_link_;
    std::vector<std::string> joint_names_;
    std::vector<unsigned int> joint_val_count_;
    unsigned int variable_count_;
  };
  
  void constructSpaceFromPoses();
  virtual void afterStateSample(ompl::base::State *sample) const;

  std::vector<PoseComponent> poses_;
};

}

#endif
