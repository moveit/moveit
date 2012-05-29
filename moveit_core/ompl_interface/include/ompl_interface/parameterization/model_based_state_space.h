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

/* Author: Ioan Sucan */

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_

#include "ompl_interface/parameterization/model_based_joint_state_space.h"
#include <kinematic_constraints/kinematic_constraint.h>
#include <constraint_samplers/constraint_sampler.h>

namespace ompl_interface
{

struct ModelBasedStateSpaceSpecification
{
  ModelBasedStateSpaceSpecification(const planning_models::KinematicModelConstPtr &kmodel,
                                    const planning_models::KinematicModel::JointModelGroup *jmg) :
    kmodel_(kmodel), joint_model_group_(jmg)
  {
  }
  
  ModelBasedStateSpaceSpecification(const planning_models::KinematicModelConstPtr &kmodel,
                                    const std::string &group_name) :
    kmodel_(kmodel), joint_model_group_(kmodel_->getJointModelGroup(group_name))
  {
    if (!joint_model_group_)
      throw std::runtime_error("Group '" + group_name + "'  was not found");
  }
  
  planning_models::KinematicModelConstPtr kmodel_;
  const planning_models::KinematicModel::JointModelGroup *joint_model_group_;
  std::vector<planning_models::KinematicModel::JointModel::Bounds> joints_bounds_;
};

class ModelBasedStateSpace : public ompl::base::CompoundStateSpace
{
public:
  
  class StateType : public ompl::base::CompoundState
  {
  public:
    
    enum
      {
        VALIDITY_KNOWN = 1,
        GOAL_DISTANCE_KNOWN = 2,
        VALIDITY_TRUE = 4,
        IS_START_STATE = 8,
        IS_GOAL_STATE = 16
      };
    
    StateType(void) : ompl::base::CompoundState(), flags(0), distance(0.0), tag(-1)
    {
    }
    
    void markValid(double d)
    {
      distance = d; 
      flags |= GOAL_DISTANCE_KNOWN;
      markValid();
    }
    
    void markValid(void)
    {
      flags |= (VALIDITY_KNOWN | VALIDITY_TRUE);
    }
    
    void markInvalid(double d)
    {
      distance = d;
      flags |= GOAL_DISTANCE_KNOWN;
      markInvalid();
    }
    
    void markInvalid(void)
    {
      flags &= ~VALIDITY_TRUE;
      flags |= VALIDITY_KNOWN;
    }
    
    bool isValidityKnown(void) const
    {
      return flags & VALIDITY_KNOWN;
    }
    
    virtual void clearKnownInformation(void)
    {
      flags = 0;
    }
    
    bool isMarkedValid(void) const
    {
      return flags & VALIDITY_TRUE;
    }
    
    bool isGoalDistanceKnown(void) const
    {
      return flags & GOAL_DISTANCE_KNOWN;
    }
    
    bool isStartState(void) const
    {
      return flags & IS_START_STATE;
    }
    
    bool isGoalState(void) const
    {
      return flags & IS_GOAL_STATE;
    }
    
    bool isInputState(void) const
    {   
      return flags & (IS_START_STATE | IS_GOAL_STATE);
    }
    
    void markStartState(void)
    {
      flags |= IS_START_STATE;
    }
    
    void markGoalState(void)
    {
      flags |= IS_GOAL_STATE;
    }
    
    int    flags;
    double distance;
    int    tag;
  };
  
  ModelBasedStateSpace(const ModelBasedStateSpaceSpecification &spec);  
  virtual ~ModelBasedStateSpace(void);

  virtual ompl::base::State* allocState(void) const;
  virtual void freeState(ompl::base::State *state) const;
  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
  virtual ompl::base::StateSamplerPtr allocDefaultStateSampler(void) const;  

  const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
  {
    return spec_.kmodel_;
  }
  
  const planning_models::KinematicModel::JointModelGroup* getJointModelGroup(void) const
  {
    return spec_.joint_model_group_;
  }  
  
  const std::string& getJointModelGroupName(void) const
  {
    return getJointModelGroup()->getName();
  }
  
  const ModelBasedStateSpaceSpecification& getSpecification(void) const
  {
    return spec_;
  }
  
  /// Set the planning volume for the possible SE2 and/or SE3 components of the state space
  virtual void setBounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
  
  const std::vector<planning_models::KinematicModel::JointModel::Bounds>& getJointsBounds(void) const
  {
    return spec_.joints_bounds_;
  }
  
  /// Copy the data from an OMPL state to a set of joint states. The join states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToKinematicState(planning_models::KinematicState::JointStateGroup* jsg, const ompl::base::State *state) const;
  
  /// Copy the data from an OMPL state to a kinematic state. The join states \b must be specified in the same order as the joint models in the constructor. This function is implemented in terms of the previous definition with the same name.
  void copyToKinematicState(planning_models::KinematicState &kstate, const ompl::base::State *state) const
  {
    copyToKinematicState(kstate.getJointStateGroup(getJointModelGroupName()), state);
  }
  
  /// Copy the data from a value vector that corresponds to the state of the considered joint model group (or array of joints)
  //  virtual void copyToOMPLState(ob::State *state, const std::vector<double> &values) const = 0;

  /// Copy the data from a kinematic state to an OMPL state. Only needed joint states are copied. This function is implemented in terms of the previous definition with the same name.
  void copyToOMPLState(ompl::base::State *state, const planning_models::KinematicState &kstate) const
  {
    copyToOMPLState(state, kstate.getJointStateGroup(getJointModelGroupName()));
  }  
        
  /// Copy the data from a set of joint states to an OMPL state. The join states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToOMPLState(ompl::base::State *state, const planning_models::KinematicState::JointStateGroup* jsg) const;
  
  double getTagSnapToSegment(void) const;
  void setTagSnapToSegment(double snap);
  
protected:
  
  ModelBasedStateSpaceSpecification spec_; 
  
  double tag_snap_to_segment_;
  double tag_snap_to_segment_complement_;
    
};

typedef boost::shared_ptr<ModelBasedStateSpace> ModelBasedStateSpacePtr;

}

#endif
