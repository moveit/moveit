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

#include <ompl/base/StateSpace.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
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
  planning_scene::KinematicsAllocatorFn kinematics_allocator_;
  planning_scene::KinematicsAllocatorMapFn kinematics_subgroup_allocators_;
};

class ModelBasedStateSpace : public ompl::base::StateSpace
{
public:
  
  class StateType : public ompl::base::State
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
    
    StateType(void) : ompl::base::State(), flags(0), distance(0.0), tag(-1)
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
    
    void clearKnownInformation(void)
    {
      flags &= ~(VALIDITY_KNOWN | VALIDITY_TRUE | GOAL_DISTANCE_KNOWN | IS_START_STATE | IS_GOAL_STATE);
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
    
    std::vector<planning_models::KinematicState::JointState*> joint_states;
    int    flags;
    double distance;
    int    tag;
  };

  ModelBasedStateSpace(const ModelBasedStateSpaceSpecification &spec);
  
  virtual ~ModelBasedStateSpace(void);
  
  virtual ompl::base::State* allocState(void) const
  {
    StateType *st = new StateType();
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
      st->joint_states.push_back(new planning_models::KinematicState::JointState(joint_model_vector[i]));
    return st;
  }
  
  virtual void freeState(ompl::base::State *state) const
  {
    std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    for (std::size_t i = 0 ; i < joint_states.size() ; ++i)
      delete joint_states[i];
    delete state->as<StateType>();
  }
  
  virtual unsigned int getDimension(void) const
  {
    unsigned int dim  = 0;
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
      dim += joint_model_vector[i]->getStateSpaceDimension();
    return dim;
  }
  
  virtual double getMaximumExtent(void) const
  {  
    double e = 0.0;
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
      e += joint_model_vector[i]->getDistanceFactor() * joint_model_vector[i]->getMaximumExtent();
    return e;
  }
  
  virtual void enforceBounds(ompl::base::State *state) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();  
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
      joint_model_vector[i]->enforceBounds(joint_states[i]->getVariableValues());
  }
  
  virtual bool satisfiesBounds(const ompl::base::State *state) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();  
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
      if (!joint_model_vector[i]->satisfiesBounds(joint_states[i]->getVariableValues()))
        return false;
    return true;
  }
  
  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &d = destination->as<StateType>()->joint_states;
    const std::vector<planning_models::KinematicState::JointState*> &s = source->as<StateType>()->joint_states;
    assert(s.size() == d.size());
    for (std::size_t i = 0 ; i < d.size() ; ++i)
      *d[i] = *s[i];
  }
  
  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const
  {
    double d = 0.0;
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
    const std::vector<planning_models::KinematicState::JointState*> &joint_states1 = state1->as<StateType>()->joint_states;
    const std::vector<planning_models::KinematicState::JointState*> &joint_states2 = state2->as<StateType>()->joint_states;
    for (std::size_t i = 0 ; i < joint_states1.size() ; ++i)
      d += joint_model_vector[i]->getDistanceFactor() * joint_states1[i]->distance(joint_states2[i]);
    return d;
  }
  
  virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &joint_states1 = state1->as<StateType>()->joint_states;
    const std::vector<planning_models::KinematicState::JointState*> &joint_states2 = state2->as<StateType>()->joint_states;
    if (joint_states1.size() != joint_states2.size())
      return false;
    for (std::size_t i = 0 ; i < joint_states1.size() ; ++i)
      if (joint_states1[i]->getVariableValues() != joint_states2[i]->getVariableValues())
        return false;
    return true;
  }
  
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
  {  
    const std::vector<planning_models::KinematicState::JointState*> &from_states = from->as<StateType>()->joint_states;
    const std::vector<planning_models::KinematicState::JointState*> &to_states = to->as<StateType>()->joint_states;
    const std::vector<planning_models::KinematicState::JointState*> &dest_states = state->as<StateType>()->joint_states;
    for (std::size_t i = 0 ; i < from_states.size() ; ++i)
      from_states[i]->interpolate(to_states[i], t, dest_states[i]);
  }
  
  virtual unsigned int getSerializationLength(void) const
  {
    return sizeof(double) * spec_.joint_model_group_->getVariableCount();
  }
  
  virtual void serialize(void *serialization, const ompl::base::State *state) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    unsigned int L = 0;
    for (std::size_t i = 0 ; i < joint_states.size() ; ++i)
    {
      const std::vector<double> &vals = joint_states[i]->getVariableValues();
      const unsigned int l = vals.size() * sizeof(double);
      memcpy(reinterpret_cast<char*>(serialization) + L, &vals[0], l);
      L += l;
    }
  }
  
  virtual void deserialize(ompl::base::State *state, const void *serialization) const
  {
    const std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    unsigned int L = 0;
    for (std::size_t i = 0 ; i < joint_states.size() ; ++i)
    {
      std::vector<double> &vals = joint_states[i]->getVariableValues();
      const unsigned int l = vals.size() * sizeof(double);
      memcpy(&vals[0], reinterpret_cast<const char*>(serialization) + L, l);
      L += l;
    }
  }
  
  virtual double* getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const
  {
    const std::vector<const planning_models::KinematicModel::JointModel*> &joint_model_vector = spec_.joint_model_group_->getJointModels();
    unsigned int tindex = 0;
    for (std::size_t i = 0 ; i < joint_model_vector.size() ; ++i)
    {
      unsigned int pindex = tindex;
      tindex += joint_model_vector[i]->getVariableCount();
      if (tindex > index)
        return &(state->as<StateType>()->joint_states[i]->getVariableValues()[index - pindex]);
    }
    return NULL;
  }
  
  virtual void printState(const ompl::base::State *state, std::ostream &out) const
  {
    out << "JointStateGroup(" << spec_.joint_model_group_->getName() << ") = [" << std::endl;
    const std::vector<planning_models::KinematicState::JointState*> &joint_states = state->as<StateType>()->joint_states;
    for (std::size_t i = 0 ; i < joint_states.size() ; ++i)
    {
      out << joint_states[i]->getName() << " = [ ";
      for (std::size_t j = 0 ; j < joint_states[i]->getVariableValues().size() ; ++j)
        out << joint_states[i]->getVariableValues()[j] << " ";
      out << "]" << std::endl;
    }
    out << "]" << std::endl;
  }
  
  virtual void printSettings(std::ostream &out) const
  {
    out << "ModelBasedStateSpace '" << getName() << "' at " << this << " using the following joints:" << std::endl;
    spec_.joint_model_group_->printGroupInfo(out);
    printProjections(out);
  }
  
  //  virtual ompl::base::StateSamplerPtr allocStateSampler(void) const;
  
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
  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
  
  const planning_scene::KinematicsAllocatorFn& getKinematicsAllocator(void) const
  {
    return spec_.kinematics_allocator_;
  }
  
  const planning_scene::KinematicsAllocatorMapFn& getKinematicsSubgroupAllocators(void) const
  {
    return spec_.kinematics_subgroup_allocators_;
  }
  
protected:
  
  /** \brief Function that is called for every sampled state (before the sampling process) */
  //  virtual void beforeStateSample(ompl::base::State *sampled) const;
  
  /** \brief Function that is called for every sampled state (at the completion of the sampling process) */
  //  virtual void afterStateSample(ompl::base::State *sampled) const;
  
  ModelBasedStateSpaceSpecification spec_; 
  
private:
  
  //  class WrappedStateSampler;
  //  friend class WrappedStateSampler;
  
};

typedef boost::shared_ptr<ModelBasedStateSpace> ModelBasedStateSpacePtr;

}

#endif
