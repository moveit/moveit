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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_JOINT_SPACE_JOINT_MODEL_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_JOINT_SPACE_JOINT_MODEL_STATE_SPACE_

#include "ompl_interface/parameterization/model_based_state_space.h"
#include "ompl_interface/parameterization/joint_space/joint_model_state_space_helper.h"

namespace ompl_interface
{

class JointModelStateSpace : public ModelBasedStateSpace
{
public:
  
  JointModelStateSpace(const ModelBasedStateSpaceSpecification &spec);
  
  virtual void copyToKinematicState(pm::KinematicState::JointStateGroup *jsg, const ob::State *state) const
  {
    state_space_tree_constr_->state_space_tree_->copyToKinematicState(jsg, state);
  }
  
  virtual void copyToOMPLState(ob::State *state, const pm::KinematicState::JointStateGroup* jsg) const
  { 
    state_space_tree_constr_->state_space_tree_->copyToOMPLState(state, jsg);
  }
  
private:
  
  struct StateSpaceTreeConstructor;
  
  struct StateSpaceTree
  {
    StateSpaceTree(void)
    {
    }
    
    virtual ~StateSpaceTree(void)
    {
    } 
    
    virtual void copyToKinematicState(pm::KinematicState::JointStateGroup* jsg, const ob::State *state) const = 0;
    virtual void copyToOMPLState(ob::State *state, const pm::KinematicState::JointStateGroup* jsg) const = 0;

    ob::StateSpacePtr state_space_;
  };
  
  struct StateSpaceTreeLeaf : public StateSpaceTree
  {    
    StateSpaceTreeLeaf(const pm::KinematicState::JointStateGroup* jmg) : StateSpaceTree(), helper_(jmg->getJointModels())
    {
      state_space_ = helper_.getStateSpace();
      if (state_space_)
        state_space_->lock();
    }
    
    virtual void copyToKinematicState(pm::KinematicState::JointStateGroup* jsg, const ob::State *state) const
    {    
      helper_.copyToKinematicState(js->getJointStateVector(), state);
    }
    
    virtual void copyToOMPLState(ob::State *state, const pm::KinematicState::JointStateGroup* jsg) const
    {    
      helper_.copyToOMPLState(state, jsg->getJointStateVector());
    }
    
    JointModelStateSpaceHelper helper_;
  };
  
  struct StateSpaceTreeNode : public StateSpaceTree
  {
    StateSpaceTreeNode(StateSpaceTreeConstructor *constr, const pm::KinematicModel::JointModelGroup *jmg) : StateSpaceTree()
    {
      ob::CompoundStateSpace *result = new ob::CompoundStateSpace();
      
      const std::vector<std::string> &sg = jmg->getDisjointSubgroupNames();
      if (sg.empty())
      {
        children_.push_back(new StateSpaceTreeLeaf(jmg));
        if (children_[0].state_space_)
          result->addSubspace(children_[0].state_space_, 1.0);
      }
      else
      {
        std::set<const pm::KinematicModel::JointModel*> inc_joints;
        inc_joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());
        
        for (std::size_t i = 0 ; i < sg.size() ; ++i)
        {
          const pm::KinematicModel::JointModelGroup *jmg_i = kmodel_->getJointModelGroup(sg[i]);
          StateSpaceTree *sst = constr->constructStateSpaceTree(constr, jmg_i;)
          children_.push_back(sst);
          if (sst->state_space_)
            result->addSubspace(sst->state_space_, (double)sst->state_space_->getDimension());
          
          for (std::size_t k = 0 ; k < jmg_i->getJointModels().size() ; ++k)
            inc_joints.erase(jmg_i->getJointModels()[k]);
        }
        if (!inc_joints.empty())
        {
          std::vector<const pm::KinematicModel::JointModel*> remaining_joints;
          for (std::set<const pm::KinematicModel::JointModel*>::const_iterator it = inc_joints.begin() ; it != inc_joints.end() ; ++it)
            remaining_joints.push_back(*it);
          StateSpaceTree *sst = new StateSpaceTreeLeaf(remaining_joints);
          children_.push_back(sst);        
          if (sst->state_space_)
            result->addSubspace(sst->state_space_, (double)sst->state_space_->getDimension());
        }
      }
      if (result->getSubspaceCount() > 0)
      {
        result->setName(jmg->getName());
        result->lock();
        state_space_.reset(result);
      }
      else
        delete result;
    }
    
    ~StateSpaceTreeNode(void)
    {
      for (std::size_t i = 0 ; i < children_.size() ; ++i)
        delete children_[i];
    }

    virtual void copyToKinematicState(pm::KinematicState::JointStateGroup* jsg, const ob::State *state) const
    {    

    }
    
    virtual void copyToOMPLState(ob::State *state, const pm::KinematicState::JointStateGroup* jsg) const
    {

    }
    
    std::vector<StateSpaceTree*> children_;
  };

  struct StateSpaceTreeConstructor
  {
    StateSpaceTreeConstructor(const pm::KinematicModel::JointModelGroup *jmg)
    {
      state_space_tree_ = constructStateSpaceTree(this, jmg);
    }
    
    ~StateSpaceTreeConstructor(void)
    {
      delete state_space_tree_;
    }
    
    StateSpaceTree* constructStateSpaceTree(StateSpaceTreeConstructor *constr, const pm::KinematicModel::JointModelGroup *jmg)
    {
      if (jmg->getDisjointSubgroupNames().empty())
        return new StateSpaceTreeLeaf(jmg->getJointModels());
      return new StateSpaceTreeNode(constr, jmg);
    }
    
    StateSpaceTree *state_space_tree_;
  };
  
  StateSpaceTreeConstructor state_space_tree_constr_;
  
};

}

#endif
