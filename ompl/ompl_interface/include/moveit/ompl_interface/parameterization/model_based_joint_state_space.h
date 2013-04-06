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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_JOINT_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_JOINT_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace ompl_interface
{

class ModelBasedJointStateSpace : public ompl::base::StateSpace
{
public:
  
  class StateType : public ompl::base::State
  {
  public:
    robot_state::JointState *joint_state;
  };
  
  ModelBasedJointStateSpace(const robot_model::JointModel *joint_model);
  ModelBasedJointStateSpace(const robot_model::JointModel *joint_model,
                            const robot_model::JointModel::Bounds &joint_bounds);
  
  virtual ~ModelBasedJointStateSpace();
  
  virtual ompl::base::State* allocState() const;
  virtual void freeState(ompl::base::State *state) const;
  
  virtual unsigned int getDimension() const;
  virtual double getMaximumExtent() const;
  virtual void enforceBounds(ompl::base::State *state) const;
  virtual bool satisfiesBounds(const ompl::base::State *state) const;
  
  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;
  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
  virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
  
  virtual unsigned int getSerializationLength() const;
  virtual void serialize(void *serialization, const ompl::base::State *state) const;
  virtual void deserialize(ompl::base::State *state, const void *serialization) const;
  virtual double* getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const;
  virtual void printState(const ompl::base::State *state, std::ostream &out) const;
  virtual void printSettings(std::ostream &out) const;
  
  virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;  
  
  const robot_model::JointModel* getJointModel() const
  {
    return joint_model_;
  }  
  
  const std::string& getJointName() const
  {
    return getJointModel()->getName();
  }
  
  /// Set the planning volume for the possible SE2 and/or SE3 components of the state space
  void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
  
  const robot_model::JointModel::Bounds& getJointBounds() const
  {
    return joint_bounds_;
  }
  
protected:

  void propagateJointStateUpdate(ompl::base::State *state) const;

  const robot_model::JointModel *joint_model_;
  robot_model::JointModel::Bounds joint_bounds_;
  
};

}

#endif
