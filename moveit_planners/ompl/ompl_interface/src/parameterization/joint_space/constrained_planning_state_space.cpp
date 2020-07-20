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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Jeroen De Maeyer */

#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

const std::string ompl_interface::ConstrainedPlanningStateSpace::PARAMETERIZATION_TYPE =
    "ConstrainedPlanningJointModel";

ompl_interface::ConstrainedPlanningStateSpace::ConstrainedPlanningStateSpace(
    const ModelBasedStateSpaceSpecification& spec)
  : ModelBasedStateSpace(spec)
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);
}

void ompl_interface::ConstrainedPlanningStateSpace::copyToRobotState(moveit::core::RobotState& rstate,
                                                                     const ompl::base::State* state) const
{
  rstate.setJointGroupPositions(
      spec_.joint_model_group_,
      state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->values);
  rstate.update();
}

void ompl_interface::ConstrainedPlanningStateSpace::copyToOMPLState(ompl::base::State* state,
                                                                    const moveit::core::RobotState& rstate) const
{
  rstate.copyJointGroupPositions(
      spec_.joint_model_group_,
      state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->values);
  // clear any cached info (such as validity known or not)
  state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->clearKnownInformation();
}

void ompl_interface::ConstrainedPlanningStateSpace::copyJointToOMPLState(ompl::base::State* state,
                                                                         const moveit::core::RobotState& robot_state,
                                                                         const moveit::core::JointModel* joint_model,
                                                                         int ompl_state_joint_index) const
{
  // Copy one joint (multiple variables possibly)
  // no need to cast the state as ConstrainedPlanningStateSpace also implements getValueAddressAtIndex
  memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
         robot_state.getVariablePositions() + joint_model->getFirstVariableIndex() * sizeof(double),
         joint_model->getVariableCount() * sizeof(double));

  // clear any cached info (such as validity known or not)
  state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->clearKnownInformation();
}