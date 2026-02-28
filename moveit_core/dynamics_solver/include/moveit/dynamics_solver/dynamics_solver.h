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

/* Author: Sachin Chitta */

#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace dynamics_solver
{

MOVEIT_CLASS_FORWARD(DynamicsSolver);

class DynamicsSolver
{
public:
  DynamicsSolver(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                 const geometry_msgs::Vector3& gravity_vector);

  bool getTorques(const std::vector<double>& joint_angles, const std::vector<double>& joint_velocities,
                  const std::vector<double>& joint_accelerations, const std::vector<geometry_msgs::Wrench>& wrenches,
                  std::vector<double>& torques) const;

  bool getMaxPayload(const std::vector<double>& joint_angles, double& payload, unsigned int& joint_saturated) const;

  bool getPayloadTorques(const std::vector<double>& joint_angles, double payload,
                         std::vector<double>& joint_torques) const;

  const std::vector<double>& getMaxTorques() const;

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  const moveit::core::JointModelGroup* getGroup() const
  {
    return joint_model_group_;
  }

private:
  bool initialize();
  void computeMaxTorques();
  void updateRobotState(const std::vector<double>& joint_angles, const std::vector<double>& joint_velocities,
                        const std::vector<double>& joint_accelerations) const;
  void updateExternalWrenches(const std::vector<geometry_msgs::Wrench>& wrenches) const;

  std::shared_ptr<KDL::ChainIdSolver_RNE> chain_id_solver_;
  KDL::Chain kdl_chain_;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;

  mutable moveit::core::RobotStatePtr state_;
  mutable std::vector<geometry_msgs::Wrench> external_wrenches_;

  std::string base_name_, tip_name_;
  unsigned int num_joints_, num_segments_;
  std::vector<double> max_torques_;
  double gravity_;
};

}  // namespace dynamics_solver

