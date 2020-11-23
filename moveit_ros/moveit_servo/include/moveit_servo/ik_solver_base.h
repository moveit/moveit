/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
/*
   Author: Andy Zelenak
   Desc: A plugin providing an IK solution.
   The plugin can be any IK method that takes a current RobotState and TwistStamped command
   and outputs an incremental trajectory_msgs::JointTrajectory
*/

#pragma once

#include <geometry_msgs/TwistStamped.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_servo/status_codes.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace moveit_servo
{
// Base class for incremental IK solvers.
// If your IK solver requires something different, please raise a Github issue or notify a maintainer.
class IKSolverBase
{
public:
  virtual void initialize(ros::NodeHandle& nh){}

  /**
   * From a Cartesian delta command and the current robot state, compute a new JointTrajectory message with an
   * incremental change in joint values.
   * The idea is to add new function signatures here as different IK solvers are added.
   * @param current_state current state of the robot from MoveIt
   * @param delta_x a vector with 6 elements (delta-x, delta-y, delta-z, delta-roll, delta-pitch, delta-yaw)
   * @return true if calculations were successful
   */
  virtual bool doIncrementalIK(const moveit::core::RobotStatePtr& current_state, Eigen::VectorXd& delta_x,
                               const moveit::core::JointModelGroup* joint_model_group,
                               double loop_period, double& velocity_scaling_for_singularity,
                               Eigen::ArrayXd& delta_theta,
                               StatusCode& status)
  {
    return true;
  }

protected:
  IKSolverBase(){}
};
}  // namespace moveit_servo
