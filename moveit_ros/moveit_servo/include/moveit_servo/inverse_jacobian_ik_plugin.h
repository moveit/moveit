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
   Desc: A plugin providing an IK solution
*/

#pragma once

#include <moveit_msgs/ChangeDriftDimensions.h>
#include <moveit_servo/ik_solver_base.h>
#include <moveit_servo/status_codes.h>

namespace moveit_servo
{
// Inherit from a generic base class
class InverseJacobianIKPlugin : public IKSolverBase
{
public:
  bool initialize(ros::NodeHandle& nh) override;

  bool doDifferentialIK(const moveit::core::RobotStatePtr& current_state, Eigen::VectorXd& twist_cmd,
                        const moveit::core::JointModelGroup* joint_model_group, const double loop_period,
                        double& velocity_scaling_for_singularity, Eigen::ArrayXd& delta_theta,
                        StatusCode& status) override;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, unsigned int row_to_remove);

  /** \brief  Scale the delta theta to match joint velocity/acceleration limits */
  void enforceVelLimits(Eigen::ArrayXd& delta_theta, double loop_period,
                        const moveit::core::JointModelGroup* joint_model_group);

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
  double velocityScalingFactorForSingularity(const moveit::core::RobotStatePtr& current_state,
                                             const moveit::core::JointModelGroup* joint_model_group,
                                             const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& pseudo_inverse, StatusCode& status);

private:
  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  bool changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                             moveit_msgs::ChangeDriftDimensions::Response& res);

  ros::ServiceServer drift_dimensions_server_;
  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> drift_dimensions_ = { { false, false, false, false, false, false } };
  double lower_singularity_threshold_;
  double hard_stop_singularity_threshold_;
};
}  // namespace moveit_servo
