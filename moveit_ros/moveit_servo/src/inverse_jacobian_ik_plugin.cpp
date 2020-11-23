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

#include <moveit_servo/inverse_jacobian_ik_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

static const std::string LOGNAME = "inverse_jacobian_ik_plugin";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace moveit_servo
{
bool InverseJacobianIKPlugin::initialize(ros::NodeHandle& nh)
{
  // ROS Server for allowing drift in some dimensions
  drift_dimensions_server_ = nh.advertiseService(ros::names::append(nh.getNamespace(), "change_drift_dimensions"),
                                                 &InverseJacobianIKPlugin::changeDriftDimensions, this);

  // Retrieve singularity thresholds from yaml

  // Optional parameter sub-namespace specified in the launch file. All other parameters will be read from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace within the node namespace, append the parameter namespace
  // to load the parameters correctly.
  ros::NodeHandle ik_nh = parameter_ns.empty() ? nh : ros::NodeHandle(nh, parameter_ns);

  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, ik_nh, "lower_singularity_threshold", lower_singularity_threshold_);
  error += !rosparam_shortcuts::get(LOGNAME, ik_nh, "hard_stop_singularity_threshold", hard_stop_singularity_threshold_);

  if (hard_stop_singularity_threshold_ < lower_singularity_threshold_)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_singularity_threshold' "
                            "should be greater than 'lower_singularity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((hard_stop_singularity_threshold_ < 0.) || (lower_singularity_threshold_ < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_singularity_threshold' "
                            "and 'lower_singularity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }

  rosparam_shortcuts::shutdownIfError(LOGNAME, error);
  return true;
}

bool InverseJacobianIKPlugin::doDifferentialIK(const moveit::core::RobotStatePtr& current_state,
                                               Eigen::VectorXd& delta_x,
                                               const moveit::core::JointModelGroup* joint_model_group,
                                               const double loop_period, double& velocity_scaling_for_singularity,
                                               Eigen::ArrayXd& delta_theta, StatusCode& status)
{
  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = current_state->getJacobian(joint_model_group);

  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
  for (auto dimension = jacobian.rows() - 1; dimension >= 0; --dimension)
  {
    if (drift_dimensions_[dimension] && jacobian.rows() > 1)
    {
      removeDimension(jacobian, delta_x, dimension);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  delta_theta = pseudo_inverse * delta_x;

  enforceVelLimits(delta_theta, loop_period, joint_model_group);

  velocity_scaling_for_singularity =
      velocityScalingFactorForSingularity(current_state, joint_model_group, delta_x, svd, pseudo_inverse, status);

  return true;
}

void InverseJacobianIKPlugin::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x,
                                              unsigned int row_to_remove)
{
  unsigned int num_rows = jacobian.rows() - 1;
  unsigned int num_cols = jacobian.cols();

  if (row_to_remove < num_rows)
  {
    jacobian.block(row_to_remove, 0, num_rows - row_to_remove, num_cols) =
        jacobian.block(row_to_remove + 1, 0, num_rows - row_to_remove, num_cols);
    delta_x.segment(row_to_remove, num_rows - row_to_remove) =
        delta_x.segment(row_to_remove + 1, num_rows - row_to_remove);
  }
  jacobian.conservativeResize(num_rows, num_cols);
  delta_x.conservativeResize(num_rows);
}

void InverseJacobianIKPlugin::enforceVelLimits(Eigen::ArrayXd& delta_theta, double loop_period,
                                               const moveit::core::JointModelGroup* joint_model_group)
{
  Eigen::ArrayXd velocity = delta_theta / loop_period;

  std::size_t joint_delta_index = 0;
  // Track the smallest velocity scaling factor required, across all joints
  double velocity_limit_scaling_factor = 1;

  for (auto joint : joint_model_group->getActiveJointModels())
  {
    // Some joints do not have bounds defined
    const auto bounds = joint->getVariableBounds(joint->getName());

    if (bounds.velocity_bounded_)
    {
      velocity(joint_delta_index) = delta_theta(joint_delta_index) / loop_period;

      bool clip_velocity = false;
      double velocity_limit = 0.0;
      if (velocity(joint_delta_index) < bounds.min_velocity_)
      {
        clip_velocity = true;
        velocity_limit = bounds.min_velocity_;
      }
      else if (velocity(joint_delta_index) > bounds.max_velocity_)
      {
        clip_velocity = true;
        velocity_limit = bounds.max_velocity_;
      }

      // Apply velocity bounds
      if (clip_velocity)
      {
        const double scaling_factor = fabs(velocity_limit * loop_period) / fabs(delta_theta(joint_delta_index));

        // Store the scaling factor if it's the smallest yet
        if (scaling_factor < velocity_limit_scaling_factor)
          velocity_limit_scaling_factor = scaling_factor;
      }
    }
    ++joint_delta_index;
  }

  // Apply the velocity scaling to all joints
  if (velocity_limit_scaling_factor < 1)
  {
    for (joint_delta_index = 0; joint_delta_index < joint_model_group->getActiveJointModels().size();
         ++joint_delta_index)
    {
      delta_theta(joint_delta_index) = velocity_limit_scaling_factor * delta_theta(joint_delta_index);
      velocity(joint_delta_index) = velocity_limit_scaling_factor * velocity(joint_delta_index);
    }
  }
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double InverseJacobianIKPlugin::velocityScalingFactorForSingularity(
    const moveit::core::RobotStatePtr& current_state, const moveit::core::JointModelGroup* joint_model_group,
    const Eigen::VectorXd& commanded_velocity, const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
    const Eigen::MatrixXd& pseudo_inverse, StatusCode& status)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(num_dimensions - 1);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(num_dimensions);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  current_state->copyJointGroupPositions(joint_model_group, new_theta);
  new_theta += pseudo_inverse * delta_x;
  current_state->setJointGroupPositions(joint_model_group, new_theta);
  Eigen::MatrixXd new_jacobian = current_state->getJacobian(joint_model_group);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==> decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between lower_singularity_threshold and
    // hard_stop_singularity_threshold, and we're moving towards the singularity
    if ((ini_condition > lower_singularity_threshold_) && (ini_condition < hard_stop_singularity_threshold_))
    {
      velocity_scale = 1. - (ini_condition - lower_singularity_threshold_) /
                                (hard_stop_singularity_threshold_ - lower_singularity_threshold_);
      status = StatusCode::DECELERATE_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status));
    }

    // Very close to singularity, so halt.
    else if (ini_condition > 50 /*parameters_.hard_stop_singularity_threshold*/)
    {
      velocity_scale = 0;
      status = StatusCode::HALT_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status));
    }
  }

  return velocity_scale;
}

bool InverseJacobianIKPlugin::changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                                                    moveit_msgs::ChangeDriftDimensions::Response& res)
{
  drift_dimensions_[0] = req.drift_x_translation;
  drift_dimensions_[1] = req.drift_y_translation;
  drift_dimensions_[2] = req.drift_z_translation;
  drift_dimensions_[3] = req.drift_x_rotation;
  drift_dimensions_[4] = req.drift_y_rotation;
  drift_dimensions_[5] = req.drift_z_rotation;

  res.success = true;
  return true;
}
}  // namespace moveit_servo

// Declare availability as a ROS plugin
PLUGINLIB_EXPORT_CLASS(moveit_servo::InverseJacobianIKPlugin, moveit_servo::IKSolverBase)
