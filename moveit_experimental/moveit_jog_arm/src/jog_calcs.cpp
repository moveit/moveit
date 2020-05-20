/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*      Title     : jog_calcs.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <moveit_jog_arm/jog_calcs.h>

static const std::string LOGNAME = "jog_calcs";

namespace moveit_jog_arm
{
// Constructor for the class that handles jogging calculations
JogCalcs::JogCalcs(const JogArmParameters& parameters, const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr)
  : parameters_(parameters), default_sleep_rate_(1000)
{
  // Publish jogger status
  status_pub_ = nh_.advertise<std_msgs::Int8>(parameters_.status_topic, 1);

  // MoveIt Setup
  while (ros::ok() && !model_loader_ptr)
  {
    ROS_WARN_THROTTLE_NAMED(5, LOGNAME, "Waiting for a non-null robot_model_loader pointer");
    default_sleep_rate_.sleep();
  }
  const moveit::core::RobotModelPtr& kinematic_model = model_loader_ptr->getModel();
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(parameters_.move_group_name);
  prev_joint_velocity_ = Eigen::ArrayXd::Zero(joint_model_group_->getActiveJointModels().size());
}

void JogCalcs::startMainLoop(JogArmShared& shared_variables)
{
  // Reset flags
  is_initialized_ = false;

  // Wait for initial messages
  ROS_INFO_NAMED(LOGNAME, "jog_calcs_thread: Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);
  ROS_INFO_NAMED(LOGNAME, "jog_calcs_thread: Received first joint msg.");

  internal_joint_state_.name = joint_model_group_->getActiveJointModelNames();
  num_joints_ = internal_joint_state_.name.size();
  internal_joint_state_.position.resize(num_joints_);
  internal_joint_state_.velocity.resize(num_joints_);
  // A map for the indices of incoming joint commands
  for (std::size_t i = 0; i < internal_joint_state_.name.size(); ++i)
  {
    joint_state_name_map_[internal_joint_state_.name[i]] = i;
  }

  // Low-pass filters for the joint positions & velocities
  for (size_t i = 0; i < num_joints_; ++i)
  {
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // Initialize the position filters to initial robot joints
  while (!updateJoints(shared_variables) && ros::ok())
  {
    if (shared_variables.stop_requested)
      return;

    shared_variables.lock();
    incoming_joint_state_ = shared_variables.joints;
    shared_variables.unlock();
    default_sleep_rate_.sleep();
  }

  is_initialized_ = true;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count = 0;

  ros::Rate loop_rate(1. / parameters_.publish_period);

  // Flag for staying inactive while there are no incoming commands
  bool wait_for_jog_commands = true;

  // Incoming command messages
  geometry_msgs::TwistStamped cartesian_deltas;
  control_msgs::JointJog joint_deltas;

  // Do jogging calcs
  while (ros::ok() && !shared_variables.stop_requested)
  {
    // Always update the joints and end-effector transform for 2 reasons:
    // 1) in case the getCommandFrameTransform() method is being used
    // 2) so the low-pass filters are up to date and don't cause a jump
    while (!updateJoints(shared_variables) && ros::ok())
    {
      default_sleep_rate_.sleep();
    }

    // Get the transform from MoveIt planning frame to jogging command frame
    // We solve (planning_frame -> base -> robot_link_command_frame)
    // by computing (base->planning_frame)^-1 * (base->robot_link_command_frame)
    kinematic_state_->setVariableValues(incoming_joint_state_);
    tf_moveit_to_cmd_frame_ = kinematic_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                              kinematic_state_->getGlobalLinkTransform(parameters_.robot_link_command_frame);
    shared_variables.lock();
    shared_variables.tf_moveit_to_cmd_frame = tf_moveit_to_cmd_frame_;
    shared_variables.unlock();

    // If paused or while waiting for initial jog commands, just keep the low-pass filters up to date with current
    // joints so a jump doesn't occur when restarting
    if (wait_for_jog_commands || shared_variables.paused)
    {
      for (std::size_t i = 0; i < num_joints_; ++i)
        position_filters_[i].reset(original_joint_state_.position[i]);

      shared_variables.lock();
      // Check if there are any new commands with valid timestamp
      wait_for_jog_commands = shared_variables.command_deltas.header.stamp == ros::Time(0.) &&
                              shared_variables.joint_command_deltas.header.stamp == ros::Time(0.);
      shared_variables.unlock();
    }
    // If not waiting for initial command, and not paused.
    // Do jogging calculations only if the robot should move, for efficiency
    else
    {
      // Halt if the command is stale or inputs are all zero, or commands were zero
      shared_variables.lock();
      bool have_nonzero_cartesian_cmd = shared_variables.have_nonzero_cartesian_cmd;
      bool have_nonzero_joint_cmd = shared_variables.have_nonzero_joint_cmd;
      bool stale_command = shared_variables.command_is_stale;
      shared_variables.unlock();

      bool valid_nonzero_command = false;
      if (!stale_command)
      {
        // Prioritize cartesian jogging above joint jogging
        if (have_nonzero_cartesian_cmd)
        {
          shared_variables.lock();
          cartesian_deltas = shared_variables.command_deltas;
          shared_variables.unlock();

          if (!cartesianJogCalcs(cartesian_deltas, shared_variables))
            continue;
        }
        else if (have_nonzero_joint_cmd)
        {
          shared_variables.lock();
          joint_deltas = shared_variables.joint_command_deltas;
          shared_variables.unlock();

          if (!jointJogCalcs(joint_deltas, shared_variables))
            continue;
        }

        valid_nonzero_command = have_nonzero_cartesian_cmd || have_nonzero_joint_cmd;
      }

      // If we should halt
      if (!valid_nonzero_command)
      {
        // Keep the joint position filters up-to-date with current joints
        for (std::size_t i = 0; i < num_joints_; ++i)
          position_filters_[i].reset(original_joint_state_.position[i]);

        suddenHalt(outgoing_command_);
        have_nonzero_cartesian_cmd = false;
        have_nonzero_joint_cmd = false;
        // Reset the valid command flag so jogging stops until a new command arrives
        shared_variables.lock();
        shared_variables.have_nonzero_cartesian_cmd = false;
        shared_variables.have_nonzero_joint_cmd = false;
        shared_variables.unlock();
      }

      // Send the newest target joints
      shared_variables.lock();
      // If everything normal, share the new traj to be published
      if (valid_nonzero_command)
      {
        shared_variables.outgoing_command = outgoing_command_;
        shared_variables.ok_to_publish = true;
      }
      // Skip the jogging publication if all inputs have been zero for several cycles in a row.
      // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep republishing forever.
      else if ((parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
               (zero_velocity_count > parameters_.num_outgoing_halt_msgs_to_publish))
      {
        shared_variables.ok_to_publish = false;
      }
      // The command is invalid but we are publishing num_outgoing_halt_msgs_to_publish
      else
      {
        shared_variables.outgoing_command = outgoing_command_;
        shared_variables.ok_to_publish = true;
      }
      shared_variables.unlock();

      // Store last zero-velocity message flag to prevent superfluous warnings.
      // Cartesian and joint commands must both be zero.
      if (!have_nonzero_cartesian_cmd && !have_nonzero_joint_cmd)
      {
        // Avoid overflow
        if (zero_velocity_count < std::numeric_limits<int>::max())
          ++zero_velocity_count;
      }
      else
        zero_velocity_count = 0;
    }

    loop_rate.sleep();
  }
}

bool JogCalcs::isInitialized()
{
  return is_initialized_;
}

// Perform the jogging calculations
bool JogCalcs::cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables)
{
  // Check for nan's in the incoming command
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_.command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  // Set uncontrolled dimensions to 0 in command frame
  if (!shared_variables.control_dimensions[0])
    cmd.twist.linear.x = 0;
  if (!shared_variables.control_dimensions[1])
    cmd.twist.linear.y = 0;
  if (!shared_variables.control_dimensions[2])
    cmd.twist.linear.z = 0;
  if (!shared_variables.control_dimensions[3])
    cmd.twist.angular.x = 0;
  if (!shared_variables.control_dimensions[4])
    cmd.twist.angular.y = 0;
  if (!shared_variables.control_dimensions[5])
    cmd.twist.angular.z = 0;

  // Transform the command to the MoveGroup planning frame
  if (cmd.header.frame_id != parameters_.planning_frame)
  {
    Eigen::Vector3d translation_vector(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    Eigen::Vector3d angular_vector(cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z);

    // We solve (planning_frame -> base -> cmd.header.frame_id)
    // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
    const auto tf_planning_to_cmd_frame =
        kinematic_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
        kinematic_state_->getGlobalLinkTransform(cmd.header.frame_id);

    translation_vector = tf_planning_to_cmd_frame.linear() * translation_vector;
    angular_vector = tf_planning_to_cmd_frame.linear() * angular_vector;

    // Put these components back into a TwistStamped
    cmd.header.frame_id = parameters_.planning_frame;
    cmd.twist.linear.x = translation_vector(0);
    cmd.twist.linear.y = translation_vector(1);
    cmd.twist.linear.z = translation_vector(2);
    cmd.twist.angular.x = angular_vector(0);
    cmd.twist.angular.y = angular_vector(1);
    cmd.twist.angular.z = angular_vector(2);
  }

  Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

  // May allow some dimensions to drift, based on shared_variables.drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector shared_variables.drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
  for (auto dimension = jacobian.rows(); dimension >= 0; --dimension)
  {
    if (shared_variables.drift_dimensions[dimension] && jacobian.rows() > 1)
    {
      removeDimension(jacobian, delta_x, dimension);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  delta_theta_ = pseudo_inverse * delta_x;

  enforceSRDFAccelVelLimits(delta_theta_);

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(shared_variables, delta_theta_,
                       velocityScalingFactorForSingularity(delta_x, svd, jacobian, pseudo_inverse));
  if (status_ == HALT_FOR_COLLISION)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(5, LOGNAME, "Halting for collision!");
    suddenHalt(delta_theta_);
  }

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;

  publishStatus();
  // Cache the status so it can be retrieved asynchronously
  updateCachedStatus(shared_variables);

  return convertDeltasToOutgoingCmd();
}

bool JogCalcs::jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables)
{
  // Check for nan's or |delta|>1 in the incoming command
  for (double velocity : cmd.velocities)
  {
    if (std::isnan(velocity))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }

  // Apply user-defined scaling
  delta_theta_ = scaleJointCommand(cmd);

  enforceSRDFAccelVelLimits(delta_theta_);

  kinematic_state_->setVariableValues(internal_joint_state_);

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;

  publishStatus();
  // Cache the status so it can be retrieved asynchronously
  updateCachedStatus(shared_variables);

  return convertDeltasToOutgoingCmd();
}

void JogCalcs::updateCachedStatus(JogArmShared& shared_variables)
{
  shared_variables.status = status_;
  status_ = NO_WARNING;
}

bool JogCalcs::convertDeltasToOutgoingCmd()
{
  internal_joint_state_ = original_joint_state_;
  if (!addJointIncrements(internal_joint_state_, delta_theta_))
    return false;

  lowPassFilterPositions(internal_joint_state_);

  // Calculate joint velocities here so that positions are filtered and SRDF bounds still get checked
  calculateJointVelocities(internal_joint_state_, delta_theta_);

  outgoing_command_ = composeJointTrajMessage(internal_joint_state_);

  if (!enforceSRDFPositionLimits(outgoing_command_))
  {
    suddenHalt(outgoing_command_);
    status_ = JOINT_BOUND;
  }

  // done with calculations
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(outgoing_command_, gazebo_redundant_message_count_);
  }

  return true;
}

// Spam several redundant points into the trajectory. The first few may be skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo simulation.
// Start from 2 because the first point's timestamp is already 1*parameters_.publish_period
void JogCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const
{
  auto point = trajectory.points[0];
  // Start from 2 because we already have the first point. End at count+1 so (total #) == count
  for (int i = 2; i < count + 1; ++i)
  {
    point.time_from_start = ros::Duration(i * parameters_.publish_period);
    trajectory.points.push_back(point);
  }
}

void JogCalcs::lowPassFilterPositions(sensor_msgs::JointState& joint_state)
{
  for (size_t i = 0; i < position_filters_.size(); ++i)
  {
    joint_state.position[i] = position_filters_[i].filter(joint_state.position[i]);
  }
}

void JogCalcs::calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta)
{
  for (int i = 0; i < delta_theta.size(); ++i)
  {
    joint_state.velocity[i] = delta_theta[i] / parameters_.publish_period;
  }
}

trajectory_msgs::JointTrajectory JogCalcs::composeJointTrajMessage(sensor_msgs::JointState& joint_state) const
{
  trajectory_msgs::JointTrajectory new_joint_traj;
  new_joint_traj.header.frame_id = parameters_.planning_frame;
  new_joint_traj.header.stamp = ros::Time::now();
  new_joint_traj.joint_names = joint_state.name;

  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);
  if (parameters_.publish_joint_positions)
    point.positions = joint_state.position;
  if (parameters_.publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (parameters_.publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(num_joints_);
    point.accelerations = acceleration;
  }
  new_joint_traj.points.push_back(point);

  return new_joint_traj;
}

// Apply velocity scaling for proximity of collisions and singularities.
// Scale for collisions is read from a shared variable.
void JogCalcs::applyVelocityScaling(JogArmShared& shared_variables, Eigen::ArrayXd& delta_theta,
                                    double singularity_scale)
{
  shared_variables.lock();
  double collision_scale = shared_variables.collision_velocity_scale;
  shared_variables.unlock();

  if (collision_scale > 0 && collision_scale < 1)
  {
    status_ = DECELERATE_FOR_COLLISION;
    ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, JOG_ARM_STATUS_CODE_MAP.at(status_));
  }
  else if (collision_scale == 0)
  {
    status_ = HALT_FOR_COLLISION;
  }

  delta_theta = collision_scale * singularity_scale * delta_theta;
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double JogCalcs::velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                                     const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                     const Eigen::MatrixXd& jacobian,
                                                     const Eigen::MatrixXd& pseudo_inverse)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = jacobian.rows();

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
  kinematic_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse * delta_x;
  kinematic_state_->setJointGroupPositions(joint_model_group_, new_theta);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(jacobian);
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
    if ((ini_condition > parameters_.lower_singularity_threshold) &&
        (ini_condition < parameters_.hard_stop_singularity_threshold))
    {
      velocity_scale = 1. -
                       (ini_condition - parameters_.lower_singularity_threshold) /
                           (parameters_.hard_stop_singularity_threshold - parameters_.lower_singularity_threshold);
      status_ = DECELERATE_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, JOG_ARM_STATUS_CODE_MAP.at(status_));
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_.hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      status_ = HALT_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, JOG_ARM_STATUS_CODE_MAP.at(status_));
    }
  }

  return velocity_scale;
}

void JogCalcs::enforceSRDFAccelVelLimits(Eigen::ArrayXd& delta_theta)
{
  Eigen::ArrayXd velocity = delta_theta / parameters_.publish_period;
  const Eigen::ArrayXd acceleration = (velocity - prev_joint_velocity_) / parameters_.publish_period;

  std::size_t joint_delta_index = 0;
  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    // Some joints do not have bounds defined
    const auto bounds = joint->getVariableBounds(joint->getName());
    if (bounds.acceleration_bounded_)
    {
      bool clip_acceleration = false;
      double acceleration_limit = 0.0;
      if (acceleration(joint_delta_index) < bounds.min_acceleration_)
      {
        clip_acceleration = true;
        acceleration_limit = bounds.min_acceleration_;
      }
      else if (acceleration(joint_delta_index) > bounds.max_acceleration_)
      {
        clip_acceleration = true;
        acceleration_limit = bounds.max_acceleration_;
      }

      // Apply acceleration bounds
      if (clip_acceleration)
      {
        // accel = (vel - vel_prev) / delta_t = ((delta_theta / delta_t) - vel_prev) / delta_t
        // --> delta_theta = (accel * delta_t _ + vel_prev) * delta_t
        const double relative_change =
            ((acceleration_limit * parameters_.publish_period + prev_joint_velocity_(joint_delta_index)) *
             parameters_.publish_period) /
            delta_theta(joint_delta_index);
        // Avoid nan
        if (fabs(relative_change) < 1)
          delta_theta(joint_delta_index) = relative_change * delta_theta(joint_delta_index);
      }
    }

    if (bounds.velocity_bounded_)
    {
      velocity(joint_delta_index) = delta_theta(joint_delta_index) / parameters_.publish_period;

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
        // delta_theta = joint_velocity * delta_t
        const double relative_change = (velocity_limit * parameters_.publish_period) / delta_theta(joint_delta_index);
        // Avoid nan
        if (fabs(relative_change) < 1)
        {
          delta_theta(joint_delta_index) = relative_change * delta_theta(joint_delta_index);
          velocity(joint_delta_index) = relative_change * velocity(joint_delta_index);
        }
      }
      ++joint_delta_index;
    }
  }
}

bool JogCalcs::enforceSRDFPositionLimits(trajectory_msgs::JointTrajectory& new_joint_traj)
{
  bool halting = false;

  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    // Halt if we're past a joint margin and joint velocity is moving even farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < original_joint_state_.name.size(); ++c)
    {
      if (original_joint_state_.name[c] == joint->getName())
      {
        joint_angle = original_joint_state_.position.at(c);
        break;
      }
    }
    if (!kinematic_state_->satisfiesPositionBounds(joint, -parameters_.joint_limit_margin))
    {
      const std::vector<moveit_msgs::JointLimits> limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (!limits.empty())
      {
        if ((kinematic_state_->getJointVelocities(joint)[0] < 0 &&
             (joint_angle < (limits[0].min_position + parameters_.joint_limit_margin))) ||
            (kinematic_state_->getJointVelocities(joint)[0] > 0 &&
             (joint_angle > (limits[0].max_position - parameters_.joint_limit_margin))))
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, ros::this_node::getName() << " " << joint->getName()
                                                                               << " close to a "
                                                                                  " position limit. Halting.");
          halting = true;
        }
      }
    }
  }
  return !halting;
}

void JogCalcs::publishStatus() const
{
  std_msgs::Int8 status_msg;
  status_msg.data = status_;
  status_pub_.publish(status_msg);
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
void JogCalcs::suddenHalt(Eigen::ArrayXd& delta_theta)
{
  delta_theta = Eigen::ArrayXd::Zero(delta_theta.rows());
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
void JogCalcs::suddenHalt(trajectory_msgs::JointTrajectory& joint_traj)
{
  if (joint_traj.points.empty())
  {
    joint_traj.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    joint_traj.points[0].positions.resize(num_joints_);
    joint_traj.points[0].velocities.resize(num_joints_);
  }

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good state
    if (parameters_.publish_joint_positions)
      joint_traj.points[0].positions[i] = original_joint_state_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_.publish_joint_velocities)
      joint_traj.points[0].velocities[i] = 0;
  }
}

// Parse the incoming joint msg for the joints of our MoveGroup
bool JogCalcs::updateJoints(JogArmShared& shared_variables)
{
  shared_variables.lock();
  incoming_joint_state_ = shared_variables.joints;
  shared_variables.unlock();

  // Check that the msg contains enough joints
  if (incoming_joint_state_.name.size() < num_joints_)
    return false;

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_joint_state_.name.size(); ++m)
  {
    std::size_t c;
    try
    {
      c = joint_state_name_map_.at(incoming_joint_state_.name[m]);
    }
    catch (const std::out_of_range& e)
    {
      ROS_DEBUG_STREAM_THROTTLE_NAMED(5, LOGNAME, "Ignoring joint " << incoming_joint_state_.name[m]);
      continue;
    }

    internal_joint_state_.position[c] = incoming_joint_state_.position[m];
  }

  // Cache the original joints in case they need to be reset
  original_joint_state_ = internal_joint_state_;

  return true;
}

// Scale the incoming jog command
Eigen::VectorXd JogCalcs::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_.command_in_type == "unitless")
  {
    result[0] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.x;
    result[1] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.y;
    result[2] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.z;
    result[3] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.x;
    result[4] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.y;
    result[5] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.z;
  }
  // Otherwise, commands are in m/s and rad/s
  else if (parameters_.command_in_type == "speed_units")
  {
    result[0] = command.twist.linear.x * parameters_.publish_period;
    result[1] = command.twist.linear.y * parameters_.publish_period;
    result[2] = command.twist.linear.z * parameters_.publish_period;
    result[3] = command.twist.angular.x * parameters_.publish_period;
    result[4] = command.twist.angular.y * parameters_.publish_period;
    result[5] = command.twist.angular.z * parameters_.publish_period;
  }
  else
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unexpected command_in_type");

  return result;
}

Eigen::VectorXd JogCalcs::scaleJointCommand(const control_msgs::JointJog& command) const
{
  Eigen::VectorXd result(num_joints_);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    result[i] = 0.0;
  }

  std::size_t c;
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    try
    {
      c = joint_state_name_map_.at(command.joint_names[m]);
    }
    catch (const std::out_of_range& e)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(5, LOGNAME, "Ignoring joint " << incoming_joint_state_.name[m]);
      continue;
    }
    // Apply user-defined scaling if inputs are unitless [-1:1]
    if (parameters_.command_in_type == "unitless")
      result[c] = command.velocities[m] * parameters_.joint_scale * parameters_.publish_period;
    // Otherwise, commands are in m/s and rad/s
    else if (parameters_.command_in_type == "speed_units")
      result[c] = command.velocities[m] * parameters_.publish_period;
    else
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Unexpected command_in_type, check yaml file.");
  }

  return result;
}

// Add the deltas to each joint
bool JogCalcs::addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const
{
  for (std::size_t i = 0, size = static_cast<std::size_t>(increments.size()); i < size; ++i)
  {
    try
    {
      output.position[i] += increments[static_cast<long>(i)];
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, ros::this_node::getName() << " Lengths of output and "
                                                                   "increments do not match.");
      return false;
    }
  }

  return true;
}

void JogCalcs::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x, unsigned int row_to_remove)
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
}  // namespace moveit_jog_arm
