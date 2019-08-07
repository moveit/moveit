///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_calcs.h
//      Project   : jog_arm
//      Created   : 1/11/2019
//      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
//
// BSD 3-Clause License
//
// Copyright (c) 2019, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include <jog_arm/jog_calcs.h>

namespace jog_arm
{
// Constructor for the class that handles jogging calculations
JogCalcs::JogCalcs(const JogArmParameters& parameters, JogArmShared& shared_variables, pthread_mutex_t& mutex,
                   const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr)
  : tf_listener_(tf_buffer_), parameters_(parameters)
{
  // Publish collision status
  warning_pub_ = nh_.advertise<std_msgs::Bool>(parameters_.warning_topic, 1);

  // MoveIt Setup
  while (ros::ok() && !model_loader_ptr)
  {
    ROS_WARN_THROTTLE_NAMED(5, LOGNAME, "Waiting for a non-null robot_model_loader pointer");
    ros::Duration(WHILE_LOOP_WAIT).sleep();
  }
  const robot_model::RobotModelPtr& kinematic_model = model_loader_ptr->getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(parameters_.move_group_name);

  // Wait for initial messages
  ROS_INFO_NAMED(LOGNAME, "Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);
  ROS_INFO_NAMED(LOGNAME, "Received first joint msg.");

  resetVelocityFilters();

  jt_state_.name = joint_model_group_->getVariableNames();
  num_joints_ = jt_state_.name.size();
  jt_state_.position.resize(num_joints_);
  jt_state_.velocity.resize(num_joints_);
  jt_state_.effort.resize(num_joints_);
  // A map for the indices of incoming joint commands
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_name_map_[jt_state_.name[i]] = i;
  }

  // Low-pass filters for the joint positions & velocities
  for (size_t i = 0; i < num_joints_; ++i)
  {
    velocity_filters_.emplace_back(parameters_.low_pass_filter_coeff);
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // Initialize the position filters to initial robot joints
  while (!updateJoints() && ros::ok())
  {
    pthread_mutex_lock(&mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&mutex);
    ros::Duration(WHILE_LOOP_WAIT).sleep();
  }
  for (std::size_t i = 0; i < num_joints_; ++i)
    position_filters_[i].reset(jt_state_.position[i]);

  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs, then free up the shared variable again.
  geometry_msgs::TwistStamped cartesian_deltas;
  control_msgs::JointJog joint_deltas;
  while (ros::ok() && (cartesian_deltas.header.stamp == ros::Time(0.)) && (joint_deltas.header.stamp == ros::Time(0.)))
  {
    ros::Duration(WHILE_LOOP_WAIT).sleep();

    pthread_mutex_lock(&mutex);
    cartesian_deltas = shared_variables.command_deltas;
    joint_deltas = shared_variables.joint_command_deltas;
    pthread_mutex_unlock(&mutex);
  }

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count = 0;

  ros::Rate loop_rate(1. / parameters_.publish_period);

  // Now do jogging calcs
  while (ros::ok())
  {
    // Flag that incoming commands are all zero. May be used to skip calculations/publication
    pthread_mutex_lock(&mutex);
    bool zero_cartesian_cmd_flag = shared_variables.zero_cartesian_cmd_flag;
    bool zero_joint_cmd_flag = shared_variables.zero_joint_cmd_flag;
    pthread_mutex_unlock(&mutex);

    if (zero_cartesian_cmd_flag && zero_joint_cmd_flag)
      // Reset low-pass filters
      resetVelocityFilters();

    // Pull data from the shared variables.
    pthread_mutex_lock(&mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&mutex);

    // Initialize the position filters to initial robot joints
    while (!updateJoints() && ros::ok())
    {
      pthread_mutex_lock(&mutex);
      incoming_jts_ = shared_variables.joints;
      pthread_mutex_unlock(&mutex);
      ros::Duration(WHILE_LOOP_WAIT).sleep();
    }

    // Prioritize cartesian jogging above joint jogging
    if (!zero_cartesian_cmd_flag)
    {
      pthread_mutex_lock(&mutex);
      cartesian_deltas = shared_variables.command_deltas;
      pthread_mutex_unlock(&mutex);

      if (!cartesianJogCalcs(cartesian_deltas, shared_variables, mutex))
        continue;
    }
    else if (!zero_joint_cmd_flag)
    {
      pthread_mutex_lock(&mutex);
      joint_deltas = shared_variables.joint_command_deltas;
      pthread_mutex_unlock(&mutex);

      if (!jointJogCalcs(joint_deltas, shared_variables))
        continue;
    }
    else
    {
      original_jt_state_ = jt_state_;
      outgoing_command_ = composeOutgoingMessage(jt_state_);
    }

    // Halt if the command is stale or inputs are all zero, or commands were zero
    pthread_mutex_lock(&mutex);
    bool stale_command = shared_variables.command_is_stale;
    pthread_mutex_unlock(&mutex);

    if (stale_command || (zero_cartesian_cmd_flag && zero_joint_cmd_flag))
    {
      suddenHalt(outgoing_command_);
      zero_cartesian_cmd_flag = true;
      zero_joint_cmd_flag = true;
    }

    bool valid_nonzero_command = !zero_cartesian_cmd_flag || !zero_joint_cmd_flag;

    // Send the newest target joints
    if (!outgoing_command_.joint_names.empty())
    {
      pthread_mutex_lock(&mutex);
      // If everything normal, share the new traj to be published
      if (valid_nonzero_command)
      {
        shared_variables.outgoing_command = outgoing_command_;
        shared_variables.ok_to_publish = true;
      }
      // Skip the jogging publication if all inputs have been zero for several cycles in a row.
      // num_halt_msgs_to_publish == 0 signifies that we should keep republishing forever.
      else if ((parameters_.num_halt_msgs_to_publish != 0) &&
               (zero_velocity_count > parameters_.num_halt_msgs_to_publish))
      {
        shared_variables.ok_to_publish = false;
        // Reset the velocity filters so robot doesn't jump when commands resume
        resetVelocityFilters();
      }
      // The command is invalid but we are publishing num_halt_msgs_to_publish
      else
      {
        shared_variables.outgoing_command = outgoing_command_;
        shared_variables.ok_to_publish = true;
      }
      pthread_mutex_unlock(&mutex);

      // Store last zero-velocity message flag to prevent superfluous warnings.
      // Cartesian and joint commands must both be zero.
      if (zero_cartesian_cmd_flag && zero_joint_cmd_flag)
      {
        // Avoid overflow
        if (zero_velocity_count < INT_MAX)
          ++zero_velocity_count;
      }
      else
        zero_velocity_count = 0;
    }

    loop_rate.sleep();
  }
}

// Perform the jogging calculations
bool JogCalcs::cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables,
                                 pthread_mutex_t& mutex)
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

  // Transform the command to the MoveGroup planning frame.
  geometry_msgs::TransformStamped command_frame_to_planning_frame;
  try
  {
    command_frame_to_planning_frame =
        tf_buffer_.lookupTransform(parameters_.planning_frame, cmd.header.frame_id, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, ros::this_node::getName() << ": " << ex.what());
    return false;
  }

  geometry_msgs::Vector3 lin_vector = cmd.twist.linear;
  try
  {
    tf2::doTransform(lin_vector, lin_vector, command_frame_to_planning_frame);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, ros::this_node::getName() << ": " << ex.what());
    return false;
  }

  geometry_msgs::Vector3 rot_vector = cmd.twist.angular;
  try
  {
    tf2::doTransform(rot_vector, rot_vector, command_frame_to_planning_frame);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, ros::this_node::getName() << ": " << ex.what());
    return false;
  }

  // Put these components back into a TwistStamped
  cmd.header.frame_id = parameters_.planning_frame;
  cmd.twist.linear = lin_vector;
  cmd.twist.angular = rot_vector;

  const Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);

  kinematic_state_->setVariableValues(jt_state_);
  original_jt_state_ = jt_state_;

  // Convert from cartesian commands to joint commands
  jacobian_ = kinematic_state_->getJacobian(joint_model_group_);
  svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  matrix_s_ = svd_.singularValues().asDiagonal();
  pseudo_inverse_ = svd_.matrixV() * matrix_s_.inverse() * svd_.matrixU().transpose();
  delta_theta_ = pseudo_inverse_ * delta_x;

  enforceJointVelocityLimits(delta_theta_);

  if (!addJointIncrements(jt_state_, delta_theta_))
    return false;

  // Include a velocity estimate for velocity-controlled robots
  Eigen::VectorXd joint_vel(delta_theta_ / parameters_.publish_period);

  lowPassFilterVelocities(joint_vel);
  lowPassFilterPositions();

  outgoing_command_ = composeOutgoingMessage(jt_state_);

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(shared_variables, mutex, outgoing_command_, delta_theta_,
                       decelerateForSingularity(delta_x, svd_));

  if (!checkIfJointsWithinURDFBounds(outgoing_command_))
  {
    suddenHalt(outgoing_command_);
    publishWarning(true);
  }
  else
    publishWarning(false);

  // If using Gazebo simulator, insert redundant points
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(outgoing_command_, gazebo_redundant_message_count_);
  }

  return true;
}

bool JogCalcs::jointJogCalcs(const control_msgs::JointJog& cmd, JogArmShared& shared_variables)
{
  // Check for nan's or |delta|>1 in the incoming command
  for (double velocity : cmd.velocities)
  {
    if (std::isnan(velocity) || (fabs(velocity) > 1))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }

  // Apply user-defined scaling
  const Eigen::VectorXd delta = scaleJointCommand(cmd);

  kinematic_state_->setVariableValues(jt_state_);
  original_jt_state_ = jt_state_;

  if (!addJointIncrements(jt_state_, delta))
    return false;

  // Include a velocity estimate for velocity-controlled robots
  Eigen::VectorXd joint_vel(delta / parameters_.publish_period);

  lowPassFilterVelocities(joint_vel);
  lowPassFilterPositions();

  // update joint state with new values
  kinematic_state_->setVariableValues(jt_state_);

  outgoing_command_ = composeOutgoingMessage(jt_state_);

  // check if new joint state is valid
  if (!checkIfJointsWithinURDFBounds(outgoing_command_))
  {
    suddenHalt(outgoing_command_);
    publishWarning(true);
  }
  else
  {
    publishWarning(false);
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

void JogCalcs::lowPassFilterPositions()
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    jt_state_.position[i] = position_filters_[i].filter(jt_state_.position[i]);

    // Check for nan's
    if (std::isnan(jt_state_.position[i]))
    {
      jt_state_.position[i] = original_jt_state_.position[i];
      jt_state_.velocity[i] = 0.;
    }
  }
}

void JogCalcs::lowPassFilterVelocities(const Eigen::VectorXd& joint_vel)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    jt_state_.velocity[i] = velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

    // Check for nan's
    if (std::isnan(jt_state_.velocity[static_cast<long>(i)]))
    {
      jt_state_.position[i] = original_jt_state_.position[i];
      jt_state_.velocity[i] = 0.;
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "nan in velocity filter");
    }
  }
}

trajectory_msgs::JointTrajectory JogCalcs::composeOutgoingMessage(sensor_msgs::JointState& joint_state) const
{
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.frame_id = parameters_.planning_frame;
  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names = joint_state.name;

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
  new_jt_traj.points.push_back(point);

  return new_jt_traj;
}

// Apply velocity scaling for proximity of collisions and singularities.
// Scale for collisions is read from a shared variable.
// Key equation: new_velocity = collision_scale*singularity_scale*previous_velocity
bool JogCalcs::applyVelocityScaling(JogArmShared& shared_variables, pthread_mutex_t& mutex,
                                    trajectory_msgs::JointTrajectory& new_jt_traj, const Eigen::VectorXd& delta_theta,
                                    double singularity_scale)
{
  pthread_mutex_unlock(&mutex);
  double collision_scale = shared_variables.collision_velocity_scale;
  pthread_mutex_unlock(&mutex);

  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (parameters_.publish_joint_positions)
    {
      // If close to a singularity or joint limit, undo any change to the joint angles
      new_jt_traj.points[0].positions[i] =
          new_jt_traj.points[0].positions[i] -
          (1. - singularity_scale * collision_scale) * delta_theta[static_cast<long>(i)];
    }
    if (parameters_.publish_joint_velocities)
      new_jt_traj.points[0].velocities[i] *= singularity_scale * collision_scale;
  }

  return true;
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double JogCalcs::decelerateForSingularity(const Eigen::VectorXd& commanded_velocity,
                                          const Eigen::JacobiSVD<Eigen::MatrixXd>& svd)
{
  double velocity_scale = 1;

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(5);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(6);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse_ * delta_x;
  kinematic_state_->setJointGroupPositions(joint_model_group_, new_theta);

  jacobian_ = kinematic_state_->getJacobian(joint_model_group_);
  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian_);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity[0] *= -1;
    vector_toward_singularity[1] *= -1;
    vector_toward_singularity[2] *= -1;
    vector_toward_singularity[3] *= -1;
    vector_toward_singularity[4] *= -1;
    vector_toward_singularity[5] *= -1;
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
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_.hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      ROS_WARN_THROTTLE_NAMED(2, LOGNAME, "Close to a singularity. Halting.");
    }
  }

  return velocity_scale;
}

void JogCalcs::enforceJointVelocityLimits(Eigen::VectorXd& calculated_joint_change)
{
  double maximum_joint_change = calculated_joint_change.cwiseAbs().maxCoeff();
  if (maximum_joint_change > parameters_.joint_scale * parameters_.publish_period)
  {
    // Scale the entire joint velocity vector so that each joint velocity is below min, and the output movement is
    // scaled uniformly to match expected motion
    calculated_joint_change =
        calculated_joint_change * parameters_.joint_scale * parameters_.publish_period / maximum_joint_change;
  }
}

bool JogCalcs::checkIfJointsWithinURDFBounds(trajectory_msgs::JointTrajectory& new_jt_traj)
{
  bool halting = false;
  for (auto joint : joint_model_group_->getJointModels())
  {
    if (!kinematic_state_->satisfiesVelocityBounds(joint))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, ros::this_node::getName() << " " << joint->getName() << " "
                                                                           << " close to a "
                                                                              " velocity limit. Enforcing limit.");
      kinematic_state_->enforceVelocityBounds(joint);
      for (std::size_t c = 0; c < num_joints_; ++c)
      {
        if (new_jt_traj.joint_names[c] == joint->getName())
        {
          new_jt_traj.points[0].velocities[c] = kinematic_state_->getJointVelocities(joint)[0];
          break;
        }
      }
    }

    // Halt if we're past a joint margin and joint velocity is moving even farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < num_joints_; ++c)
    {
      if (original_jt_state_.name[c] == joint->getName())
      {
        joint_angle = original_jt_state_.position.at(c);
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

void JogCalcs::publishWarning(bool active) const
{
  std_msgs::Bool status;
  status.data = static_cast<std_msgs::Bool::_data_type>(active);
  warning_pub_.publish(status);
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
void JogCalcs::suddenHalt(trajectory_msgs::JointTrajectory& jt_traj)
{
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good state
    if (parameters_.publish_joint_positions)
      jt_traj.points[0].positions[i] = original_jt_state_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_.publish_joint_velocities)
      jt_traj.points[0].velocities[i] = 0;
  }
}

// Reset the data stored in filters so the trajectory won't jump when jogging is resumed.
void JogCalcs::resetVelocityFilters()
{
  for (std::size_t i = 0; i < num_joints_; ++i)
    velocity_filters_[i].reset(0);  // Zero velocity
}

// Parse the incoming joint msg for the joints of our MoveGroup
bool JogCalcs::updateJoints()
{
  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < num_joints_)
    return false;

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_jts_.name.size(); ++m)
  {
    std::size_t c;
    try
    {
      c = jt_state_name_map_.at(incoming_jts_.name[m]);
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(5, LOGNAME, "Command joint name unknown.");
      continue;
    }

    jt_state_.position[c] = incoming_jts_.position[m];
  }

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
      c = jt_state_name_map_.at(command.joint_names[m]);
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(5, LOGNAME, "Command joint name unknown.");
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
}  // namespace jog_arm
