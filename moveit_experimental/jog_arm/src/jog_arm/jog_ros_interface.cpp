///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_ros_interface.cpp
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
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

// Server node for arm jogging with MoveIt.

#include <jog_arm/jog_ros_interface.h>

namespace jog_arm
{

// Initialize these static struct to hold ROS parameters.
// They must be static because they are used as arguments in thread creation.
JogArmShared JogROSInterface::shared_variables_;

/////////////////////////////////////////////////////////////////////////////////
// JogROSInterface handles ROS subscriptions and instantiates the worker
// threads.
// One worker thread does the jogging calculations.
// Another worker thread does collision checking.
/////////////////////////////////////////////////////////////////////////////////

static const double WHILE_LOOP_WAIT = 0.001;

// Constructor for the main ROS interface node
JogROSInterface::JogROSInterface()
{
  ros::NodeHandle nh;

  // Read ROS parameters, typically from YAML file
  if (!readParameters(nh))
    exit(EXIT_FAILURE);

  // Load the robot model. This is used by the worker threads.
  model_loader_ptr_ = std::shared_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);

  // Crunch the numbers in this thread
  std::thread jogging_thread (JogROSInterface::startJogCalcThread,
    ros_parameters_, std::ref(shared_variables_), model_loader_ptr_);

  // Check collisions in this thread
  std::thread collision_thread (JogROSInterface::startCollisionCheckThread, 
    ros_parameters_, std::ref(shared_variables_), std::ref(model_loader_ptr_));  

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub =
      nh.subscribe(ros_parameters_.cartesian_command_in_topic, 1, &JogROSInterface::deltaCartesianCmdCB, this);
  ros::Subscriber joints_sub = nh.subscribe(ros_parameters_.joint_topic, 1, &JogROSInterface::jointsCB, this);
  ros::Subscriber joint_jog_cmd_sub =
      nh.subscribe(ros_parameters_.joint_command_in_topic, 1, &JogROSInterface::deltaJointCmdCB, this);
  ros::topic::waitForMessage<sensor_msgs::JointState>(ros_parameters_.joint_topic);
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(ros_parameters_.cartesian_command_in_topic);

  // Publish freshly-calculated joints to the robot
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory
  // or std_msgs/Float64MultiArray).
  ros::Publisher outgoing_cmd_pub;
  if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>(ros_parameters_.command_out_topic, 1);
  else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>(ros_parameters_.command_out_topic, 1);

  // Wait for low pass filters to stabilize
  ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for low-pass filters to stabilize.");
  ros::Duration(10 * ros_parameters_.publish_period).sleep();

  ros::Rate main_rate(1. / ros_parameters_.publish_period);

  while (ros::ok())
  {
    ros::spinOnce();

    pthread_mutex_lock(&shared_variables_.shared_variables_mutex);
    trajectory_msgs::JointTrajectory new_traj = shared_variables_.new_traj;

    // Check for stale cmds
    if ((ros::Time::now() - shared_variables_.incoming_cmd_stamp) <
        ros::Duration(ros_parameters_.incoming_command_timeout))
    {
      // Mark that incoming commands are not stale
      shared_variables_.command_is_stale = false;
    }
    else
    {
      shared_variables_.command_is_stale = true;
    }

    // Publish the most recent trajectory, unless the jogging calculation thread
    // tells not to
    if (shared_variables_.ok_to_publish)
    {
      // Put the outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
      if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
      {
        new_traj.header.stamp = ros::Time::now();
        outgoing_cmd_pub.publish(new_traj);
      }
      else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
      {
        std_msgs::Float64MultiArray joints;
        if (ros_parameters_.publish_joint_positions)
          joints.data = new_traj.points[0].positions;
        else if (ros_parameters_.publish_joint_velocities)
          joints.data = new_traj.points[0].velocities;
        outgoing_cmd_pub.publish(joints);
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, LOGNAME, "Stale or zero command. "
                                                 "Try a larger 'incoming_command_timeout' parameter?");
    }
    pthread_mutex_unlock(&shared_variables_.shared_variables_mutex);

    main_rate.sleep();
  }

  jogging_thread.join();
  collision_thread.join();
}

// A separate thread for the heavy jogging calculations.
bool JogROSInterface::startJogCalcThread(const JogArmParameters& parameters, JogArmShared& shared_variables,
  const robot_model_loader::RobotModelLoaderPtr model_loader_ptr)
{
  JogCalcs ja(parameters, shared_variables, model_loader_ptr);
  return true;
}

// A separate thread for collision checking.
bool JogROSInterface::startCollisionCheckThread(const JogArmParameters& parameters, JogArmShared& shared_variables,
  const robot_model_loader::RobotModelLoaderPtr& model_loader_ptr)
{
  collisionCheckThread cc(parameters, shared_variables, model_loader_ptr);
  return true;
}

// Constructor for the class that handles collision checking
collisionCheckThread::collisionCheckThread(const JogArmParameters parameters, JogArmShared& shared_variables,
                                           const robot_model_loader::RobotModelLoaderPtr model_loader_ptr)
{
  // If user specified true in yaml file
  if (parameters.check_collisions)
  {
    // MoveIt Setup
    // Wait for model_loader_ptr to be non-null.
    while (ros::ok() && !model_loader_ptr)
    {
      ROS_WARN_THROTTLE_NAMED(5, LOGNAME, "Waiting for a non-null robot_model_loader pointer");
      ros::Duration(WHILE_LOOP_WAIT).sleep();
    }
    const robot_model::RobotModelPtr& kinematic_model = model_loader_ptr->getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = parameters.move_group_name;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(model_loader_ptr));
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();

    if (planning_scene_monitor->getPlanningScene())
    {
      planning_scene_monitor->startSceneMonitor("/planning_scene");
      planning_scene_monitor->startWorldGeometryMonitor();
      planning_scene_monitor->startStateMonitor();
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    // Wait for initial messages
    ROS_INFO_NAMED(LOGNAME, "Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(parameters.joint_topic);
    ROS_INFO_NAMED(LOGNAME, "Received first joint msg.");

    ROS_INFO_NAMED(LOGNAME, "Waiting for first command msg.");
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>(parameters.cartesian_command_in_topic);
    ROS_INFO_NAMED(LOGNAME, "Received first command msg.");

    // A very low cutoff frequency
    LowPassFilter velocity_scale_filter(20);
    // Assume no scaling, initially
    velocity_scale_filter.reset(1);
    ros::Rate collision_rate(parameters.collision_check_rate);

    /////////////////////////////////////////////////
    // Spin while checking collisions
    /////////////////////////////////////////////////
    while (ros::ok())
    {
      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      sensor_msgs::JointState jts = shared_variables.joints;
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

      for (std::size_t i = 0; i < jts.position.size(); ++i)
        current_state.setJointPositions(jts.name[i], &jts.position[i]);

      collision_result.clear();
      planning_scene_monitor->getPlanningScene()->checkCollision(collision_request, collision_result, current_state);

      // Scale robot velocity according to collision proximity and user-defined
      // thresholds.
      // I scaled exponentially (cubic power) so velocity drops off quickly
      // after the threshold.
      // Proximity decreasing --> decelerate
      double velocity_scale = 1;

      // Ramp velocity down linearly when collision proximity is between
      // lower_collision_proximity_threshold and
      // hard_stop_collision_proximity_threshold
      if ((collision_result.distance > parameters.hard_stop_collision_proximity_threshold) &&
          (collision_result.distance < parameters.lower_collision_proximity_threshold))
      {
        // scale = k*(proximity-hard_stop_threshold)^3
        velocity_scale =
            64000. * pow(collision_result.distance - parameters.hard_stop_collision_proximity_threshold, 3);
      }
      else if (collision_result.distance < parameters.hard_stop_collision_proximity_threshold)
        velocity_scale = 0;

      velocity_scale = velocity_scale_filter.filter(velocity_scale);
      // Put a ceiling and a floor on velocity_scale
      if (velocity_scale > 1)
        velocity_scale = 1;
      else if (velocity_scale < 0.05)
        velocity_scale = 0.05;

      // Very slow if actually in collision
      if (collision_result.collision)
      {
        ROS_WARN_NAMED(LOGNAME, "Very close to collision. Slowing way down.");
        velocity_scale = 0.02;
      }

      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      shared_variables.collision_velocity_scale = velocity_scale;
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

      collision_rate.sleep();
    }
  }
}

// Constructor for the class that handles jogging calculations
JogCalcs::JogCalcs(const JogArmParameters parameters, JogArmShared& shared_variables,
                   const robot_model_loader::RobotModelLoaderPtr model_loader_ptr)
  : move_group_(parameters.move_group_name), tf_listener_(tf_buffer_), parameters_(parameters)
{
  // Publish collision status
  warning_pub_ = nh_.advertise<std_msgs::Bool>(parameters_.warning_topic, 1);

  // MoveIt Setup
  // Wait for model_loader_ptr to be non-null.
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

  ROS_INFO_NAMED(LOGNAME, "Waiting for first command msg.");
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(parameters_.cartesian_command_in_topic);
  ROS_INFO_NAMED(LOGNAME, "Received first command msg.");

  resetVelocityFilters();

  jt_state_.name = move_group_.getJointNames();
  jt_state_.position.resize(jt_state_.name.size());
  jt_state_.velocity.resize(jt_state_.name.size());
  jt_state_.effort.resize(jt_state_.name.size());

  // Low-pass filters for the joint positions & velocities
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    velocity_filters_.emplace_back(parameters_.low_pass_filter_coeff);
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // Initialize the position filters to initial robot joints
  while (!updateJoints() && ros::ok())
  {
    pthread_mutex_lock(&shared_variables.shared_variables_mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.shared_variables_mutex);
    ros::Duration(WHILE_LOOP_WAIT).sleep();
  }
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
    position_filters_[i].reset(jt_state_.position[i]);

  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  geometry_msgs::TwistStamped cartesian_deltas;
  moveit_msgs::JogJoint joint_deltas;
  while (ros::ok() && (cartesian_deltas.header.stamp == ros::Time(0.)) && (joint_deltas.header.stamp == ros::Time(0.)))
  {
    ros::Duration(WHILE_LOOP_WAIT).sleep();

    pthread_mutex_lock(&shared_variables.shared_variables_mutex);
    cartesian_deltas = shared_variables.command_deltas;
    joint_deltas = shared_variables.joint_command_deltas;
    pthread_mutex_unlock(&shared_variables.shared_variables_mutex);
  }

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count = 0;
  int num_zero_cycles_to_publish = 4;

  // Now do jogging calcs
  while (ros::ok())
  {
    // If user commands are all zero, reset the low-pass filters
    // when commands resume
    pthread_mutex_lock(&shared_variables.shared_variables_mutex);
    bool zero_cartesian_traj_flag = shared_variables.zero_cartesian_cmd_flag;
    bool zero_joint_traj_flag = shared_variables.zero_joint_cmd_flag;
    pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

    if (zero_cartesian_traj_flag && zero_joint_traj_flag)
      // Reset low-pass filters
      resetVelocityFilters();

    // Pull data from the shared variables.
    pthread_mutex_lock(&shared_variables.shared_variables_mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

    // Initialize the position filters to initial robot joints
    while (!updateJoints() && ros::ok())
    {
      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      incoming_jts_ = shared_variables.joints;
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);
      ros::Duration(WHILE_LOOP_WAIT).sleep();
    }

    // If there have not been several consecutive cycles of all zeros and joint
    // jogging commands are empty
    if ((zero_velocity_count <= num_zero_cycles_to_publish) && zero_joint_traj_flag)
    {
      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      cartesian_deltas = shared_variables.command_deltas;
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

      if (!cartesianJogCalcs(cartesian_deltas, shared_variables))
        continue;
    }
    // If there have not been several consecutive cycles of all zeros and joint
    // jogging commands are not empty
    else if ((zero_velocity_count <= num_zero_cycles_to_publish) && !zero_joint_traj_flag)
    {
      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      joint_deltas = shared_variables.joint_command_deltas;
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

      if (!jointJogCalcs(joint_deltas, shared_variables))
        continue;
    }

    // Halt if the command is stale or inputs are all zero, or commands were
    // zero
    pthread_mutex_lock(&shared_variables.shared_variables_mutex);
    bool stale_command = shared_variables.command_is_stale;
    pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

    if (stale_command || (zero_cartesian_traj_flag && zero_joint_traj_flag))
    {
      halt(new_traj_);
      zero_cartesian_traj_flag = true;
      zero_joint_traj_flag = true;
    }

    // Has the velocity command been zero for several cycles in a row?
    // If so, stop publishing so other controllers can take over
    bool valid_nonzero_trajectory =
        !((zero_velocity_count > num_zero_cycles_to_publish) && zero_cartesian_traj_flag && zero_joint_traj_flag);

    // Send the newest target joints
    if (!new_traj_.joint_names.empty())
    {
      pthread_mutex_lock(&shared_variables.shared_variables_mutex);
      // If everything normal, share the new traj to be published
      if (valid_nonzero_trajectory)
      {
        shared_variables.new_traj = new_traj_;
        shared_variables.ok_to_publish = true;
      }
      // Skip the jogging publication if all inputs have been zero for several
      // cycles in a row
      else if (zero_velocity_count > num_zero_cycles_to_publish)
      {
        shared_variables.ok_to_publish = false;
      }
      pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

      // Store last zero-velocity message flag to prevent superfluous warnings.
      // Cartesian and joint commands must both be zero.
      if (zero_cartesian_traj_flag && zero_joint_traj_flag)
        zero_velocity_count += 1;
      else
        zero_velocity_count = 0;
    }

    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(WHILE_LOOP_WAIT).sleep();
  }
}

// Perform the jogging calculations
bool JogCalcs::cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, JogArmShared& shared_variables)
{
  // Check for nan's in the incoming command
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_.command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Component of incoming command is >1. Skipping this datapoint.");
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
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

  if (!addJointIncrements(jt_state_, delta_theta))
    return false;

  // Include a velocity estimate for velocity-controlled robots
  Eigen::VectorXd joint_vel(delta_theta / parameters_.publish_period);

  lowPassFilterVelocities(joint_vel);
  lowPassFilterPositions();

  const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_period);
  new_traj_ = composeOutgoingMessage(jt_state_, next_time);

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(shared_variables, new_traj_, delta_theta, decelerateForSingularity(jacobian, delta_x));

  if (!checkIfJointsWithinBounds(new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }
  else
    publishWarning(false);

  // If using Gazebo simulator, insert redundant points
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(new_traj_, gazebo_redundant_message_count_);
  }

  return true;
}

bool JogCalcs::jointJogCalcs(const moveit_msgs::JogJoint& cmd, JogArmShared& shared_variables)
{
  // Check for nan's or |delta|>1 in the incoming command
  for (std::size_t i = 0; i < cmd.deltas.size(); ++i)
  {
    if (std::isnan(cmd.deltas[i]) || (fabs(cmd.deltas[i]) > 1))
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "nan in incoming command. Skipping this datapoint.");
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

  const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_delay);
  new_traj_ = composeOutgoingMessage(jt_state_, next_time);

  // check if new joint state is valid
  if (!checkIfJointsWithinBounds(new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }
  else
  {
    publishWarning(false);
  }

  // done with calculations
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(new_traj_, gazebo_redundant_message_count_);
  }

  return true;
}

// Spam several redundant points into the trajectory. The first few may be
// skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo
// simulation.
// Start from 2 because the first point's timestamp is already
// 1*parameters_.publish_period
void JogCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const
{
  auto point = trajectory.points[0];
  // Start from 2 because we already have the first point. End at count+1 so
  // total # == count
  for (int i = 2; i < count + 1; ++i)
  {
    point.time_from_start = ros::Duration(i * parameters_.publish_period);
    trajectory.points.push_back(point);
  }
}

void JogCalcs::lowPassFilterPositions()
{
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
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
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_.velocity[i] = velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

    // Check for nan's
    if (std::isnan(jt_state_.velocity[static_cast<long>(i)]))
    {
      jt_state_.position[i] = original_jt_state_.position[i];
      jt_state_.velocity[i] = 0.;
      ROS_WARN_STREAM_NAMED(LOGNAME, "nan in velocity filter");
    }
  }
}

trajectory_msgs::JointTrajectory JogCalcs::composeOutgoingMessage(sensor_msgs::JointState& joint_state,
                                                                  const ros::Time& stamp) const
{
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.frame_id = parameters_.planning_frame;
  new_jt_traj.header.stamp = stamp;
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
    std::vector<double> acceleration(joint_state.velocity.size());
    point.accelerations = acceleration;
  }
  new_jt_traj.points.push_back(point);

  return new_jt_traj;
}

// Apply velocity scaling for proximity of collisions and singularities.
// Scale for collisions is read from a shared variable.
// Key equation: new_velocity =
// collision_scale*singularity_scale*previous_velocity
bool JogCalcs::applyVelocityScaling(JogArmShared& shared_variables, trajectory_msgs::JointTrajectory& new_jt_traj,
                                    const Eigen::VectorXd& delta_theta, double singularity_scale)
{
  pthread_mutex_unlock(&shared_variables.shared_variables_mutex);
  double collision_scale = shared_variables.collision_velocity_scale;
  pthread_mutex_unlock(&shared_variables.shared_variables_mutex);

  for (size_t i = 0; i < jt_state_.velocity.size(); ++i)
  {
    if (parameters_.publish_joint_positions)
    {
      // If close to a singularity or joint limit, undo any change to the joint
      // angles
      new_jt_traj.points[0].positions[i] =
          new_jt_traj.points[0].positions[i] -
          (1. - singularity_scale * collision_scale) * delta_theta[static_cast<long>(i)];
    }
    if (parameters_.publish_joint_velocities)
      new_jt_traj.points[0].velocities[i] *= singularity_scale * collision_scale;
  }

  return true;
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity
// and direction of motion
double JogCalcs::decelerateForSingularity(Eigen::MatrixXd jacobian, const Eigen::VectorXd commanded_velocity)
{
  double velocity_scale = 1;

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points away from the
  // singularity
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU);
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(5);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity
  // in the Singular Value Decomposition"
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction.
  // Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(6);
  double scale = 100;
  delta_x[0] = vector_toward_singularity[0] / scale;
  delta_x[1] = vector_toward_singularity[1] / scale;
  delta_x[2] = vector_toward_singularity[2] / scale;
  delta_x[3] = vector_toward_singularity[3] / scale;
  delta_x[4] = vector_toward_singularity[4] / scale;
  delta_x[5] = vector_toward_singularity[5] / scale;

  // Calculate a small change in joints
  Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

  double theta[6];
  const double* prev_joints = kinematic_state_->getVariablePositions();
  for (std::size_t i = 0, size = static_cast<std::size_t>(delta_theta.size()); i < size; ++i)
    theta[i] = prev_joints[i] + delta_theta(i);

  kinematic_state_->setVariablePositions(theta);
  jacobian = kinematic_state_->getJacobian(joint_model_group_);
  svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian);
  double new_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity.
  //  Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity[0] *= -1;
    vector_toward_singularity[1] *= -1;
    vector_toward_singularity[2] *= -1;
    vector_toward_singularity[3] *= -1;
    vector_toward_singularity[4] *= -1;
    vector_toward_singularity[5] *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==>
  // decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between
    // lower_singularity_threshold and
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
      ROS_WARN_NAMED(LOGNAME, "Close to a singularity. Halting.");
    }
  }

  return velocity_scale;
}

bool JogCalcs::checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory& new_jt_traj)
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
      for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c)
      {
        if (new_jt_traj.joint_names[c] == joint->getName())
        {
          new_jt_traj.points[0].velocities[c] = kinematic_state_->getJointVelocities(joint)[0];
          break;
        }
      }
    }

    // Halt if we're past a joint margin and joint velocity is moving even
    // farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c)
    {
      if (original_jt_state_.name[c] == joint->getName())
      {
        joint_angle = original_jt_state_.position.at(c);
        break;
      }
    }

    if (!kinematic_state_->satisfiesPositionBounds(joint,
                                                   -parameters_.joint_limit_margin))
    {
      const std::vector<moveit_msgs::JointLimits> limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (limits.size() > 0)
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

void JogCalcs::publishWarning(const bool active) const
{
  std_msgs::Bool status;
  status.data = static_cast<std_msgs::Bool::_data_type>(active);
  warning_pub_.publish(status);
}

// Avoid a singularity or other issue.
// Needs to be handled differently for position vs. velocity control
void JogCalcs::halt(trajectory_msgs::JointTrajectory& jt_traj)
{
  for (std::size_t i = 0; i < jt_state_.velocity.size(); ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good
    // state
    if (parameters_.publish_joint_positions)
      jt_traj.points[0].positions[i] = original_jt_state_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_.publish_joint_velocities)
      jt_traj.points[0].velocities[i] = 0;
  }
}

// Reset the data stored in filters so the trajectory won't jump when jogging is
// resumed.
void JogCalcs::resetVelocityFilters()
{
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
    velocity_filters_[i].reset(0);  // Zero velocity
}

// Parse the incoming joint msg for the joints of our MoveGroup
bool JogCalcs::updateJoints()
{
  // Check if every joint was zero. Sometimes an issue.
  bool all_zeros = true;

  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < jt_state_.name.size())
    return false;

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_jts_.name.size(); ++m)
  {
    for (std::size_t c = 0; c < jt_state_.name.size(); ++c)
    {
      if (incoming_jts_.name[m] == jt_state_.name[c])
      {
        jt_state_.position[c] = incoming_jts_.position[m];
        // Make sure there was at least one nonzero value
        if (incoming_jts_.position[m] != 0.)
          all_zeros = false;
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }

  return !all_zeros;
}

// Scale the incoming jog command
Eigen::VectorXd JogCalcs::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_.command_in_type == "unitless")
  {
    result[0] = parameters_.linear_scale * command.twist.linear.x;
    result[1] = parameters_.linear_scale * command.twist.linear.y;
    result[2] = parameters_.linear_scale * command.twist.linear.z;
    result[3] = parameters_.rotational_scale * command.twist.angular.x;
    result[4] = parameters_.rotational_scale * command.twist.angular.y;
    result[5] = parameters_.rotational_scale * command.twist.angular.z;
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

Eigen::VectorXd JogCalcs::scaleJointCommand(const moveit_msgs::JogJoint& command) const
{
  Eigen::VectorXd result(jt_state_.name.size());

  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    result[i] = 0.0;
  }

  // Store joints in a member variable
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    for (std::size_t c = 0; c < jt_state_.name.size(); ++c)
    {
      if (command.joint_names[m] == jt_state_.name[c])
      {
        // Apply user-defined scaling if inputs are unitless [-1:1]
        if (parameters_.command_in_type == "unitless")
          result[c] = command.deltas[m] * parameters_.joint_scale;
        // Otherwise, commands are in m/s and rad/s
        else if (parameters_.command_in_type == "speed_units")
          result[c] = command.deltas[m] * parameters_.publish_period;
        else
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unexpected command_in_type");
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }

  return result;
}

// Calculate a pseudo-inverse.
Eigen::MatrixXd JogCalcs::pseudoInverse(const Eigen::MatrixXd& J) const
{
  return J.transpose() * (J * J.transpose()).inverse();
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

// Listen to cartesian delta commands.
// Store them in a shared variable.
void JogROSInterface::deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.shared_variables_mutex);

  // Copy everything but the frame name. The frame name is set by yaml file at startup.
  // (so it doesn't need to be copied over and over)
  shared_variables_.command_deltas.twist = msg->twist;
  shared_variables_.command_deltas.header.stamp = msg->header.stamp;

  // Check if input is all zeros. Flag it if so to skip calculations/publication
  shared_variables_.zero_cartesian_cmd_flag = shared_variables_.command_deltas.twist.linear.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.z == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.z == 0.0;

  shared_variables_.incoming_cmd_stamp = msg->header.stamp;
  pthread_mutex_unlock(&shared_variables_.shared_variables_mutex);
}

// Listen to joint delta commands.
// Store them in a shared variable.
void JogROSInterface::deltaJointCmdCB(const moveit_msgs::JogJointConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.shared_variables_mutex);
  shared_variables_.joint_command_deltas = *msg;

  // Input frame determined by YAML file
  shared_variables_.joint_command_deltas.header.frame_id = ros_parameters_.command_frame;

  // Check if joint inputs is all zeros. Flag it if so to skip
  // calculations/publication
  bool all_zeros = true;
  for (double delta : shared_variables_.joint_command_deltas.deltas)
  {
    all_zeros &= (delta == 0.0);
  };
  shared_variables_.zero_joint_cmd_flag = all_zeros;

  shared_variables_.incoming_cmd_stamp = msg->header.stamp;
  pthread_mutex_unlock(&shared_variables_.shared_variables_mutex);
}

// Listen to joint angles.
// Store them in a shared variable.
void JogROSInterface::jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.shared_variables_mutex);
  shared_variables_.joints = *msg;
  pthread_mutex_unlock(&shared_variables_.shared_variables_mutex);
}

// Read ROS parameters, typically from YAML file
bool JogROSInterface::readParameters(ros::NodeHandle& n)
{
  std::size_t error = 0;

  // Specified in the launch file. All other parameters will be read
  // from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);
  if (parameter_ns == "")
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "A namespace must be specified in the launch file, like:");
    ROS_ERROR_STREAM_NAMED(LOGNAME, "<param name=\"parameter_ns\" "
                                    "type=\"string\" "
                                    "value=\"left_jog_arm_server\" />");
    return false;
  }

  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_period", ros_parameters_.publish_period);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_delay", ros_parameters_.publish_delay);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/collision_check_rate", ros_parameters_.collision_check_rate);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/linear", ros_parameters_.linear_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/rotational", ros_parameters_.rotational_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/scale/joint", ros_parameters_.joint_scale);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/low_pass_filter_coeff", ros_parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/joint_topic", ros_parameters_.joint_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_in_type", ros_parameters_.command_in_type);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/cartesian_command_in_topic",
                                    ros_parameters_.cartesian_command_in_topic);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/joint_command_in_topic", ros_parameters_.joint_command_in_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_frame", ros_parameters_.command_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/incoming_command_timeout",
                                    ros_parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/lower_singularity_threshold",
                                    ros_parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/hard_stop_singularity_threshold",
                                    ros_parameters_.hard_stop_singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/lower_collision_proximity_threshold",
                                    ros_parameters_.lower_collision_proximity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/hard_stop_collision_proximity_threshold",
                                    ros_parameters_.hard_stop_collision_proximity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/move_group_name", ros_parameters_.move_group_name);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/planning_frame", ros_parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/use_gazebo", ros_parameters_.use_gazebo);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/check_collisions", ros_parameters_.check_collisions);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/warning_topic", ros_parameters_.warning_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/joint_limit_margin", ros_parameters_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_topic", ros_parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_type", ros_parameters_.command_out_type);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_positions",
                                    ros_parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_velocities",
                                    ros_parameters_.publish_joint_velocities);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_accelerations",
                                    ros_parameters_.publish_joint_accelerations);

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Set the input frame, as determined by YAML file:
  pthread_mutex_lock(&shared_variables_.shared_variables_mutex);
  shared_variables_.command_deltas.header.frame_id = ros_parameters_.command_frame;
  pthread_mutex_unlock(&shared_variables_.shared_variables_mutex);

  // Input checking
  if (ros_parameters_.hard_stop_singularity_threshold < ros_parameters_.lower_singularity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_singularity_threshold' "
                            "should be greater than 'lower_singularity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((ros_parameters_.hard_stop_singularity_threshold < 0.) || (ros_parameters_.lower_singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_singularity_threshold' "
                            "and 'lower_singularity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.hard_stop_collision_proximity_threshold >= ros_parameters_.lower_collision_proximity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_collision_proximity_threshold' "
                            "should be less than 'lower_collision_proximity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((ros_parameters_.hard_stop_collision_proximity_threshold < 0.) ||
      (ros_parameters_.lower_collision_proximity_threshold < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_collision_proximity_threshold' "
                            "and 'lower_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'low_pass_filter_coeff' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.joint_limit_margin < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'joint_limit_margin' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.command_in_type != "unitless" && ros_parameters_.command_in_type != "speed_units")
  {
    ROS_WARN_NAMED(LOGNAME, "command_in_type should be 'unitless' or "
                            "'speed_units'. Check yaml file.");
    return false;
  }
  if (ros_parameters_.command_out_type != "trajectory_msgs/JointTrajectory" &&
      ros_parameters_.command_out_type != "std_msgs/Float64MultiArray")
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter command_out_type should be "
                            "'trajectory_msgs/JointTrajectory' or "
                            "'std_msgs/Float64MultiArray'. Check yaml file.");
    return false;
  }
  if (!ros_parameters_.publish_joint_positions && !ros_parameters_.publish_joint_velocities &&
      !ros_parameters_.publish_joint_accelerations)
  {
    ROS_WARN_NAMED(LOGNAME, "At least one of publish_joint_positions / "
                            "publish_joint_velocities / "
                            "publish_joint_accelerations must be true. Check "
                            "yaml file.");
    return false;
  }
  if ((ros_parameters_.command_out_type == "std_msgs/Float64MultiArray") && ros_parameters_.publish_joint_positions &&
      ros_parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(LOGNAME, "When publishing a std_msgs/Float64MultiArray, "
                            "you must select positions OR velocities.");
    return false;
  }
  if (ros_parameters_.collision_check_rate < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'collision_check_rate' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }

  return true;
}

LowPassFilter::LowPassFilter(double low_pass_filter_coeff)
{
  filter_coeff_ = low_pass_filter_coeff;
}

void LowPassFilter::reset(double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;
  previous_measurements_[2] = data;

  previous_filtered_measurements_[0] = data;
  previous_filtered_measurements_[1] = data;
}

double LowPassFilter::filter(double new_measurement_)
{
  // Push in the new measurement
  previous_measurements_[2] = previous_measurements_[1];
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement_;

  double new_filtered_msrmt =
      (1. / (1. + filter_coeff_ * filter_coeff_ + 1.414 * filter_coeff_)) *
      (previous_measurements_[2] + 2. * previous_measurements_[1] + previous_measurements_[0] -
       (filter_coeff_ * filter_coeff_ - 1.414 * filter_coeff_ + 1.) * previous_filtered_measurements_[1] -
       (-2. * filter_coeff_ * filter_coeff_ + 2.) * previous_filtered_measurements_[0]);

  // Store the new filtered measurement
  previous_filtered_measurements_[1] = previous_filtered_measurements_[0];
  previous_filtered_measurements_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}
}  // namespace jog_arm
