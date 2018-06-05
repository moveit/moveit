///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.cpp
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

#include <jog_arm/jog_arm_server.h>

/////////////////////////////////////////////////
// MAIN handles ROS subscriptions.
// A worker thread does the jogging calculations.
// Another worker thread does collision checking.
/////////////////////////////////////////////////

// MAIN: create the worker thread and subscribe to jogging cmds and joint angles
int main(int argc, char **argv) {
  ros::init(argc, argv, "jog_arm_server");
  ros::NodeHandle n;

  // Read ROS parameters, typically from YAML file
  if (jog_arm::readParameters(n))
    return 1;

  // Crunch the numbers in this thread
  pthread_t joggingThread;
  int rc = pthread_create(&joggingThread, NULL, jog_arm::joggingPipeline, 0);

  // Check collisions in this thread
  pthread_t collisionThread;
  rc = pthread_create(&collisionThread, NULL, jog_arm::collisionCheck, 0);

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub =
      n.subscribe(jog_arm::g_parameters.command_in_topic, 1, jog_arm::deltaCmdCB);
  ros::Subscriber joints_sub =
      n.subscribe(jog_arm::g_parameters.joint_topic, 1, jog_arm::jointsCB);

  // Publish freshly-calculated joints to the robot
  ros::Publisher joint_trajectory_pub =
      n.advertise<trajectory_msgs::JointTrajectory>(
          jog_arm::g_parameters.command_out_topic, 1);

  ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::g_parameters.joint_topic);
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
      jog_arm::g_parameters.command_in_topic);

  // Wait for jog filters to stablize
  ros::Duration(10 * jog_arm::g_parameters.publish_period).sleep();

  ros::Rate main_rate(1. / jog_arm::g_parameters.publish_period);

  while (ros::ok()) {
    ros::spinOnce();

    // Send the newest target joints
    pthread_mutex_lock(&jog_arm::g_new_traj_mutex);
    if (jog_arm::g_new_traj.joint_names.size() != 0) {
      // Check for stale cmds
      if (ros::Time::now() - jog_arm::g_new_traj.header.stamp <
          ros::Duration(jog_arm::g_parameters.incoming_command_timeout)) {
        // Skip the jogging publication if all inputs are 0.
        pthread_mutex_lock(&jog_arm::g_zero_trajectory_flagmutex);
        if (!jog_arm::g_zero_trajectory_flag) {
          jog_arm::g_new_traj.header.stamp = ros::Time::now();
          joint_trajectory_pub.publish(jog_arm::g_new_traj);
        }
        pthread_mutex_unlock(&jog_arm::g_zero_trajectory_flagmutex);
      } else {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "jog_arm_server",
                                       "Stale joint "
                                       "trajectory msg. Try a larger "
                                       "'incoming_command_timeout' parameter.");
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "jog_arm_server",
                                       "Did input from the "
                                       "controller get interrupted? Are "
                                       "calculations taking too long?");
      }
    }
    pthread_mutex_unlock(&jog_arm::g_new_traj_mutex);

    main_rate.sleep();
  }


  (void) pthread_join(joggingThread, NULL);
  (void) pthread_join(collisionThread, NULL);

  return 0;
}

namespace jog_arm {
// A separate thread for the heavy jogging calculations.
void *joggingPipeline(void *) {
  jog_arm::JogCalcs ja(jog_arm::g_parameters.move_group_name);
  return nullptr;
}

// A separate thread for collision checking.
void *collisionCheck(void *) {
  jog_arm::CollisionCheck cc(jog_arm::g_parameters.move_group_name);
  return nullptr;
}

// Constructor for the class that handles collision checking
CollisionCheck::CollisionCheck(const std::string &move_group_name) {
  // If user specified true in yaml file
  if (jog_arm::g_parameters.collision_check) {
    // Publish collision status
    warning_pub_ = nh_.advertise<std_msgs::Bool>(jog_arm::g_parameters.warning_topic, 1);
    std_msgs::Bool collision_status;

    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    const robot_model::RobotModelPtr &kinematic_model =
        robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = move_group_name;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState &current_state =
        planning_scene.getCurrentStateNonConst();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Wait for initial joint message
    ROS_INFO_NAMED("jog_arm_server", "Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::g_parameters.joint_topic);
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
        g_parameters.command_in_topic);
    ROS_INFO_NAMED("jog_arm_server", "Received first joint msg.");

    ros::Rate collision_rate(100);

    /////////////////////////////////////////////////
    // Spin while checking collisions
    /////////////////////////////////////////////////
    while (ros::ok()) {
      pthread_mutex_lock(&g_joints_mutex);
      sensor_msgs::JointState jts = jog_arm::g_joints;
      pthread_mutex_unlock(&g_joints_mutex);

      for (std::size_t i = 0; i < jts.position.size(); i++)
        current_state.setJointPositions(jts.name[i], &jts.position[i]);

      // process collision objects in scene
      std::map<std::string, moveit_msgs::CollisionObject> c_objects_map =
          planning_scene_interface.getObjects();
      for (auto &kv : c_objects_map) {
        planning_scene.processCollisionObjectMsg(kv.second);
      }

      collision_result.clear();
      planning_scene.checkCollision(collision_request, collision_result);

      // If collision, signal the jogging to stop
      if (collision_result.collision) {
        pthread_mutex_lock(&jog_arm::g_imminent_collision_mutex);
        jog_arm::g_imminent_collision = true;
        pthread_mutex_unlock(&jog_arm::g_imminent_collision_mutex);

        collision_status.data = true;
        warning_pub_.publish(collision_status);
      } else {
        pthread_mutex_lock(&jog_arm::g_imminent_collision_mutex);
        jog_arm::g_imminent_collision = false;
        pthread_mutex_unlock(&jog_arm::g_imminent_collision_mutex);
      }

      ros::spinOnce();
      collision_rate.sleep();
    }
  }
}

// Constructor for the class that handles jogging calculations
JogCalcs::JogCalcs(const std::string &move_group_name)
    : arm_(move_group_name), prev_time_(ros::Time::now()) {
  // Publish collision status
  warning_pub_ = nh_.advertise<std_msgs::Bool>(jog_arm::g_parameters.warning_topic, 1);

  // MoveIt Setup
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  const robot_model::RobotModelPtr &kinematic_model = model_loader.getModel();

  kinematic_state_ = std::shared_ptr<robot_state::RobotState>(
      new robot_state::RobotState(kinematic_model));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name);

  const std::vector<std::string> &joint_names =
      joint_model_group_->getJointModelNames();
  std::vector<double> dummy_joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_,
                                            dummy_joint_values);

  // Wait for initial messages
  ROS_INFO_NAMED("jog_arm_server", "Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>(jog_arm::g_parameters.joint_topic);
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
      g_parameters.command_in_topic);
  ROS_INFO_NAMED("jog_arm_server", "Received first joint msg.");

  jt_state_.name = arm_.getJointNames();
  jt_state_.position.resize(jt_state_.name.size());
  jt_state_.velocity.resize(jt_state_.name.size());
  jt_state_.effort.resize(jt_state_.name.size());

  // Low-pass filters for the joint positions & velocities
  for (std::size_t i = 0; i < joint_names.size(); i++) {
    velocity_filters_.push_back(
        jog_arm::LowPassFilter(jog_arm::g_parameters.low_pass_filter_coeff));
    position_filters_.push_back(
        jog_arm::LowPassFilter(jog_arm::g_parameters.low_pass_filter_coeff));
  }

  // Initialize the position filters to initial robot joints
  pthread_mutex_lock(&g_joints_mutex);
  incoming_jts_ = jog_arm::g_joints;
  pthread_mutex_unlock(&g_joints_mutex);
  updateJoints();
  for (std::size_t i = 0; i < jt_state_.name.size(); i++)
    position_filters_[i].reset(jt_state_.position[i]);

  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  while (cmd_deltas_.header.stamp == ros::Time(0.)) {
    ros::Duration(0.05).sleep();
    pthread_mutex_lock(&g_command_deltas_mutex);
    cmd_deltas_ = jog_arm::g_command_deltas;
    pthread_mutex_unlock(&g_command_deltas_mutex);
  }

  // Now do jogging calcs
  while (ros::ok()) {
    // If user commands are all zero, reset the low-pass filters
    // when commands resume
    pthread_mutex_lock(&jog_arm::g_zero_trajectory_flagmutex);
    bool flag = jog_arm::g_zero_trajectory_flag;
    pthread_mutex_unlock(&jog_arm::g_zero_trajectory_flagmutex);
    if (flag)
      // Reset low-pass filters
      resetVelocityFilters();

    // Pull data from the shared variables.
    pthread_mutex_lock(&g_command_deltas_mutex);
    cmd_deltas_ = jog_arm::g_command_deltas;
    pthread_mutex_unlock(&g_command_deltas_mutex);

    pthread_mutex_lock(&g_joints_mutex);
    incoming_jts_ = jog_arm::g_joints;
    pthread_mutex_unlock(&g_joints_mutex);

    updateJoints();

    jogCalcs(cmd_deltas_);

    // Generally want to run these calculations fast.
    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(0.001).sleep();
  }
}

// Perform the jogging calculations
void JogCalcs::jogCalcs(const geometry_msgs::TwistStamped &cmd) {
  // Convert the cmd to the MoveGroup planning frame.
  try {
    listener_.waitForTransform(cmd.header.frame_id, jog_arm::g_parameters.planning_frame,
                               ros::Time::now(), ros::Duration(0.2));
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM_NAMED("jog_arm_server", ros::this_node::getName()
                                                 << ": " << ex.what());
    return;
  }
  // To transform, these vectors need to be stamped. See answers.ros.org
  // Q#199376
  // Transform the linear component of the cmd message
  geometry_msgs::Vector3Stamped lin_vector;
  lin_vector.vector = cmd.twist.linear;
  lin_vector.header.frame_id = cmd.header.frame_id;
  try {
    listener_.transformVector(jog_arm::g_parameters.planning_frame, lin_vector,
                              lin_vector);
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM_NAMED("jog_arm_server", ros::this_node::getName()
                                                 << ": " << ex.what());
    return;
  }

  geometry_msgs::Vector3Stamped rot_vector;
  rot_vector.vector = cmd.twist.angular;
  rot_vector.header.frame_id = cmd.header.frame_id;
  try {
    listener_.transformVector(jog_arm::g_parameters.planning_frame, rot_vector,
                              rot_vector);
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM_NAMED("jog_arm_server", ros::this_node::getName()
                                                 << ": " << ex.what());
    return;
  }

  // Put these components back into a TwistStamped
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.header.stamp = cmd.header.stamp;
  twist_cmd.header.frame_id = jog_arm::g_parameters.planning_frame;
  twist_cmd.twist.linear = lin_vector.vector;
  twist_cmd.twist.angular = rot_vector.vector;

  // Apply user-defined scaling
  const Vector6d delta_x = scaleCommand(twist_cmd);

  kinematic_state_->setVariableValues(jt_state_);
  orig_jts_ = jt_state_;

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

  // This inner loop may execute slower or faster than the desired rate. Scale
  // these joint
  // commands to match the desired rate. Then the velocity will match the user's
  // expectations.
  delta_t_ = (ros::Time::now() - prev_time_).toSec();
  prev_time_ = ros::Time::now();
  delta_theta(0) *= jog_arm::g_parameters.publish_period / delta_t_;
  delta_theta(1) *= jog_arm::g_parameters.publish_period / delta_t_;
  delta_theta(2) *= jog_arm::g_parameters.publish_period / delta_t_;
  delta_theta(3) *= jog_arm::g_parameters.publish_period / delta_t_;
  delta_theta(4) *= jog_arm::g_parameters.publish_period / delta_t_;
  delta_theta(5) *= jog_arm::g_parameters.publish_period / delta_t_;

  if (!addJointIncrements(jt_state_, delta_theta))
    return;

  // Check the Jacobian with these new joints.
  kinematic_state_->setVariableValues(jt_state_);
  jacobian = kinematic_state_->getJacobian(joint_model_group_);

  // Include a velocity estimate for velocity-controller robots
  Eigen::VectorXd joint_vel(delta_theta / delta_t_);

  // Low-pass filter the velocities
  for (std::size_t i = 0; i < jt_state_.name.size(); i++) {
    joint_vel[static_cast<long>(i)] =
        velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

    // Check for nan's
    if (std::isnan(joint_vel[static_cast<long>(i)]))
      joint_vel[static_cast<long>(i)] = 0.;
  }
  updateJointVels(jt_state_, joint_vel);

  // Low-pass filter the positions
  for (std::size_t i = 0; i < jt_state_.name.size(); i++) {
    jt_state_.position[i] = position_filters_[i].filter(jt_state_.position[i]);

    // Check for nan's
    if (std::isnan(jt_state_.position[i]))
      jt_state_.position[i] = 0.;
  }

  // Compose the outgoing msg
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.frame_id = jog_arm::g_parameters.planning_frame;
  new_jt_traj.header.stamp = cmd.header.stamp;
  new_jt_traj.joint_names = jt_state_.name;
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = jt_state_.position;
  point.time_from_start = ros::Duration(jog_arm::g_parameters.publish_period);
  point.velocities = jt_state_.velocity;

  new_jt_traj.points.push_back(point);

  // Stop if imminent collision
  pthread_mutex_lock(&jog_arm::g_imminent_collision_mutex);
  bool collision = jog_arm::g_imminent_collision;
  pthread_mutex_unlock(&jog_arm::g_imminent_collision_mutex);
  if (collision) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "jog_arm_server",
                                    ros::this_node::getName()
                                        << " Close to a collision. Halting.");

    halt(new_jt_traj);
  }

  // Verify that the future Jacobian is well-conditioned before moving.
  // Slow down if very close to a singularity.
  // Stop if extremely close.
  double current_condition_number = checkConditionNumber(jacobian);
  if (current_condition_number > jog_arm::g_parameters.singularity_threshold) {
    if (current_condition_number > jog_arm::g_parameters.hard_stop_singularity_threshold) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(2, "jog_arm_server",
                                      ros::this_node::getName()
                                          << " Close to a "
                                             "singularity ("
                                          << current_condition_number
                                          << "). Halting.");

      halt(new_jt_traj);

      std_msgs::Bool singularity_status;
      singularity_status.data = true;
      warning_pub_.publish(singularity_status);
    }
    // Only somewhat close to singularity. Just slow down.
    else {
      for (std::size_t i = 0; i < jt_state_.velocity.size(); i++) {
        new_jt_traj.points[0].positions[i] =
            new_jt_traj.points[0].positions[i] -
            0.7 * delta_theta[static_cast<long>(i)];
        new_jt_traj.points[0].velocities[i] *= 0.3;
      }
    }
  }

  // Check if new joints would be within bounds
  if (!kinematic_state_->satisfiesBounds(joint_model_group_)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(
        2, "jog_arm_server", ros::this_node::getName()
                                 << " Close to a "
                                    "position or velocity limit. Halting.");

    halt(new_jt_traj);

    std_msgs::Bool limit_status;
    limit_status.data = true;
    warning_pub_.publish(limit_status);
  }

  if (jog_arm::g_parameters.gazebo)
    // Spam several redundant points into the trajectory. The first few may be
    // skipped if the
    // time stamp is in the past when it reaches the client. Needed for gazebo
    // simulation.
    // Start from 2 because the first point's timestamp is already
    // 1*jog_arm::g_parameters.publish_period
    point = new_jt_traj.points[0];
  for (int i = 2; i < 30; i++) {
    point.time_from_start = ros::Duration(i * jog_arm::g_parameters.publish_period);
    new_jt_traj.points.push_back(point);
  }

  // Share with main to be published
  pthread_mutex_lock(&jog_arm::g_new_traj_mutex);
  jog_arm::g_new_traj = new_jt_traj;
  pthread_mutex_unlock(&jog_arm::g_new_traj_mutex);
}

// Halt the robot
void JogCalcs::halt(trajectory_msgs::JointTrajectory &jt_traj) {
  for (std::size_t i = 0; i < jt_state_.velocity.size(); i++) {
    jt_traj.points[0].positions[i] = orig_jts_.position[i];
    jt_traj.points[0].velocities[i] = 0.;
  }
  // Store all zeros in the velocity filter
  resetVelocityFilters();
}

// Reset the data stored in filters so the trajectory won't jump when jogging is
// resumed.
void JogCalcs::resetVelocityFilters() {
  for (std::size_t i = 0; i < jt_state_.name.size(); i++)
    velocity_filters_[i].reset(0); // Zero velocity
}

// Update joint velocities
bool JogCalcs::updateJointVels(sensor_msgs::JointState &output,
                               const Eigen::VectorXd &joint_vels) const {
  for (std::size_t i = 0, size = static_cast<std::size_t>(joint_vels.size());
       i < size; ++i) {
    try {
      output.velocity[i] = joint_vels(static_cast<long>(i));
    } catch (std::out_of_range e) {
      ROS_ERROR_STREAM_NAMED("jog_arm_server",
                             ros::this_node::getName()
                                 << " Vector lengths do not match.");
      return false;
    }
  }

  return true;
}

// Parse the incoming joint msg for the joints of our MoveGroup
void JogCalcs::updateJoints() {
  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < jt_state_.name.size()) {
    ROS_WARN_NAMED("JogCalcs", "The joint msg does not contain enough "
                               "joints.");
    return;
  }

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_jts_.name.size(); m++) {
    for (std::size_t c = 0; c < jt_state_.name.size(); c++) {
      if (incoming_jts_.name[m] == jt_state_.name[c]) {
        jt_state_.position[c] = incoming_jts_.position[m];
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }
}

// Scale the incoming jog command
JogCalcs::Vector6d
JogCalcs::scaleCommand(const geometry_msgs::TwistStamped &command) const {
  Vector6d result;

  result(0) = jog_arm::g_parameters.linear_scale * command.twist.linear.x;
  result(1) = jog_arm::g_parameters.linear_scale * command.twist.linear.y;
  result(2) = jog_arm::g_parameters.linear_scale * command.twist.linear.z;
  result(3) = jog_arm::g_parameters.rotational_scale * command.twist.angular.x;
  result(4) = jog_arm::g_parameters.rotational_scale * command.twist.angular.y;
  result(5) = jog_arm::g_parameters.rotational_scale * command.twist.angular.z;

  return result;
}

// Calculate a pseudo-inverse.
Eigen::MatrixXd JogCalcs::pseudoInverse(const Eigen::MatrixXd &J) const {
  Eigen::MatrixXd transpose = J.transpose();
  return transpose * (J * transpose).inverse();
}

// Add the deltas to each joint
bool JogCalcs::addJointIncrements(sensor_msgs::JointState &output,
                                  const Eigen::VectorXd &increments) const {
  for (std::size_t i = 0, size = static_cast<std::size_t>(increments.size());
       i < size; ++i) {
    try {
      output.position[i] += increments(static_cast<long>(i));
    } catch (std::out_of_range e) {
      ROS_ERROR_STREAM_NAMED("jog_arm_server",
                             ros::this_node::getName()
                                 << " Lengths of output and "
                                    "increments do not match.");
      return false;
    }
  }

  return true;
}

/// Calculate the condition number of the jacobian, to check for singularities
double JogCalcs::checkConditionNumber(const Eigen::MatrixXd &matrix) const {
  // Get Eigenvalues
  Eigen::MatrixXd::EigenvaluesReturnType eigs = matrix.eigenvalues();
  Eigen::VectorXd eig_vector = eigs.cwiseAbs();

  // CN = max(eigs)/min(eigs)
  double min = eig_vector.minCoeff();
  double max = eig_vector.maxCoeff();

  double condition_number = max / min;

  return condition_number;
}

// Listen to cartesian delta commands.
// Store them in a shared variable.
void deltaCmdCB(const geometry_msgs::TwistStampedConstPtr &msg) {
  pthread_mutex_lock(&g_command_deltas_mutex);
  jog_arm::g_command_deltas = *msg;
  // Input frame determined by YAML file:
  jog_arm::g_command_deltas.header.frame_id = jog_arm::g_parameters.command_frame;
  pthread_mutex_unlock(&g_command_deltas_mutex);

  // Check if input is all zeros. Flag it if so to skip calculations/publication
  pthread_mutex_lock(&jog_arm::g_zero_trajectory_flagmutex);
  if (jog_arm::g_command_deltas.twist.linear.x == 0 &&
      jog_arm::g_command_deltas.twist.linear.y == 0 &&
      jog_arm::g_command_deltas.twist.linear.z == 0 &&
      jog_arm::g_command_deltas.twist.angular.x == 0 &&
      jog_arm::g_command_deltas.twist.linear.y == 0 &&
      jog_arm::g_command_deltas.twist.linear.z == 0)
    jog_arm::g_zero_trajectory_flag = true;
  else
    jog_arm::g_zero_trajectory_flag = false;
  pthread_mutex_unlock(&jog_arm::g_zero_trajectory_flagmutex);
}

// Listen to joint angles.
// Store them in a shared variable.
void jointsCB(const sensor_msgs::JointStateConstPtr &msg) {
  pthread_mutex_lock(&g_joints_mutex);
  jog_arm::g_joints = *msg;
  pthread_mutex_unlock(&g_joints_mutex);
}

// Read ROS parameters, typically from YAML file
int readParameters(ros::NodeHandle &n) {
  ROS_INFO_NAMED("jog_arm_server", "---------------------------------------");
  ROS_INFO_NAMED("jog_arm_server", " Parameters:");
  ROS_INFO_NAMED("jog_arm_server", "---------------------------------------");

  // If specified in the launch file, all of the other parameters will be read
  // from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "Parameter namespace: " << parameter_ns);

  jog_arm::g_parameters.move_group_name = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/move_group_name", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "move_group_name: " << jog_arm::g_parameters.move_group_name);
  jog_arm::g_parameters.linear_scale = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/scale/linear", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "linear_scale: " << jog_arm::g_parameters.linear_scale);
  jog_arm::g_parameters.rotational_scale = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/scale/rotational", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "rotational_scale: " << jog_arm::g_parameters.rotational_scale);
  jog_arm::g_parameters.low_pass_filter_coeff = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/low_pass_filter_coeff", n);
  ROS_INFO_STREAM_NAMED(
      "jog_arm_server",
      "low_pass_filter_coeff: " << jog_arm::g_parameters.low_pass_filter_coeff);
  jog_arm::g_parameters.joint_topic = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/joint_topic", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "joint_topic: " << jog_arm::g_parameters.joint_topic);
  g_parameters.command_in_topic = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/command_in_topic", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "command_in_topic: " << g_parameters.command_in_topic);
  jog_arm::g_parameters.command_frame = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/command_frame", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "command_frame: " << jog_arm::g_parameters.command_frame);
  jog_arm::g_parameters.incoming_command_timeout = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/incoming_command_timeout", n);
  ROS_INFO_STREAM_NAMED(
      "jog_arm_server",
      "incoming_command_timeout: " << jog_arm::g_parameters.incoming_command_timeout);
  jog_arm::g_parameters.command_out_topic = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/command_out_topic", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "command_out_topic: " << jog_arm::g_parameters.command_out_topic);
  jog_arm::g_parameters.singularity_threshold = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/singularity_threshold", n);
  ROS_INFO_STREAM_NAMED(
      "jog_arm_server",
      "singularity_threshold: " << jog_arm::g_parameters.singularity_threshold);
  jog_arm::g_parameters.hard_stop_singularity_threshold = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/hard_stop_singularity_threshold", n);
  ROS_INFO_STREAM_NAMED(
      "jog_arm_server",
      "hard_stop_singularity_threshold: " << jog_arm::g_parameters.hard_stop_singularity_threshold);
  jog_arm::g_parameters.planning_frame = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/planning_frame", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "planning_frame: " << jog_arm::g_parameters.planning_frame);
  jog_arm::g_parameters.publish_period = get_ros_params::getDoubleParam(
      parameter_ns + "/jog_arm_server/publish_period", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "publish_period: " << jog_arm::g_parameters.publish_period);
  jog_arm::g_parameters.gazebo = get_ros_params::getBoolParam(
      parameter_ns + "/jog_arm_server/gazebo", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "gazebo: " << jog_arm::g_parameters.gazebo);
  jog_arm::g_parameters.collision_check = get_ros_params::getBoolParam(
      parameter_ns + "/jog_arm_server/collision_check", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "collision_check: " << jog_arm::g_parameters.collision_check);
  jog_arm::g_parameters.warning_topic = get_ros_params::getStringParam(
      parameter_ns + "/jog_arm_server/warning_topic", n);
  ROS_INFO_STREAM_NAMED("jog_arm_server",
                        "warning_topic: " << jog_arm::g_parameters.warning_topic);
  ROS_INFO_NAMED("jog_arm_server", "---------------------------------------");
  ROS_INFO_NAMED("jog_arm_server", "---------------------------------------");

  // Input checking
  if (jog_arm::g_parameters.hard_stop_singularity_threshold < jog_arm::g_parameters.singularity_threshold) {
    ROS_WARN_NAMED("jog_arm_server",
                   "Parameter 'hard_stop_singularity_threshold' "
                   "should be greater than 'singularity_threshold.'");
    return 1;
  }
  if ((jog_arm::g_parameters.hard_stop_singularity_threshold < 0.) ||
      (jog_arm::g_parameters.singularity_threshold < 0.)) {
    ROS_WARN_NAMED("jog_arm_server",
                   "Parameters 'hard_stop_singularity_threshold' "
                   "and 'singularity_threshold' should be greater than zero.");
    return 1;
  }

  return 0;
}
} // namespace jog_arm