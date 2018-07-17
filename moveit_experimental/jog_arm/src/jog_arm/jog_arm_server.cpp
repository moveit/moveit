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
#include <memory>

// Initialize these static struct to hold ROS parameters
jog_arm::jog_arm_parameters jog_arm::jogROSInterface::ros_parameters_;
jog_arm::jog_arm_shared jog_arm::jogROSInterface::shared_variables_;

/////////////////////////////////////////////////////////////////////////////////
// jogROSInterface handles ROS subscriptions and instantiates the worker threads.
// One worker thread does the jogging calculations.
// Another worker thread does collision checking.
/////////////////////////////////////////////////////////////////////////////////

static const char* const NODE_NAME = "jog_arm_server";

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);

  jog_arm::jogROSInterface ros_interface;

  return 0;
}

namespace jog_arm
{
// Constructor for the main ROS interface node
jogROSInterface::jogROSInterface()
{
  ros::NodeHandle n;

  // Read ROS parameters, typically from YAML file
  readParameters(n);

  // Crunch the numbers in this thread
  pthread_t joggingThread;
  int rc = pthread_create(&joggingThread, nullptr, jog_arm::jogROSInterface::joggingPipeline, this);
  if (rc)
  {
    ROS_FATAL_NAMED(NODE_NAME, "Creating pipeline thread failed", rc);
    return;
  }

  // Check collisions in this thread
  pthread_t collisionThread;
  rc = pthread_create(&collisionThread, nullptr, jog_arm::jogROSInterface::collisionCheck, this);
  if (rc)
  {
    ROS_FATAL_NAMED(NODE_NAME, "Creating collision check failed", rc);
    return;
  }

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub = n.subscribe(ros_parameters_.command_in_topic, 1, &jogROSInterface::deltaCmdCB, this);
  ros::Subscriber joints_sub = n.subscribe(ros_parameters_.joint_topic, 1, &jogROSInterface::jointsCB, this);

  ros::topic::waitForMessage<sensor_msgs::JointState>(ros_parameters_.joint_topic);
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(ros_parameters_.command_in_topic);

  // Wait for jog filters to stabilize
  ros::Duration(10 * ros_parameters_.publish_period).sleep();

  ros::spin();

  (void)pthread_join(joggingThread, nullptr);
  (void)pthread_join(collisionThread, nullptr);
}

// A separate thread for the heavy jogging calculations.
void* jogROSInterface::joggingPipeline(void*)
{
  jog_arm::JogCalcs ja(ros_parameters_, shared_variables_);
  return nullptr;
}

// A separate thread for collision checking.
void* jogROSInterface::collisionCheck(void*)
{
  jog_arm::CollisionCheck cc(ros_parameters_, shared_variables_);
  return nullptr;
}

// Constructor for the class that handles collision checking
CollisionCheck::CollisionCheck(const jog_arm_parameters& parameters, jog_arm_shared& shared_variables)
{
  // If user specified true in yaml file
  if (parameters.collision_check)
  {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = parameters.move_group_name;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Wait for initial messages
    ROS_INFO_NAMED(NODE_NAME, "Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(parameters.joint_topic);
    ROS_INFO_NAMED(NODE_NAME, "Received first joint msg.");

    ROS_INFO_NAMED(NODE_NAME, "Waiting for first command msg.");
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>(parameters.command_in_topic);
    ROS_INFO_NAMED(NODE_NAME, "Received first command msg.");

    ros::Rate collision_rate(100);

    /////////////////////////////////////////////////
    // Spin while checking collisions
    /////////////////////////////////////////////////
    while (ros::ok())
    {
      pthread_mutex_lock(&shared_variables.joints_mutex);
      sensor_msgs::JointState jts = shared_variables.joints;
      pthread_mutex_unlock(&shared_variables.joints_mutex);

      for (std::size_t i = 0; i < jts.position.size(); ++i)
        current_state.setJointPositions(jts.name[i], &jts.position[i]);

      // process collision objects in scene
      std::map<std::string, moveit_msgs::CollisionObject> c_objects_map = planning_scene_interface.getObjects();
      for (auto& kv : c_objects_map)
      {
        planning_scene.processCollisionObjectMsg(kv.second);
      }

      collision_result.clear();
      planning_scene.checkCollision(collision_request, collision_result);

      // If collision, signal the jogging to stop
      pthread_mutex_lock(&shared_variables.imminent_collision_mutex);
      shared_variables.imminent_collision = collision_result.collision;
      pthread_mutex_unlock(&shared_variables.imminent_collision_mutex);

      ros::spinOnce();
      collision_rate.sleep();
    }
  }
}

// Constructor for the class that handles jogging calculations
JogCalcs::JogCalcs(const jog_arm_parameters& parameters, jog_arm_shared& shared_variables)
  : move_group_(parameters.move_group_name)
{
  parameters_ = parameters;

  // Publish collision status
  warning_pub_ = nh_.advertise<std_msgs::Bool>(parameters_.warning_topic, 1);

  // Publish freshly-calculated joints to the robot
  joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(parameters.command_out_topic, 1);

  // MoveIt Setup
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  const robot_model::RobotModelPtr& kinematic_model = model_loader.getModel();

  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(parameters_.move_group_name);

  const std::vector<std::string>& joint_names = joint_model_group_->getJointModelNames();
  std::vector<double> dummy_joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, dummy_joint_values);

  // Wait for initial messages
  ROS_INFO_NAMED(NODE_NAME, "Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);
  ROS_INFO_NAMED(NODE_NAME, "Received first joint msg.");

  ROS_INFO_NAMED(NODE_NAME, "Waiting for first command msg.");
  ros::topic::waitForMessage<geometry_msgs::TwistStamped>(parameters_.command_in_topic);
  ROS_INFO_NAMED(NODE_NAME, "Received first command msg.");
  ;

  jt_state_.name = move_group_.getJointNames();
  jt_state_.position.resize(jt_state_.name.size());
  jt_state_.velocity.resize(jt_state_.name.size());
  jt_state_.effort.resize(jt_state_.name.size());

  // Low-pass filters for the joint positions & velocities
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    velocity_filters_.emplace_back(parameters_.low_pass_filter_coeff);
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // Initialize the position filters to initial robot joints
  pthread_mutex_lock(&shared_variables.joints_mutex);
  incoming_jts_ = shared_variables.joints;
  pthread_mutex_unlock(&shared_variables.joints_mutex);
  updateJoints();
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
    position_filters_[i].reset(jt_state_.position[i]);

  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  while (cmd_deltas_.header.stamp == ros::Time(0.))
  {
    ros::Duration(0.05).sleep();
    pthread_mutex_lock(&shared_variables.command_deltas_mutex);
    cmd_deltas_ = shared_variables.command_deltas;
    pthread_mutex_unlock(&shared_variables.command_deltas_mutex);
  }

  // Now do jogging calcs
  ros::Rate main_rate(1. / parameters_.publish_period);
  bool last_was_zero_traj = false;
  while (ros::ok())
  {
    // If user commands are all zero, reset the low-pass filters
    // when commands resume
    pthread_mutex_lock(&shared_variables.zero_trajectory_flag_mutex);
    bool zero_traj_flag = shared_variables.zero_trajectory_flag;
    pthread_mutex_unlock(&shared_variables.zero_trajectory_flag_mutex);
    if (zero_traj_flag)
      // Reset low-pass filters
      resetVelocityFilters();

    // Pull data from the shared variables.
    pthread_mutex_lock(&shared_variables.command_deltas_mutex);
    cmd_deltas_ = shared_variables.command_deltas;
    pthread_mutex_unlock(&shared_variables.command_deltas_mutex);

    pthread_mutex_lock(&shared_variables.joints_mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.joints_mutex);

    updateJoints();

    if (!zero_traj_flag)
      jogCalcs(cmd_deltas_, shared_variables);

    // Send the newest target joints
    if (!new_traj_.joint_names.empty())
    {
      // Check for stale cmds
      if (ros::Time::now() - new_traj_.header.stamp < ros::Duration(parameters.incoming_command_timeout))
      {
        // Skip the jogging publication if all inputs are 0.
        if (!zero_traj_flag)
        {
          joint_trajectory_pub_.publish(new_traj_);
        }
      }
      else if (!last_was_zero_traj)
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, "Stale joint "
                                                     "trajectory msg. Try a larger "
                                                     "'incoming_command_timeout' parameter.");
        ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, "Did input from the "
                                                     "controller get interrupted? Are "
                                                     "calculations taking too long?");
      }

      // Store last traj message flag to prevent superflous warnings
      last_was_zero_traj = zero_traj_flag;
    }

    main_rate.sleep();
  }
}

// Perform the jogging calculations
void JogCalcs::jogCalcs(const geometry_msgs::TwistStamped& cmd, jog_arm_shared& shared_variables)
{
  // Convert the cmd to the MoveGroup planning frame.
  try
  {
    listener_.waitForTransform(cmd.header.frame_id, parameters_.planning_frame, ros::Time::now(), ros::Duration(0.2));
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
    return;
  }
  // To transform, these vectors need to be stamped. See answers.ros.org
  // Q#199376
  // Transform the linear component of the cmd message
  geometry_msgs::Vector3Stamped lin_vector;
  lin_vector.vector = cmd.twist.linear;
  lin_vector.header.frame_id = cmd.header.frame_id;
  try
  {
    listener_.transformVector(parameters_.planning_frame, lin_vector, lin_vector);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
    return;
  }

  geometry_msgs::Vector3Stamped rot_vector;
  rot_vector.vector = cmd.twist.angular;
  rot_vector.header.frame_id = cmd.header.frame_id;
  try
  {
    listener_.transformVector(parameters_.planning_frame, rot_vector, rot_vector);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << ": " << ex.what());
    return;
  }

  // Put these components back into a TwistStamped
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.header.stamp = cmd.header.stamp;
  twist_cmd.header.frame_id = parameters_.planning_frame;
  twist_cmd.twist.linear = lin_vector.vector;
  twist_cmd.twist.angular = rot_vector.vector;

  // Apply user-defined scaling
  const Eigen::VectorXd delta_x = scaleCommand(twist_cmd);

  kinematic_state_->setVariableValues(jt_state_);
  orig_jts_ = jt_state_;

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd old_jacobian = kinematic_state_->getJacobian(joint_model_group_);
  Eigen::VectorXd delta_theta = pseudoInverse(old_jacobian) * delta_x;

  if (!addJointIncrements(jt_state_, delta_theta))
    return;

  // Include a velocity estimate for velocity-controller robots
  Eigen::VectorXd joint_vel(delta_theta / parameters_.publish_period);

  // Low-pass filter the velocities
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_.velocity[i] = velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

    // Check for nan's
    if (std::isnan(joint_vel[static_cast<long>(i)]))
    {
      jt_state_.position[i] = orig_jts_.position[i];
      jt_state_.velocity[i] = 0.;
    }
  }

  // Low-pass filter the positions
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_.position[i] = position_filters_[i].filter(jt_state_.position[i]);

    // Check for nan's
    if (std::isnan(jt_state_.position[i]))
    {
      jt_state_.position[i] = orig_jts_.position[i];
      jt_state_.velocity[i] = 0.;
    }
  }

  // Check the Jacobian with these new joints.
  kinematic_state_->setVariableValues(jt_state_);
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

  const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_period);
  new_traj_ = composeOutgoingMessage(jt_state_, next_time);

  // apply several checks if new joint state is valid
  if (!checkIfImminentCollision(shared_variables, new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }

  else if (!verifyJacobianIsWellConditioned(old_jacobian, delta_theta, jacobian, new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }

  else if (!checkIfJointsWithinBounds(new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }

  else
  {
    publishWarning(false);
  }

  // done with calculations
  if (parameters_.gazebo)
  {
    // Spam several redundant points into the trajectory. The first few may be
    // skipped if the
    // time stamp is in the past when it reaches the client. Needed for gazebo
    // simulation.
    // Start from 2 because the first point's timestamp is already
    // 1*parameters_.publish_period
    auto point = new_traj_.points[0];
    for (int i = 2; i < 30; ++i)
    {
      point.time_from_start = ros::Duration(i * parameters_.publish_period);
      new_traj_.points.push_back(point);
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
  point.positions = joint_state.position;
  point.time_from_start = ros::Duration(parameters_.publish_period);
  point.velocities = joint_state.velocity;
  new_jt_traj.points.push_back(point);

  return new_jt_traj;
}

bool JogCalcs::checkIfImminentCollision(jog_arm_shared& shared_variables, trajectory_msgs::JointTrajectory& new_jt_traj)
{
  pthread_mutex_lock(&shared_variables.imminent_collision_mutex);
  bool collision = shared_variables.imminent_collision;
  pthread_mutex_unlock(&shared_variables.imminent_collision_mutex);
  if (collision)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " Close to a collision. "
                                                                              "Halting.");
    return false;
  }
  return true;
}

bool JogCalcs::verifyJacobianIsWellConditioned(const Eigen::MatrixXd& old_jacobian, const Eigen::VectorXd& delta_theta,
                                               const Eigen::MatrixXd& new_jacobian,
                                               trajectory_msgs::JointTrajectory& new_jt_traj)
{
  // Ramp velocity down linearly when the Jacobian condition is between singularity_threshold and
  // hard_stop_singurality_threshold
  double current_condition_number = checkConditionNumber(new_jacobian);
  if ((current_condition_number > parameters_.singularity_threshold) &&
      (current_condition_number < parameters_.hard_stop_singularity_threshold))
  {
    double velocity_scale = 1. -
                            (current_condition_number - parameters_.singularity_threshold) /
                                (parameters_.hard_stop_singularity_threshold - parameters_.singularity_threshold);
    for (size_t i = 0; i < jt_state_.velocity.size(); ++i)
    {
      new_jt_traj.points[0].positions[i] =
          new_jt_traj.points[0].positions[i] - velocity_scale * delta_theta[static_cast<long>(i)];
      new_jt_traj.points[0].velocities[i] *= velocity_scale;
    }
  }

  // Very close to singularity, so halt.
  else
  {
    double old_condition_number = checkConditionNumber(old_jacobian);
    if ((current_condition_number > parameters_.singularity_threshold) &&
        (current_condition_number > old_condition_number))
    {
      if (current_condition_number > parameters_.hard_stop_singularity_threshold)
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(1, NODE_NAME, ros::this_node::getName() << " Close to a "
                                                                                  "singularity ("
                                                                               << current_condition_number
                                                                               << "). Halting.");

        return false;
      }
    }
  }

  return true;
}

bool JogCalcs::checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory& new_jt_traj)
{
  bool halting = false;
  for (auto joint : joint_model_group_->getJointModels())
  {
    if (!kinematic_state_->satisfiesVelocityBounds(joint))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " " << joint->getName() << " "
                                                                             << joint->getFirstVariableIndex()
                                                                             << " close to a "
                                                                                " velocity limit. Enforcing limit.");
      kinematic_state_->enforceVelocityBounds(joint);
      new_jt_traj.points[0].velocities[joint->getFirstVariableIndex()] = kinematic_state_->getJointVelocities(joint)[0];
    }
    if (!kinematic_state_->satisfiesPositionBounds(joint))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " " << joint->getName()
                                                                             << " close to a "
                                                                                " position limit. Halting.");
      halting = true;
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

// Halt the robot
void JogCalcs::halt(trajectory_msgs::JointTrajectory& jt_traj)
{
  for (std::size_t i = 0; i < jt_state_.velocity.size(); ++i)
  {
    jt_traj.points[0].positions[i] = orig_jts_.position[i];
    jt_traj.points[0].velocities[i] = 0.;
  }
  // Store all zeros in the velocity filter
  resetVelocityFilters();
}

// Reset the data stored in filters so the trajectory won't jump when jogging is
// resumed.
void JogCalcs::resetVelocityFilters()
{
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
    velocity_filters_[i].reset(0);  // Zero velocity
}

// Parse the incoming joint msg for the joints of our MoveGroup
void JogCalcs::updateJoints()
{
  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < jt_state_.name.size())
  {
    ROS_WARN_NAMED("JogCalcs", "The joint msg does not contain enough "
                               "joints.");
    return;
  }

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_jts_.name.size(); ++m)
  {
    for (std::size_t c = 0; c < jt_state_.name.size(); ++c)
    {
      if (incoming_jts_.name[m] == jt_state_.name[c])
      {
        jt_state_.position[c] = incoming_jts_.position[m];
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }
}

// Scale the incoming jog command
Eigen::VectorXd JogCalcs::scaleCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  result[0] = parameters_.linear_scale * command.twist.linear.x;
  result[1] = parameters_.linear_scale * command.twist.linear.y;
  result[2] = parameters_.linear_scale * command.twist.linear.z;
  result[3] = parameters_.rotational_scale * command.twist.angular.x;
  result[4] = parameters_.rotational_scale * command.twist.angular.y;
  result[5] = parameters_.rotational_scale * command.twist.angular.z;

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
      ROS_ERROR_STREAM_NAMED(NODE_NAME, ros::this_node::getName() << " Lengths of output and "
                                                                     "increments do not match.");
      return false;
    }
  }

  return true;
}

// Calculate the condition number of the jacobian, to check for singularities
double JogCalcs::checkConditionNumber(const Eigen::MatrixXd& matrix) const
{
  // For 6-DOF arms, use eigenvalues. It's faster than singular value
  // decomposition
  if (jt_state_.name.size() == 6)
  {
    Eigen::MatrixXd::EigenvaluesReturnType eigs = matrix.eigenvalues();
    Eigen::VectorXd eig_vector = eigs.cwiseAbs();

    // condition = max(eigs)/min(eigs)
    return eig_vector.maxCoeff() / eig_vector.minCoeff();
  }
  else
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  }
}

// Listen to cartesian delta commands.
// Store them in a shared variable.
void jogROSInterface::deltaCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.command_deltas_mutex);
  shared_variables_.command_deltas = *msg;
  // Input frame determined by YAML file:
  shared_variables_.command_deltas.header.frame_id = ros_parameters_.command_frame;
  pthread_mutex_unlock(&shared_variables_.command_deltas_mutex);

  // Check if input is all zeros. Flag it if so to skip calculations/publication
  pthread_mutex_lock(&shared_variables_.zero_trajectory_flag_mutex);
  shared_variables_.zero_trajectory_flag = shared_variables_.command_deltas.twist.linear.x == 0.0 &&
                                           shared_variables_.command_deltas.twist.linear.y == 0.0 &&
                                           shared_variables_.command_deltas.twist.linear.z == 0.0 &&
                                           shared_variables_.command_deltas.twist.angular.x == 0.0 &&
                                           shared_variables_.command_deltas.twist.angular.y == 0.0 &&
                                           shared_variables_.command_deltas.twist.angular.z == 0.0;
  pthread_mutex_unlock(&shared_variables_.zero_trajectory_flag_mutex);
}

// Listen to joint angles.
// Store them in a shared variable.
void jogROSInterface::jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_.joints_mutex);
  shared_variables_.joints = *msg;
  pthread_mutex_unlock(&shared_variables_.joints_mutex);
}

// Read ROS parameters, typically from YAML file
int jogROSInterface::readParameters(ros::NodeHandle& n)
{
  // If specified in the launch file, all other parameters will be read
  // from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);

  std::size_t error = 0;
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/move_group_name",
                                    ros_parameters_.move_group_name);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/publish_period", ros_parameters_.publish_period);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/scale/linear", ros_parameters_.linear_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/scale/rotational",
                                    ros_parameters_.rotational_scale);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/low_pass_filter_coeff",
                                    ros_parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/joint_topic", ros_parameters_.joint_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/command_in_topic",
                                    ros_parameters_.command_in_topic);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/command_frame", ros_parameters_.command_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/incoming_command_timeout",
                                    ros_parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/command_out_topic",
                                    ros_parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/singularity_threshold",
                                    ros_parameters_.singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/hard_stop_singularity_threshold",
                                    ros_parameters_.hard_stop_singularity_threshold);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/planning_frame", ros_parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/gazebo", ros_parameters_.gazebo);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/collision_check",
                                    ros_parameters_.collision_check);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/jog_arm_server/warning_topic", ros_parameters_.warning_topic);

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Input checking
  if (ros_parameters_.hard_stop_singularity_threshold < ros_parameters_.singularity_threshold)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'hard_stop_singularity_threshold' "
                              "should be greater than 'singularity_threshold.'");
    return 1;
  }
  if ((ros_parameters_.hard_stop_singularity_threshold < 0.) || (ros_parameters_.singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameters 'hard_stop_singularity_threshold' "
                              "and 'singularity_threshold' should be greater than zero.");
    return 1;
  }
  if (ros_parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(NODE_NAME, "Parameter 'low_pass_filter_coeff' should be greater than zero.");
    return 1;
  }

  return 0;
}
}  // namespace jog_arm
