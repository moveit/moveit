/*
     Title     : jog_ros_interface.cpp
     Project   : moveit_jog_arm
     Created   : 3/9/2017
     Author    : Brian O'Neil, Andy Zelenak, Blake Anderson

BSD 3-Clause License

Copyright (c) 2018, Los Alamos National Security, LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Server node for arm jogging with MoveIt.

#include <moveit_jog_arm/jog_ros_interface.h>

namespace moveit_jog_arm
{
/////////////////////////////////////////////////////////////////////////////////
// JogROSInterface handles ROS subscriptions and instantiates the worker threads.
// One worker thread does the jogging calculations.
// Another worker thread does collision checking.
/////////////////////////////////////////////////////////////////////////////////

// Constructor for the main ROS interface node
JogROSInterface::JogROSInterface()
{
  ros::NodeHandle nh;

  pthread_mutex_init(&shared_variables_mutex_, nullptr);

  // Read ROS parameters, typically from YAML file
  if (!readParameters(nh))
    exit(EXIT_FAILURE);

  // Load the robot model. This is used by the worker threads.
  model_loader_ptr_ = std::shared_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);

  // Crunch the numbers in this thread
  std::thread jogging_thread(&JogROSInterface::startJogCalcThread, this);

  // Check collisions in this thread
  std::thread collision_thread(&JogROSInterface::startCollisionCheckThread, this);

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub =
      nh.subscribe(ros_parameters_.cartesian_command_in_topic, 1, &JogROSInterface::deltaCartesianCmdCB, this);
  ros::Subscriber joints_sub = nh.subscribe(ros_parameters_.joint_topic, 1, &JogROSInterface::jointsCB, this);
  ros::Subscriber joint_jog_cmd_sub =
      nh.subscribe(ros_parameters_.joint_command_in_topic, 1, &JogROSInterface::deltaJointCmdCB, this);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  ros::Publisher outgoing_cmd_pub;
  if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>(ros_parameters_.command_out_topic, 1);
  else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>(ros_parameters_.command_out_topic, 1);

  // Wait for incoming topics to appear
  ROS_DEBUG_NAMED(LOGNAME, "Waiting for JointState topic");
  ros::topic::waitForMessage<sensor_msgs::JointState>(ros_parameters_.joint_topic);

  // Wait for low pass filters to stabilize
  ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for low-pass filters to stabilize.");
  ros::Duration(10 * ros_parameters_.publish_period).sleep();

  ros::Rate main_rate(1. / ros_parameters_.publish_period);

  while (ros::ok())
  {
    ros::spinOnce();

    pthread_mutex_lock(&shared_variables_mutex_);
    trajectory_msgs::JointTrajectory outgoing_command = shared_variables_.outgoing_command;

    // Check for stale cmds
    if ((ros::Time::now() - shared_variables_.latest_nonzero_cmd_stamp) <
        ros::Duration(ros_parameters_.incoming_command_timeout))
    {
      // Mark that incoming commands are not stale
      shared_variables_.command_is_stale = false;
    }
    else
    {
      shared_variables_.command_is_stale = true;
    }

    // Publish the most recent trajectory, unless the jogging calculation thread tells not to
    if (shared_variables_.ok_to_publish)
    {
      // Put the outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
      if (ros_parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
      {
        outgoing_command.header.stamp = ros::Time::now();
        outgoing_cmd_pub.publish(outgoing_command);
      }
      else if (ros_parameters_.command_out_type == "std_msgs/Float64MultiArray")
      {
        std_msgs::Float64MultiArray joints;
        if (ros_parameters_.publish_joint_positions)
          joints.data = outgoing_command.points[0].positions;
        else if (ros_parameters_.publish_joint_velocities)
          joints.data = outgoing_command.points[0].velocities;
        outgoing_cmd_pub.publish(joints);
      }
    }
    else if (shared_variables_.command_is_stale)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(10, LOGNAME, "Stale command. "
                                                  "Try a larger 'incoming_command_timeout' parameter?");
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE_NAMED(10, LOGNAME, "All-zero command. Doing nothing.");
    }

    pthread_mutex_unlock(&shared_variables_mutex_);

    main_rate.sleep();
  }

  jogging_thread.join();
  collision_thread.join();
}

JogROSInterface::~JogROSInterface()
{
  pthread_mutex_destroy(&shared_variables_mutex_);
}

// A separate thread for the heavy jogging calculations.
bool JogROSInterface::startJogCalcThread()
{
  JogCalcs ja(ros_parameters_, shared_variables_, shared_variables_mutex_, model_loader_ptr_);
  return true;
}

// A separate thread for collision checking.
bool JogROSInterface::startCollisionCheckThread()
{
  CollisionCheckThread cc(ros_parameters_, shared_variables_, shared_variables_mutex_, model_loader_ptr_);
  return true;
}

// Listen to cartesian delta commands. Store them in a shared variable.
void JogROSInterface::deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_mutex_);

  // Copy everything but the frame name. The frame name is set by yaml file at startup.
  // (so it isn't copied over and over)
  shared_variables_.command_deltas.twist = msg->twist;
  shared_variables_.command_deltas.header = msg->header;

  // Input frame determined by YAML file if not passed with message
  if (shared_variables_.command_deltas.header.frame_id.empty())
  {
    shared_variables_.command_deltas.header.frame_id = ros_parameters_.command_frame;
  }

  // Check if input is all zeros. Flag it if so to skip calculations/publication after num_halt_msgs_to_publish
  shared_variables_.zero_cartesian_cmd_flag = shared_variables_.command_deltas.twist.linear.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.linear.z == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.x == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.y == 0.0 &&
                                              shared_variables_.command_deltas.twist.angular.z == 0.0;

  if (!shared_variables_.zero_cartesian_cmd_flag)
  {
    shared_variables_.latest_nonzero_cmd_stamp = msg->header.stamp;
  }
  pthread_mutex_unlock(&shared_variables_mutex_);
}

// Listen to joint delta commands. Store them in a shared variable.
void JogROSInterface::deltaJointCmdCB(const control_msgs::JointJogConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_mutex_);
  shared_variables_.joint_command_deltas = *msg;

  // Check if joint inputs is all zeros. Flag it if so to skip calculations/publication
  bool all_zeros = true;
  for (double delta : shared_variables_.joint_command_deltas.velocities)
  {
    all_zeros &= (delta == 0.0);
  };
  shared_variables_.zero_joint_cmd_flag = all_zeros;

  if (!shared_variables_.zero_joint_cmd_flag)
  {
    shared_variables_.latest_nonzero_cmd_stamp = msg->header.stamp;
  }
  pthread_mutex_unlock(&shared_variables_mutex_);
}

// Listen to joint angles. Store them in a shared variable.
void JogROSInterface::jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  pthread_mutex_lock(&shared_variables_mutex_);
  shared_variables_.joints = *msg;
  pthread_mutex_unlock(&shared_variables_mutex_);
}

// Read ROS parameters, typically from YAML file
bool JogROSInterface::readParameters(ros::NodeHandle& n)
{
  std::size_t error = 0;

  // Specified in the launch file. All other parameters will be read from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);
  if (parameter_ns.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "A namespace must be specified in the launch file, like:");
    ROS_ERROR_STREAM_NAMED(LOGNAME, "<param name=\"parameter_ns\" "
                                    "type=\"string\" "
                                    "value=\"left_jog_server\" />");
    return false;
  }

  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_period", ros_parameters_.publish_period);
  error +=
      !rosparam_shortcuts::get("", n, parameter_ns + "/collision_check_rate", ros_parameters_.collision_check_rate);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/num_halt_msgs_to_publish",
                                    ros_parameters_.num_halt_msgs_to_publish);
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
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/collision_proximity_threshold",
                                    ros_parameters_.collision_proximity_threshold);
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

  // Input checking
  if (ros_parameters_.num_halt_msgs_to_publish < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'num_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return false;
  }
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
  if (ros_parameters_.collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'collision_proximity_threshold' should be "
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
}  // namespace moveit_jog_arm
