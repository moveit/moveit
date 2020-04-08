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

/*      Title     : jog_ros_interface.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

// Server node for arm jogging with MoveIt.

#include <moveit_jog_arm/jog_ros_interface.h>

static const std::string LOGNAME = "jog_ros_interface";

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

  // Read ROS parameters, typically from YAML file
  if (!readParameters(nh))
    exit(EXIT_FAILURE);

  // Load the planning scene monitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor_->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor_->startStateMonitor();

  // Crunch the numbers in this thread
  startJogCalcThread();

  // Check collisions in this thread
  if (ros_parameters_.check_collisions)
    startCollisionCheckThread();

  // ROS subscriptions. Share the data with the worker threads
  ros::Subscriber cmd_sub =
      nh.subscribe(ros_parameters_.cartesian_command_in_topic, 1, &JogROSInterface::deltaCartesianCmdCB, this);
  ros::Subscriber joint_jog_cmd_sub =
      nh.subscribe(ros_parameters_.joint_command_in_topic, 1, &JogROSInterface::deltaJointCmdCB, this);
  ros::Subscriber joints_sub =
      nh.subscribe(ros_parameters_.joint_topic, 1, &JogInterfaceBase::jointsCB, dynamic_cast<JogInterfaceBase*>(this));

  // ROS Server for allowing drift in some dimensions
  ros::ServiceServer drift_dimensions_server =
      nh.advertiseService(nh.getNamespace() + "/" + ros::this_node::getName() + "/change_drift_dimensions",
                          &JogInterfaceBase::changeDriftDimensions, dynamic_cast<JogInterfaceBase*>(this));

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

    shared_variables_.lock();
    trajectory_msgs::JointTrajectory outgoing_command = shared_variables_.outgoing_command;

    // Check for stale cmds
    shared_variables_.command_is_stale = ((ros::Time::now() - shared_variables_.latest_nonzero_cmd_stamp) >=
                                          ros::Duration(ros_parameters_.incoming_command_timeout));

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

    shared_variables_.unlock();

    main_rate.sleep();
  }

  // Stop JogArm threads
  shared_variables_.stop_requested = true;
  stopJogCalcThread();
  stopCollisionCheckThread();
}

// Listen to cartesian delta commands. Store them in a shared variable.
void JogROSInterface::deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  shared_variables_.lock();

  // Copy everything but the frame name. The frame name is set by yaml file at startup.
  // (so it isn't copied over and over)
  shared_variables_.command_deltas.twist = msg->twist;
  shared_variables_.command_deltas.header = msg->header;

  // Input frame determined by YAML file if not passed with message
  if (shared_variables_.command_deltas.header.frame_id.empty())
  {
    shared_variables_.command_deltas.header.frame_id = ros_parameters_.robot_link_command_frame;
  }

  // Check if input is all zeros. Flag it if so to skip calculations/publication after num_outgoing_halt_msgs_to_publish
  shared_variables_.have_nonzero_cartesian_cmd = shared_variables_.command_deltas.twist.linear.x != 0.0 ||
                                                 shared_variables_.command_deltas.twist.linear.y != 0.0 ||
                                                 shared_variables_.command_deltas.twist.linear.z != 0.0 ||
                                                 shared_variables_.command_deltas.twist.angular.x != 0.0 ||
                                                 shared_variables_.command_deltas.twist.angular.y != 0.0 ||
                                                 shared_variables_.command_deltas.twist.angular.z != 0.0;

  if (shared_variables_.have_nonzero_cartesian_cmd)
  {
    shared_variables_.latest_nonzero_cmd_stamp = msg->header.stamp;
  }
  shared_variables_.unlock();
}

// Listen to joint delta commands. Store them in a shared variable.
void JogROSInterface::deltaJointCmdCB(const control_msgs::JointJogConstPtr& msg)
{
  shared_variables_.lock();
  shared_variables_.joint_command_deltas = *msg;

  // Check if joint inputs is all zeros. Flag it if so to skip calculations/publication
  bool all_zeros = true;
  for (double delta : shared_variables_.joint_command_deltas.velocities)
  {
    all_zeros &= (delta == 0.0);
  };
  shared_variables_.have_nonzero_joint_cmd = !all_zeros;

  if (shared_variables_.have_nonzero_joint_cmd)
  {
    shared_variables_.latest_nonzero_cmd_stamp = msg->header.stamp;
  }
  shared_variables_.unlock();
}
}  // namespace moveit_jog_arm
