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

/*      Title     : jog_arm.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include "moveit_jog_arm/jog_arm.h"

static const std::string LOGNAME = "jog_arm";

namespace moveit_jog_arm
{
JogArm::JogArm(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh), planning_scene_monitor_(planning_scene_monitor)
{
  // Read ROS parameters, typically from YAML file
  if (!readParameters())
    exit(EXIT_FAILURE);

  // loop period
  period_ = ros::Duration(parameters_.publish_period);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(parameters_.command_out_topic, 1);
  else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_out_topic, 1);

  // publish commands (for c++ interface)
  twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(parameters_.cartesian_command_in_topic, 1);
  joint_jog_pub_ = nh_.advertise<control_msgs::JointJog>(parameters_.joint_command_in_topic, 1);

  // Subscribe to output commands in internal namespace
  ros::NodeHandle internal_nh("~internal");
  joint_trajectory_sub_ = internal_nh.subscribe("joint_trajectory", 1, &JogArm::jointTrajectoryCB, this);

  // Wait for incoming topics to appear
  ROS_DEBUG_NAMED(LOGNAME, "Waiting for JointState topic");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);

  jog_calcs_ = std::make_unique<JogCalcs>(nh_, parameters_, shared_variables_, planning_scene_monitor_);

  collision_checker_ =
      std::make_unique<CollisionCheckThread>(nh_, parameters_, shared_variables_, planning_scene_monitor_);
}

// Read ROS parameters, typically from YAML file
bool JogArm::readParameters()
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

  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_period", parameters_.publish_period);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_check_rate", parameters_.collision_check_rate);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/num_outgoing_halt_msgs_to_publish",
                                    parameters_.num_outgoing_halt_msgs_to_publish);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/linear", parameters_.linear_scale);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/rotational", parameters_.rotational_scale);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/joint", parameters_.joint_scale);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/low_pass_filter_coeff", parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_topic", parameters_.joint_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_in_type", parameters_.command_in_type);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/cartesian_command_in_topic",
                                    parameters_.cartesian_command_in_topic);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_command_in_topic", parameters_.joint_command_in_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/robot_link_command_frame",
                                    parameters_.robot_link_command_frame);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/incoming_command_timeout",
                                    parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/lower_singularity_threshold",
                                    parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/hard_stop_singularity_threshold",
                                    parameters_.hard_stop_singularity_threshold);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/move_group_name", parameters_.move_group_name);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/planning_frame", parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/use_gazebo", parameters_.use_gazebo);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_limit_margin", parameters_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_out_topic", parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_out_type", parameters_.command_out_type);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_positions", parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_velocities",
                                    parameters_.publish_joint_velocities);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_accelerations",
                                    parameters_.publish_joint_accelerations);

  // Parameters for collision checking
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/check_collisions", parameters_.check_collisions);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_check_type", parameters_.collision_check_type);
  bool have_self_collision_proximity_threshold = rosparam_shortcuts::get(
      "", nh_, parameter_ns + "/self_collision_proximity_threshold", parameters_.self_collision_proximity_threshold);
  bool have_scene_collision_proximity_threshold = rosparam_shortcuts::get(
      "", nh_, parameter_ns + "/scene_collision_proximity_threshold", parameters_.scene_collision_proximity_threshold);
  double collision_proximity_threshold;
  // 'collision_proximity_threshold' parameter was removed, replaced with separate self- and scene-collision proximity
  // thresholds
  // TODO(JStech): remove this deprecation warning in ROS Noetic; simplify error case handling
  if (nh_.hasParam(parameter_ns + "/collision_proximity_threshold") &&
      rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_proximity_threshold", collision_proximity_threshold))
  {
    ROS_WARN_NAMED(LOGNAME, "'collision_proximity_threshold' parameter is deprecated, and has been replaced by separate"
                            "'self_collision_proximity_threshold' and 'scene_collision_proximity_threshold' "
                            "parameters. Please update the jogging yaml file.");
    if (!have_self_collision_proximity_threshold)
    {
      parameters_.self_collision_proximity_threshold = collision_proximity_threshold;
      have_self_collision_proximity_threshold = true;
    }
    if (!have_scene_collision_proximity_threshold)
    {
      parameters_.scene_collision_proximity_threshold = collision_proximity_threshold;
      have_scene_collision_proximity_threshold = true;
    }
  }
  error += !have_self_collision_proximity_threshold;
  error += !have_scene_collision_proximity_threshold;
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_distance_safety_factor",
                                    parameters_.collision_distance_safety_factor);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/min_allowable_collision_distance",
                                    parameters_.min_allowable_collision_distance);

  // This parameter name was changed recently.
  // Try retrieving from the correct name. If it fails, then try the deprecated name.
  // TODO(andyz): remove this deprecation warning in ROS Noetic
  if (!rosparam_shortcuts::get("", nh_, parameter_ns + "/status_topic", parameters_.status_topic))
  {
    ROS_WARN_NAMED(LOGNAME, "'status_topic' parameter is missing. Recently renamed from 'warning_topic'. Please update "
                            "the jogging yaml file.");
    error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/warning_topic", parameters_.status_topic);
  }

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Input checking
  if (parameters_.publish_period <= 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'publish_period' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.num_outgoing_halt_msgs_to_publish < 0)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.hard_stop_singularity_threshold < parameters_.lower_singularity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_singularity_threshold' "
                            "should be greater than 'lower_singularity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((parameters_.hard_stop_singularity_threshold < 0.) || (parameters_.lower_singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_singularity_threshold' "
                            "and 'lower_singularity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'low_pass_filter_coeff' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.joint_limit_margin < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'joint_limit_margin' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.command_in_type != "unitless" && parameters_.command_in_type != "speed_units")
  {
    ROS_WARN_NAMED(LOGNAME, "command_in_type should be 'unitless' or "
                            "'speed_units'. Check yaml file.");
    return false;
  }
  if (parameters_.command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters_.command_out_type != "std_msgs/Float64MultiArray")
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter command_out_type should be "
                            "'trajectory_msgs/JointTrajectory' or "
                            "'std_msgs/Float64MultiArray'. Check yaml file.");
    return false;
  }
  if (!parameters_.publish_joint_positions && !parameters_.publish_joint_velocities &&
      !parameters_.publish_joint_accelerations)
  {
    ROS_WARN_NAMED(LOGNAME, "At least one of publish_joint_positions / "
                            "publish_joint_velocities / "
                            "publish_joint_accelerations must be true. Check "
                            "yaml file.");
    return false;
  }
  if ((parameters_.command_out_type == "std_msgs/Float64MultiArray") && parameters_.publish_joint_positions &&
      parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(LOGNAME, "When publishing a std_msgs/Float64MultiArray, "
                            "you must select positions OR velocities.");
    return false;
  }
  // Collision checking
  if (parameters_.collision_check_type != "threshold_distance" && parameters_.collision_check_type != "stop_distance")
  {
    ROS_WARN_NAMED(LOGNAME, "collision_check_type must be 'threshold_distance' or 'stop_distance'");
    return false;
  }
  if (parameters_.self_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.scene_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'scene_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.scene_collision_proximity_threshold < parameters_.self_collision_proximity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should probably be less "
                            "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (parameters_.collision_check_rate < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'collision_check_rate' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.collision_distance_safety_factor < 1)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'collision_distance_safety_factor' should be "
                            "greater than or equal to 1. Check yaml file.");
    return false;
  }
  if (parameters_.min_allowable_collision_distance < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'min_allowable_collision_distance' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }

  return true;
}

void JogArm::run(const ros::TimerEvent& timer_event)
{
  // Log last loop duration and warn if it was longer than period
  if (timer_event.profile.last_duration.toSec() < period_.toSec())
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(10, LOGNAME, "last_duration: " << timer_event.profile.last_duration.toSec() << " ("
                                                                   << period_.toSec() << ")");
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(1, LOGNAME, "last_duration: " << timer_event.profile.last_duration.toSec() << " > "
                                                                 << period_.toSec());
  }

  // Get the latest joint trajectory
  trajectory_msgs::JointTrajectory joint_trajectory;
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    if (latest_joint_trajectory_)
    {
      joint_trajectory = *latest_joint_trajectory_;
    }
  }

  // Publish the most recent trajectory, unless the jogging calculation thread tells not to
  if (shared_variables_.ok_to_publish)
  {
    // Put the outgoing msg in the right format
    // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    {
      joint_trajectory.header.stamp = ros::Time::now();
      outgoing_cmd_pub_.publish(joint_trajectory);
    }
    else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    {
      std_msgs::Float64MultiArray joints;
      if (parameters_.publish_joint_positions)
        joints.data = joint_trajectory.points[0].positions;
      else if (parameters_.publish_joint_velocities)
        joints.data = joint_trajectory.points[0].velocities;
      outgoing_cmd_pub_.publish(joints);
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
}

void JogArm::start()
{
  shared_variables_.paused = false;

  // Crunch the numbers in this thread
  jog_calcs_->start();

  // Check collisions in this thread
  if (parameters_.check_collisions)
    collision_checker_->start();

  // Start the jog server timer
  timer_ = nh_.createTimer(period_, &JogArm::run, this);
}

void JogArm::stop()
{
  timer_.stop();
  jog_calcs_->stop();
  collision_checker_->stop();
}

JogArm::~JogArm()
{
  stop();
}

void JogArm::provideTwistStampedCommand(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_stamped_pub_.publish(msg);
}

void JogArm::provideJointCommand(const control_msgs::JointJogConstPtr& msg)
{
  joint_jog_pub_.publish(msg);
}

void JogArm::setPaused(bool paused)
{
  shared_variables_.paused = paused;
}

bool JogArm::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  if (!jog_calcs_->isInitialized())
    return false;
  return jog_calcs_->getCommandFrameTransform(transform);
}

StatusCode JogArm::getJoggerStatus() const
{
  return shared_variables_.status;
}

void JogArm::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryPtr& msg)
{
  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_joint_trajectory_ = msg;
}

}  // namespace moveit_jog_arm
