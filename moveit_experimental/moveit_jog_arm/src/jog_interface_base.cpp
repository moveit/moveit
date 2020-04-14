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

/*      Title     : jog_interface_base.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include "moveit_jog_arm/jog_interface_base.h"

static const std::string LOGNAME = "jog_interface_base";

namespace moveit_jog_arm
{
JogInterfaceBase::JogInterfaceBase()
{
}

// Read ROS parameters, typically from YAML file
bool JogInterfaceBase::readParameters(ros::NodeHandle& n)
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
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/num_outgoing_halt_msgs_to_publish",
                                    ros_parameters_.num_outgoing_halt_msgs_to_publish);
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
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/robot_link_command_frame",
                                    ros_parameters_.robot_link_command_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/incoming_command_timeout",
                                    ros_parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/lower_singularity_threshold",
                                    ros_parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/hard_stop_singularity_threshold",
                                    ros_parameters_.hard_stop_singularity_threshold);
  // parameter was removed, replaced with separate self- and scene-collision proximity thresholds; the logic handling
  // the different possible sets of defined parameters is somewhat complicated at this point
  // TODO(JStech): remove this deprecation warning in ROS Noetic; simplify error case handling
  bool have_self_collision_proximity_threshold = rosparam_shortcuts::get(
      "", n, parameter_ns + "/self_collision_proximity_threshold", ros_parameters_.self_collision_proximity_threshold);
  bool have_scene_collision_proximity_threshold =
      rosparam_shortcuts::get("", n, parameter_ns + "/scene_collision_proximity_threshold",
                              ros_parameters_.scene_collision_proximity_threshold);
  double collision_proximity_threshold;
  if (n.hasParam(parameter_ns + "/collision_proximity_threshold") &&
      rosparam_shortcuts::get("", n, parameter_ns + "/collision_proximity_threshold", collision_proximity_threshold))
  {
    ROS_WARN_NAMED(LOGNAME, "'collision_proximity_threshold' parameter is deprecated, and has been replaced by separate"
                            "'self_collision_proximity_threshold' and 'scene_collision_proximity_threshold' "
                            "parameters. Please update the jogging yaml file.");
    if (!have_self_collision_proximity_threshold)
    {
      ros_parameters_.self_collision_proximity_threshold = collision_proximity_threshold;
      have_self_collision_proximity_threshold = true;
    }
    if (!have_scene_collision_proximity_threshold)
    {
      ros_parameters_.scene_collision_proximity_threshold = collision_proximity_threshold;
      have_scene_collision_proximity_threshold = true;
    }
  }
  error += !have_self_collision_proximity_threshold;
  error += !have_scene_collision_proximity_threshold;
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/move_group_name", ros_parameters_.move_group_name);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/planning_frame", ros_parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/use_gazebo", ros_parameters_.use_gazebo);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/check_collisions", ros_parameters_.check_collisions);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/joint_limit_margin", ros_parameters_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_topic", ros_parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/command_out_type", ros_parameters_.command_out_type);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_positions",
                                    ros_parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_velocities",
                                    ros_parameters_.publish_joint_velocities);
  error += !rosparam_shortcuts::get("", n, parameter_ns + "/publish_joint_accelerations",
                                    ros_parameters_.publish_joint_accelerations);

  // This parameter name was changed recently.
  // Try retrieving from the correct name. If it fails, then try the deprecated name.
  // TODO(andyz): remove this deprecation warning in ROS Noetic
  if (!rosparam_shortcuts::get("", n, parameter_ns + "/status_topic", ros_parameters_.status_topic))
  {
    ROS_WARN_NAMED(LOGNAME, "'status_topic' parameter is missing. Recently renamed from 'warning_topic'. Please update "
                            "the jogging yaml file.");
    error += !rosparam_shortcuts::get("", n, parameter_ns + "/warning_topic", ros_parameters_.status_topic);
  }

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Input checking
  if (ros_parameters_.publish_period <= 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'publish_period' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.num_outgoing_halt_msgs_to_publish < 0)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
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
  if (ros_parameters_.self_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.scene_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'scene_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (ros_parameters_.scene_collision_proximity_threshold < ros_parameters_.self_collision_proximity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should probably be less "
                            "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
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

// Listen to joint angles. Store them in a shared variable.
void JogInterfaceBase::jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  shared_variables_.lock();
  shared_variables_.joints = *msg;
  shared_variables_.unlock();
}

bool JogInterfaceBase::changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                                             moveit_msgs::ChangeDriftDimensions::Response& res)
{
  // These are std::atomic's, they are threadsafe without a mutex lock
  shared_variables_.drift_dimensions[0] = req.drift_x_translation;
  shared_variables_.drift_dimensions[1] = req.drift_y_translation;
  shared_variables_.drift_dimensions[2] = req.drift_z_translation;
  shared_variables_.drift_dimensions[3] = req.drift_x_rotation;
  shared_variables_.drift_dimensions[4] = req.drift_y_rotation;
  shared_variables_.drift_dimensions[5] = req.drift_z_rotation;

  res.success = true;
  return true;
}

// A separate thread for the heavy jogging calculations.
bool JogInterfaceBase::startJogCalcThread()
{
  if (!jog_calcs_)
    jog_calcs_.reset(new JogCalcs(ros_parameters_, planning_scene_monitor_->getRobotModelLoader()));

  jog_calc_thread_.reset(new std::thread([&]() { jog_calcs_->startMainLoop(shared_variables_); }));

  return true;
}

bool JogInterfaceBase::stopJogCalcThread()
{
  if (jog_calc_thread_)
  {
    if (jog_calc_thread_->joinable())
      jog_calc_thread_->join();

    jog_calc_thread_.reset();
  }

  return true;
}

// A separate thread for collision checking.
bool JogInterfaceBase::startCollisionCheckThread()
{
  if (!collision_checker_)
    collision_checker_.reset(new CollisionCheckThread(ros_parameters_, planning_scene_monitor_));

  collision_check_thread_.reset(new std::thread([&]() { collision_checker_->startMainLoop(shared_variables_); }));

  return true;
}

bool JogInterfaceBase::stopCollisionCheckThread()
{
  if (collision_check_thread_)
  {
    if (collision_check_thread_->joinable())
      collision_check_thread_->join();

    collision_check_thread_.reset();
  }

  return true;
}
}  // namespace moveit_jog_arm
