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

/*      Title     : jog_server.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 12/31/2018
 *      Author    : Andy Zelenak
 */

#include <moveit_jog_arm/jog_arm.h>

namespace
{
constexpr char LOGNAME[] = "jog_server";
constexpr char ROS_THREADS = 4;

struct JogServerParameters
{
  bool read_error = true;
  std::string cartesian_command_in_topic = "";
  std::string joint_command_in_topic = "";
};

JogServerParameters readParameters()
{
  ros::NodeHandle nh;
  JogServerParameters params;

  // Load parameters
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);

  if (parameter_ns.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "A namespace must be specified in the launch file, like:");
    ROS_ERROR_STREAM_NAMED(LOGNAME, "<param name=\"parameter_ns\" "
                                    "type=\"string\" "
                                    "value=\"left_jog_server\" />");
    params.read_error = true;
  }
  else
  {
    std::size_t error = 0;
    error += !rosparam_shortcuts::get("", nh, parameter_ns + "/cartesian_command_in_topic",
                                      params.cartesian_command_in_topic);
    error += !rosparam_shortcuts::get("", nh, parameter_ns + "/joint_command_in_topic", params.joint_command_in_topic);
    params.read_error = (error != 0);
  }
  return params;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::AsyncSpinner spinner(ROS_THREADS);
  spinner.start();

  ros::NodeHandle nh;

  const auto params = readParameters();
  if (params.read_error)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error reading parameters.");
    exit(EXIT_FAILURE);
  }

  // Load the planning scene monitor
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Start the planning scene monitor
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the jog server
  moveit_jog_arm::JogArm jog_arm(planning_scene_monitor);

  // ROS subscriptions. Share the data with the worker threads
  auto cmd_sub =
      nh.subscribe(params.cartesian_command_in_topic, 1, &moveit_jog_arm::JogArm::provideTwistStampedCommand, &jog_arm);
  auto joint_jog_cmd_sub =
      nh.subscribe(params.joint_command_in_topic, 1, &moveit_jog_arm::JogArm::provideJointCommand, &jog_arm);

  // ROS Server for allowing drift in some dimensions
  auto drift_dimensions_server =
      nh.advertiseService(nh.getNamespace() + "/" + ros::this_node::getName() + "/change_drift_dimensions",
                          &moveit_jog_arm::JogArm::changeDriftDimensions, &jog_arm);

  // ROS Server for changing the control dimensions
  auto dims_server =
      nh.advertiseService(nh.getNamespace() + "/" + ros::this_node::getName() + "/change_control_dimensions",
                          &moveit_jog_arm::JogArm::changeControlDimensions, &jog_arm);

  // Start the jog server (runs in the ros spinner)
  jog_arm.start();

  // Wait for ros to shutdown
  ros::waitForShutdown();

  // Stop the jog server
  jog_arm.stop();

  return 0;
}
