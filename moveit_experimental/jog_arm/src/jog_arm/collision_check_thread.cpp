///////////////////////////////////////////////////////////////////////////////
//      Title     : collision_check_thread.cpp
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

#include <jog_arm/collision_check_thread.h>

namespace jog_arm
{
// Constructor for the class that handles collision checking
CollisionCheckThread::CollisionCheckThread(const jog_arm::JogArmParameters parameters, JogArmShared& shared_variables,
                                           const robot_model_loader::RobotModelLoaderPtr model_loader_ptr)
{
  // If user specified true in yaml file
  if (parameters.check_collisions)
  {
    // MoveIt! Setup
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

      // Scale robot velocity according to collision proximity and user-defined thresholds.
      // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
      // Proximity decreasing --> decelerate
      double velocity_scale = 1;

      // Ramp velocity down linearly when collision proximity is between lower_collision_proximity_threshold and
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
}  // namespace jog_arm
