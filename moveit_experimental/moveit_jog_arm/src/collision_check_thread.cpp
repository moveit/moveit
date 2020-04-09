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

/*      Title     : collision_check_thread.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <moveit_jog_arm/collision_check_thread.h>

static const std::string LOGNAME = "collision_check_thread";
static const double MIN_RECOMMENDED_COLLISION_RATE = 10;

namespace moveit_jog_arm
{
// Constructor for the class that handles collision checking
CollisionCheckThread::CollisionCheckThread(
    const moveit_jog_arm::JogArmParameters& parameters,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : parameters_(parameters), planning_scene_monitor_(planning_scene_monitor)
{
  if (parameters_.collision_check_rate < MIN_RECOMMENDED_COLLISION_RATE)
    ROS_WARN_STREAM_THROTTLE_NAMED(5, LOGNAME, "Collision check rate is low, increase it in yaml file if CPU allows");
}

planning_scene_monitor::LockedPlanningSceneRO CollisionCheckThread::getLockedPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

void CollisionCheckThread::startMainLoop(JogArmShared& shared_variables)
{
  // Init collision request
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = parameters_.move_group_name;
  collision_request.distance = true;  // enable distance-based collision checking
  collision_detection::CollisionResult collision_result;

  // Copy the planning scene's version of current state into new memory
  moveit::core::RobotState current_state(getLockedPlanningSceneRO()->getCurrentState());

  const double self_velocity_scale_coefficient = -log(0.001) / parameters_.self_collision_proximity_threshold;
  const double scene_velocity_scale_coefficient = -log(0.001) / parameters_.scene_collision_proximity_threshold;
  ros::Rate collision_rate(parameters_.collision_check_rate);

  double self_collision_distance = 0;
  double scene_collision_distance = 0;
  bool collision_detected;

  // Scale robot velocity according to collision proximity and user-defined thresholds.
  // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
  // Proximity decreasing --> decelerate
  double velocity_scale = 1;

  collision_detection::AllowedCollisionMatrix acm = getLockedPlanningSceneRO()->getAllowedCollisionMatrix();
  /////////////////////////////////////////////////
  // Spin while checking collisions
  /////////////////////////////////////////////////
  while (ros::ok() && !shared_variables.stop_requested)
  {
    if (!shared_variables.paused)
    {
      shared_variables.lock();
      sensor_msgs::JointState jts = shared_variables.joints;
      shared_variables.unlock();

      for (std::size_t i = 0; i < jts.position.size(); ++i)
        current_state.setJointPositions(jts.name[i], &jts.position[i]);

      current_state.updateCollisionBodyTransforms();
      collision_detected = false;

      // Do a thread-safe distance-based collision detection
      collision_result.clear();
      getLockedPlanningSceneRO()->getCollisionEnv()->checkRobotCollision(collision_request, collision_result,
                                                                         current_state);
      scene_collision_distance = collision_result.distance;
      collision_detected |= collision_result.collision;

      collision_result.clear();
      getLockedPlanningSceneRO()->getCollisionEnvUnpadded()->checkSelfCollision(collision_request, collision_result,
                                                                                current_state, acm);
      self_collision_distance = collision_result.distance;
      collision_detected |= collision_result.collision;

      velocity_scale = 1;
      // If we're definitely in collision, stop immediately
      if (collision_detected)
      {
        velocity_scale = 0;
      }

      // If we are far from a collision, velocity_scale should be 1.
      // If we are very close to a collision, velocity_scale should be ~zero.
      // When scene_collision_proximity_threshold is breached, start decelerating exponentially.
      if (scene_collision_distance < parameters_.scene_collision_proximity_threshold)
      {
        // velocity_scale = e ^ k * (collision_distance - threshold)
        // k = - ln(0.001) / collision_proximity_threshold
        // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
        // velocity_scale should equal 0.001 when collision_distance is at zero.
        velocity_scale =
            std::min(velocity_scale, exp(scene_velocity_scale_coefficient *
                                         (scene_collision_distance - parameters_.scene_collision_proximity_threshold)));
      }

      if (self_collision_distance < parameters_.self_collision_proximity_threshold)
      {
        velocity_scale =
            std::min(velocity_scale, exp(self_velocity_scale_coefficient *
                                         (self_collision_distance - parameters_.self_collision_proximity_threshold)));
      }

      shared_variables.lock();
      shared_variables.collision_velocity_scale = velocity_scale;
      shared_variables.unlock();
    }

    collision_rate.sleep();
  }
}
}  // namespace moveit_jog_arm
