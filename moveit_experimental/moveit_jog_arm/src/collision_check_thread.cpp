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
  // Reset loop termination flag
  stop_requested_ = false;

  // Init collision request
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = parameters_.move_group_name;
  collision_request.distance = true;  // enable distance-based collision checking
  collision_detection::CollisionResult collision_result;

  // Copy the planning scene's version of current state into new memory
  moveit::core::RobotState current_state(getLockedPlanningSceneRO()->getCurrentState());

  double velocity_scale_coefficient = -log(0.001) / parameters_.collision_proximity_threshold;
  ros::Rate collision_rate(parameters_.collision_check_rate);

  // Scale robot velocity according to collision proximity and user-defined thresholds.
  // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
  // Proximity decreasing --> decelerate
  double velocity_scale = 1;

  /////////////////////////////////////////////////
  // Spin while checking collisions
  /////////////////////////////////////////////////
  while (ros::ok() && !stop_requested_)
  {
    shared_variables.lock();
    sensor_msgs::JointState jts = shared_variables.joints;
    shared_variables.unlock();

    for (std::size_t i = 0; i < jts.position.size(); ++i)
      current_state.setJointPositions(jts.name[i], &jts.position[i]);

    collision_result.clear();
    current_state.updateCollisionBodyTransforms();

    // Do a thread-safe distance-based collision detection
    getLockedPlanningSceneRO()->checkCollision(collision_request, collision_result, current_state);

    // If we're definitely in collision, stop immediately
    if (collision_result.collision)
    {
      velocity_scale = 0;
    }

    // If we are far from a collision, velocity_scale should be 1.
    // If we are very close to a collision, velocity_scale should be ~zero.
    // When collision_proximity_threshold is breached, start decelerating exponentially.
    else if (collision_result.distance < parameters_.collision_proximity_threshold)
    {
      // velocity_scale = e ^ k * (collision_result.distance - threshold)
      // k = - ln(0.001) / collision_proximity_threshold
      // velocity_scale should equal one when collision_result.distance is at collision_proximity_threshold.
      // velocity_scale should equal 0.001 when collision_result.distance is at zero.
      velocity_scale =
          exp(velocity_scale_coefficient * (collision_result.distance - parameters_.collision_proximity_threshold));
    }

    shared_variables.lock();
    shared_variables.collision_velocity_scale = velocity_scale;
    shared_variables.unlock();

    collision_rate.sleep();
  }
}

void CollisionCheckThread::stopMainLoop()
{
  stop_requested_ = true;
}
}  // namespace moveit_jog_arm
