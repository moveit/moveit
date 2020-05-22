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
static constexpr double MIN_RECOMMENDED_COLLISION_RATE = 10;
constexpr double EPSILON = 1e-6;  // For very small numeric comparisons

namespace moveit_jog_arm
{
// Constructor for the class that handles collision checking
CollisionCheckThread::CollisionCheckThread(
    const moveit_jog_arm::JogArmParameters& parameters,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : parameters_(parameters), planning_scene_monitor_(planning_scene_monitor)
{
  ros::NodeHandle nh;

  if (parameters_.collision_check_rate < MIN_RECOMMENDED_COLLISION_RATE)
    ROS_WARN_STREAM_THROTTLE_NAMED(5, LOGNAME, "Collision check rate is low, increase it in yaml file if CPU allows");

  // subscribe to joints
  joint_state_sub_ = nh.subscribe(parameters.joint_topic, 1, &CollisionCheckThread::jointStateCB, this);

  // Wait for incoming topics to appear
  ROS_DEBUG_NAMED(LOGNAME, "Waiting for JointState topic");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters.joint_topic);
}

planning_scene_monitor::LockedPlanningSceneRO CollisionCheckThread::getLockedPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

void CollisionCheckThread::run(JogArmShared& shared_variables)
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

  // This variable scales the robot velocity when a collision is close
  double velocity_scale = 1;

  collision_check_type_ =
      (parameters_.collision_check_type == "threshold_distance" ? K_THRESHOLD_DISTANCE : K_STOP_DISTANCE);

  // Variables for stop-distance-based collision checking
  double current_collision_distance = 0;
  double derivative_of_collision_distance = 0;
  double prev_collision_distance = 0;
  double est_time_to_collision = 0;
  double safety_factor = parameters_.collision_distance_safety_factor;

  collision_detection::AllowedCollisionMatrix acm = getLockedPlanningSceneRO()->getAllowedCollisionMatrix();

  /////////////////////////////////////////////////
  // Spin while checking collisions
  /////////////////////////////////////////////////
  sensor_msgs::JointState joint_state;

  while (ros::ok() && !shared_variables.stop_requested)
  {
    if (!shared_variables.paused)
    {
      {
        // Copy the latest joint state
        const std::lock_guard<std::mutex> lock(CollisionCheckThread);
        joint_state = latest_joint_state_;
      }

      for (std::size_t i = 0; i < joint_state.position.size(); ++i)
        current_state.setJointPositions(joint_state.name[i], &joint_state.position[i]);

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
      // If threshold distances were specified
      else if (collision_check_type_ == K_THRESHOLD_DISTANCE)
      {
        // If we are far from a collision, velocity_scale should be 1.
        // If we are very close to a collision, velocity_scale should be ~zero.
        // When scene_collision_proximity_threshold is breached, start decelerating exponentially.
        if (scene_collision_distance < parameters_.scene_collision_proximity_threshold)
        {
          // velocity_scale = e ^ k * (collision_distance - threshold)
          // k = - ln(0.001) / collision_proximity_threshold
          // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
          // velocity_scale should equal 0.001 when collision_distance is at zero.
          velocity_scale = std::min(velocity_scale,
                                    exp(scene_velocity_scale_coefficient *
                                        (scene_collision_distance - parameters_.scene_collision_proximity_threshold)));
        }

        if (self_collision_distance < parameters_.self_collision_proximity_threshold)
        {
          velocity_scale =
              std::min(velocity_scale, exp(self_velocity_scale_coefficient *
                                           (self_collision_distance - parameters_.self_collision_proximity_threshold)));
        }
      }
      // Else, we stop based on worst-case stopping distance
      else
      {
        // Retrieve the worst-case time to stop, based on current joint velocities

        // Calculate rate of change of distance to nearest collision
        current_collision_distance = std::min(scene_collision_distance, self_collision_distance);
        derivative_of_collision_distance =
            (current_collision_distance - prev_collision_distance) / collision_rate.expectedCycleTime().toSec();

        if (current_collision_distance < parameters_.min_allowable_collision_distance &&
            derivative_of_collision_distance <= 0)
        {
          velocity_scale = 0;
        }
        // Only bother doing calculations if we are moving toward the nearest collision
        else if (derivative_of_collision_distance < -EPSILON)
        {
          // At the rate the collision distance is decreasing, how long until we collide?
          est_time_to_collision = fabs(current_collision_distance / derivative_of_collision_distance);

          // halt if we can't stop fast enough (including the safety factor)
          if (est_time_to_collision < (safety_factor * shared_variables.worst_case_stop_time))
          {
            velocity_scale = 0;
          }
        }

        // Update for the next iteration
        prev_collision_distance = current_collision_distance;
      }

      // Communicate a velocity-scaling factor back to the other threads
      shared_variables.collision_velocity_scale = velocity_scale;
    }

    collision_rate.sleep();
  }
}

void CollisionCheckThread::jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = *msg;
}
}  // namespace moveit_jog_arm
