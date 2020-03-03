/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Jia Pan, Jens Petit */

#include <moveit/collision_detection/allvalid/collision_env_allvalid.h>
#include <moveit/collision_detection/allvalid/collision_detector_allocator_allvalid.h>

const std::string collision_detection::CollisionDetectorAllocatorAllValid::NAME("ALL_VALID");

collision_detection::CollisionEnvAllValid::CollisionEnvAllValid(const moveit::core::RobotModelConstPtr& robot_model,
                                                                double padding, double scale)
  : CollisionEnv(robot_model, padding, scale)
{
}

collision_detection::CollisionEnvAllValid::CollisionEnvAllValid(const moveit::core::RobotModelConstPtr& robot_model,
                                                                const WorldPtr& world, double padding, double scale)
  : CollisionEnv(robot_model, world, padding, scale)
{
}

collision_detection::CollisionEnvAllValid::CollisionEnvAllValid(const CollisionEnv& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
}

void collision_detection::CollisionEnvAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const moveit::core::RobotState& state) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const moveit::core::RobotState& state,
                                                                    const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const moveit::core::RobotState& state1,
                                                                    const moveit::core::RobotState& state2) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const moveit::core::RobotState& state1,
                                                                    const moveit::core::RobotState& state2,
                                                                    const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::distanceRobot(const collision_detection::DistanceRequest& req,
                                                              collision_detection::DistanceResult& res,
                                                              const moveit::core::RobotState& state) const
{
  res.collision = false;
}

double collision_detection::CollisionEnvAllValid::distanceRobot(const moveit::core::RobotState& state) const
{
  return 0.0;
}

double collision_detection::CollisionEnvAllValid::distanceRobot(const moveit::core::RobotState& state,
                                                                const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

void collision_detection::CollisionEnvAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                   const moveit::core::RobotState& state) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                   const moveit::core::RobotState& state,
                                                                   const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    ROS_INFO_NAMED("collision_detection", "Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionEnvAllValid::distanceSelf(const collision_detection::DistanceRequest& req,
                                                             collision_detection::DistanceResult& res,
                                                             const moveit::core::RobotState& state) const
{
  res.collision = false;
}
