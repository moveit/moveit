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

/* Author: Ioan Sucan */

#include <moveit/collision_detection/allvalid/collision_world_allvalid.h>

collision_detection::CollisionWorldAllValid::CollisionWorldAllValid() : CollisionWorld()
{
}

collision_detection::CollisionWorldAllValid::CollisionWorldAllValid(const WorldPtr& world) : CollisionWorld(world)
{
}

collision_detection::CollisionWorldAllValid::CollisionWorldAllValid(const CollisionWorld& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
}

void collision_detection::CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionRobot& robot,
                                                                      const robot_state::RobotState& state) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionRobot& robot,
                                                                      const robot_state::RobotState& state,
                                                                      const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionRobot& robot,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionWorldAllValid::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionRobot& robot,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2,
                                                                      const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionWorldAllValid::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionWorld& other_world) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionWorldAllValid::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const CollisionWorld& other_world,
                                                                      const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

double collision_detection::CollisionWorldAllValid::distanceRobot(const CollisionRobot& robot,
                                                                  const robot_state::RobotState& state) const
{
  return 0.0;
}

double collision_detection::CollisionWorldAllValid::distanceRobot(const CollisionRobot& robot,
                                                                  const robot_state::RobotState& state,
                                                                  const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

double collision_detection::CollisionWorldAllValid::distanceWorld(const CollisionWorld& world) const
{
  return 0.0;
}

double collision_detection::CollisionWorldAllValid::distanceWorld(const CollisionWorld& world,
                                                                  const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

#include <moveit/collision_detection/allvalid/collision_detector_allocator_allvalid.h>
const std::string collision_detection::CollisionDetectorAllocatorAllValid::NAME_("ALL_VALID");
