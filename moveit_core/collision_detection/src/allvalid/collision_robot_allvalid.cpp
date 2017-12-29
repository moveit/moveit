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

/* Author: Ioan Sucan, Jia Pan */

#include <moveit/collision_detection/allvalid/collision_robot_allvalid.h>

collision_detection::CollisionRobotAllValid::CollisionRobotAllValid(const robot_model::RobotModelConstPtr& kmodel,
                                                                    double padding, double scale)
  : CollisionRobot(kmodel, padding, scale)
{
}

collision_detection::CollisionRobotAllValid::CollisionRobotAllValid(const CollisionRobot& other) : CollisionRobot(other)
{
}

void collision_detection::CollisionRobotAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                     const robot_state::RobotState& state) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                     const robot_state::RobotState& state,
                                                                     const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                     const robot_state::RobotState& state1,
                                                                     const robot_state::RobotState& state2) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                     const robot_state::RobotState& state1,
                                                                     const robot_state::RobotState& state2,
                                                                     const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const robot_state::RobotState& state,
                                                                      const CollisionRobot& other_robot,
                                                                      const robot_state::RobotState& other_state) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const robot_state::RobotState& state,
                                                                      const CollisionRobot& other_robot,
                                                                      const robot_state::RobotState& other_state,
                                                                      const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2,
                                                                      const CollisionRobot& other_robot,
                                                                      const robot_state::RobotState& other_state1,
                                                                      const robot_state::RobotState& other_state2) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

void collision_detection::CollisionRobotAllValid::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2,
                                                                      const CollisionRobot& other_robot,
                                                                      const robot_state::RobotState& other_state1,
                                                                      const robot_state::RobotState& other_state2,
                                                                      const AllowedCollisionMatrix& acm) const
{
  res.collision = false;
  if (req.verbose)
    CONSOLE_BRIDGE_logInform("Using AllValid collision detection. No collision checking is performed.");
}

double collision_detection::CollisionRobotAllValid::distanceSelf(const robot_state::RobotState& state) const
{
  return 0.0;
}

double collision_detection::CollisionRobotAllValid::distanceSelf(const robot_state::RobotState& state,
                                                                 const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}

double collision_detection::CollisionRobotAllValid::distanceOther(const robot_state::RobotState& state,
                                                                  const CollisionRobot& other_robot,
                                                                  const robot_state::RobotState& other_state) const
{
  return 0.0;
}

double collision_detection::CollisionRobotAllValid::distanceOther(const robot_state::RobotState& state,
                                                                  const CollisionRobot& other_robot,
                                                                  const robot_state::RobotState& other_state,
                                                                  const AllowedCollisionMatrix& acm) const
{
  return 0.0;
}
