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

#include <moveit/collision_detection/collision_world.h>
#include <geometric_shapes/shape_operations.h>

namespace collision_detection
{
CollisionWorld::CollisionWorld() : world_(new World()), world_const_(world_)
{
}

CollisionWorld::CollisionWorld(const WorldPtr& world) : world_(world), world_const_(world)
{
}

CollisionWorld::CollisionWorld(const CollisionWorld& other, const WorldPtr& world) : world_(world), world_const_(world)
{
}

void CollisionWorld::checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                    const robot_state::RobotState& state) const
{
  robot.checkSelfCollision(req, res, state);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state);
}

void CollisionWorld::checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                    const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  robot.checkSelfCollision(req, res, state, acm);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state, acm);
}

void CollisionWorld::checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                    const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
{
  robot.checkSelfCollision(req, res, state1, state2);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state1, state2);
}

void CollisionWorld::checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                    const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                    const AllowedCollisionMatrix& acm) const
{
  robot.checkSelfCollision(req, res, state1, state2, acm);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state1, state2, acm);
}

void CollisionWorld::setWorld(const WorldPtr& world)
{
  world_ = world;
  if (!world_)
    world_.reset(new World());

  world_const_ = world;
}

}  // end of namespace collision_detection