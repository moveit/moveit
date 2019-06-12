/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

/* Author: Jens Petit */

#include <moveit/collision_detection_bullet/collision_world_bt.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>

#include <boost/bind.hpp>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorBt::NAME("Bullet");

CollisionWorldBt::CollisionWorldBt() : CollisionWorld()
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));
}

CollisionWorldBt::CollisionWorldBt(const WorldPtr& world) : CollisionWorld(world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldBt::CollisionWorldBt(const CollisionWorldBt& other, const WorldPtr& world) : CollisionWorld(other, world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));
}

CollisionWorldBt::~CollisionWorldBt()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionWorldBt::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void CollisionWorldBt::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionRobot& robot, const robot_state::RobotState& state,
                                           const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionWorldBt::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionRobot& robot, const robot_state::RobotState& state1,
                                           const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet continuous collision checking not yet implemented");
}

void CollisionWorldBt::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionRobot& robot, const robot_state::RobotState& state1,
                                           const robot_state::RobotState& state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet continuous collision checking not yet implemented");
}

void CollisionWorldBt::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet robot world collision checking not yet implemented");
}

void CollisionWorldBt::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void CollisionWorldBt::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionWorld& other_world, const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void CollisionWorldBt::checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionWorld& other_world,
                                                 const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet checking with other world not implemented yet.");
}

void CollisionWorldBt::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldBt::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
}

void CollisionWorldBt::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                     const robot_state::RobotState& state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet checking with other world not implemented yet.");
}

void CollisionWorldBt::distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet checking with other world not implemented yet.");
}

}  // end of namespace collision_detection
