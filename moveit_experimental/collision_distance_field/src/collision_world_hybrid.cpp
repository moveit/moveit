/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#include <moveit/collision_distance_field/collision_world_hybrid.h>

namespace collision_detection
{
CollisionWorldHybrid::CollisionWorldHybrid(Eigen::Vector3d size, Eigen::Vector3d origin, bool use_signed_distance_field,
                                           double resolution, double collision_tolerance,
                                           double max_propogation_distance)
  : CollisionWorldFCL()
  , cworld_distance_(new collision_detection::CollisionWorldDistanceField(
        getWorld(), size, origin, use_signed_distance_field, resolution, collision_tolerance, max_propogation_distance))

{
}

CollisionWorldHybrid::CollisionWorldHybrid(const WorldPtr& world, Eigen::Vector3d size, Eigen::Vector3d origin,
                                           bool use_signed_distance_field, double resolution,
                                           double collision_tolerance, double max_propogation_distance)
  : CollisionWorldFCL(world)
  , cworld_distance_(new collision_detection::CollisionWorldDistanceField(
        getWorld(), size, origin, use_signed_distance_field, resolution, collision_tolerance, max_propogation_distance))
{
}

CollisionWorldHybrid::CollisionWorldHybrid(const CollisionWorldHybrid& other, const WorldPtr& world)
  : CollisionWorldFCL(other, world)
  , cworld_distance_(
        new collision_detection::CollisionWorldDistanceField(*other.getCollisionWorldDistanceField(), world))
{
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                       const CollisionRobot& robot,
                                                       const robot_state::RobotState& state) const
{
  cworld_distance_->checkCollision(req, res, robot, state);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                       const CollisionRobot& robot,
                                                       const robot_state::RobotState& state,
                                                       GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->checkCollision(req, res, robot, state, gsr);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                       const CollisionRobot& robot,
                                                       const robot_state::RobotState& state,
                                                       const AllowedCollisionMatrix& acm) const
{
  cworld_distance_->checkCollision(req, res, robot, state, acm);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                       const CollisionRobot& robot,
                                                       const robot_state::RobotState& state,
                                                       const AllowedCollisionMatrix& acm,
                                                       GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->checkCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                            const CollisionRobot& robot,
                                                            const robot_state::RobotState& state) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                            const CollisionRobot& robot,
                                                            const robot_state::RobotState& state,
                                                            GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, gsr);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                            const CollisionRobot& robot,
                                                            const robot_state::RobotState& state,
                                                            const AllowedCollisionMatrix& acm) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, acm);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                            const CollisionRobot& robot,
                                                            const robot_state::RobotState& state,
                                                            const AllowedCollisionMatrix& acm,
                                                            GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  cworld_distance_->setWorld(world);
  CollisionWorldFCL::setWorld(world);
}

void CollisionWorldHybrid::getCollisionGradients(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm,
                                                 GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->getCollisionGradients(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::getAllCollisions(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state,
                                            const AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr) const
{
  cworld_distance_->getAllCollisions(req, res, robot, state, acm, gsr);
}
}

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
const std::string collision_detection::CollisionDetectorAllocatorHybrid::NAME_("HYBRID");
