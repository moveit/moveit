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

#include <moveit/collision_distance_field/hybrid_collision_world.h>

namespace collision_detection 
{
CollisionWorldHybrid::CollisionWorldHybrid(double size_x, 
                                           double size_y,
                                           double size_z,
                                           bool use_signed_distance_field,
                                           double resolution,
                                           double collision_tolerance,
                                           double max_propogation_distance) :
  CollisionWorldFCL()
{
  cworld_distance_.reset(new collision_detection::CollisionWorldDistanceField(size_x, size_y, size_z, use_signed_distance_field, 
                                                                              resolution, collision_tolerance, max_propogation_distance));
}

CollisionWorldHybrid::CollisionWorldHybrid(const CollisionWorldHybrid &other) :
  CollisionWorldFCL(other)
{
  cworld_distance_.reset(new collision_detection::CollisionWorldDistanceField(*other.getCollisionWorldDistanceField().get()));
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest &req,
                                                       CollisionResult &res,
                                                       const CollisionRobot &robot,
                                                       const robot_state::RobotState &state) const
{
  cworld_distance_->checkCollision(req, res, robot, state);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest &req,
                                                       CollisionResult &res,
                                                       const CollisionRobot &robot,
                                                       const robot_state::RobotState &state,
                                                       boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->checkCollision(req, res, robot, state, gsr);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest &req,
                                                       CollisionResult &res,
                                                       const CollisionRobot &robot,
                                                       const robot_state::RobotState &state,
                                                       const AllowedCollisionMatrix &acm) const
{
  cworld_distance_->checkCollision(req, res, robot, state, acm);
}

void CollisionWorldHybrid::checkCollisionDistanceField(const CollisionRequest &req,
                                                       CollisionResult &res,
                                                       const CollisionRobot &robot,
                                                       const robot_state::RobotState &state,
                                                       const AllowedCollisionMatrix &acm,
                                                       boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->checkCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                                            CollisionResult &res, 
                                                            const CollisionRobot &robot, 
                                                            const robot_state::RobotState &state) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                                            CollisionResult &res, 
                                                            const CollisionRobot &robot, 
                                                            const robot_state::RobotState &state,
                                                            boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, gsr);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                                            CollisionResult &res, 
                                                            const CollisionRobot &robot, 
                                                            const robot_state::RobotState &state, 
                                                            const AllowedCollisionMatrix &acm) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, acm);
}

void CollisionWorldHybrid::checkRobotCollisionDistanceField(const CollisionRequest &req, 
                                                            CollisionResult &res, 
                                                            const CollisionRobot &robot, 
                                                            const robot_state::RobotState &state, 
                                                            const AllowedCollisionMatrix &acm,
                                                            boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->checkRobotCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::addToObject(const std::string &id, 
                                       const std::vector<shapes::ShapeConstPtr> &shapes, 
                                       const EigenSTL::vector_Affine3d &poses)
{
  CollisionWorldFCL::addToObject(id, shapes, poses);
  cworld_distance_->addToObject(id, shapes, poses);
}

void CollisionWorldHybrid::addToObject(const std::string &id, 
                                       const shapes::ShapeConstPtr &shape, 
                                       const Eigen::Affine3d &pose)
{
  CollisionWorldFCL::addToObject(id, shape, pose);
  cworld_distance_->addToObject(id, shape, pose);
}
bool CollisionWorldHybrid::moveShapeInObject(const std::string &id, 
                                             const shapes::ShapeConstPtr &shape, 
                                             const Eigen::Affine3d &pose)
{
  CollisionWorldFCL::moveShapeInObject(id, shape, pose);
  return cworld_distance_->moveShapeInObject(id, shape, pose);
}

bool CollisionWorldHybrid::removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape)
{
  CollisionWorldFCL::removeShapeFromObject(id, shape);
  return cworld_distance_->removeShapeFromObject(id, shape);
}
void CollisionWorldHybrid::removeObject(const std::string &id)
{
  CollisionWorldFCL::removeObject(id);
  cworld_distance_->removeObject(id);
}
void CollisionWorldHybrid::clearObjects()
{
  CollisionWorldFCL::clearObjects();
  cworld_distance_->clearObjects();
}

void CollisionWorldHybrid::getCollisionGradients(const CollisionRequest &req, 
                                                 CollisionResult &res, 
                                                 const CollisionRobot &robot, 
                                                 const robot_state::RobotState &state, 
                                                 const AllowedCollisionMatrix* acm,
                                                 boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->getCollisionGradients(req, res, robot, state, acm, gsr);
}

void CollisionWorldHybrid::getAllCollisions(const CollisionRequest &req, 
                                            CollisionResult &res, 
                                            const CollisionRobot &robot, 
                                            const robot_state::RobotState &state, 
                                            const AllowedCollisionMatrix* acm,
                                            boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  cworld_distance_->getAllCollisions(req, res, robot, state, acm, gsr);
}

}
