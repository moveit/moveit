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

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_HYBRID_H_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_HYBRID_H_

#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>

namespace collision_detection
{
class CollisionRobotHybrid;

class CollisionWorldHybrid : public CollisionWorldFCL
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionWorldHybrid(Eigen::Vector3d size = Eigen::Vector3d(3, 3, 4),
                       Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0), bool use_signed_distance_field = false,
                       double resolution = .02, double collision_tolerance = 0.0,
                       double max_propogation_distance = .25);

  explicit CollisionWorldHybrid(const WorldPtr& world, Eigen::Vector3d size = Eigen::Vector3d(3, 3, 4),
                                Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0),
                                bool use_signed_distance_field = false, double resolution = .02,
                                double collision_tolerance = 0.0, double max_propogation_distance = .25);

  CollisionWorldHybrid(const CollisionWorldHybrid& other, const WorldPtr& world);

  virtual ~CollisionWorldHybrid()
  {
  }

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, const AllowedCollisionMatrix& acm,
                                   GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                        const robot_state::RobotState& state) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                        const robot_state::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                        const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                        const robot_state::RobotState& state, const AllowedCollisionMatrix& acm,
                                        GroupStateRepresentationPtr& gsr) const;

  virtual void setWorld(const WorldPtr& world);

  void getCollisionGradients(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                             const robot_state::RobotState& state, const AllowedCollisionMatrix* acm,
                             GroupStateRepresentationPtr& gsr) const;

  void getAllCollisions(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                        const robot_state::RobotState& state, const AllowedCollisionMatrix* acm,
                        GroupStateRepresentationPtr& gsr) const;

  const CollisionWorldDistanceFieldConstPtr getCollisionWorldDistanceField() const
  {
    return cworld_distance_;
  }

protected:
  CollisionWorldDistanceFieldPtr cworld_distance_;
};
}

#endif
