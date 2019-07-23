/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_ROBOT_BULLET_H_
#define MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_ROBOT_BULLET_H_

#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>
#include <moveit/collision_detection/collision_robot.h>

namespace collision_detection
{
class CollisionRobotBullet : public CollisionRobot
{
  friend class CollisionWorldBullet;

public:
  CollisionRobotBullet(const robot_model::RobotModelConstPtr& robot_model, double padding = 0.0, double scale = 1.0);

  CollisionRobotBullet(const CollisionRobotBullet& other);

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const robot_state::RobotState& state) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                          const robot_state::RobotState& state2) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                          const robot_state::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                           const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                           const CollisionRobot& other_robot, const robot_state::RobotState& other_state,
                           const AllowedCollisionMatrix& acm) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                           const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state1,
                           const robot_state::RobotState& other_state2) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                           const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state1, const robot_state::RobotState& other_state2,
                           const AllowedCollisionMatrix& acm) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const robot_state::RobotState& state) const override;

  void distanceOther(const DistanceRequest& req, DistanceResult& res, const robot_state::RobotState& state,
                     const CollisionRobot& other_robot, const robot_state::RobotState& other_state) const override;

protected:
  /** \brief Updates the poses of the objects in the manager according to given robot state */
  void updateTransformsFromState(const robot_state::RobotState& state,
                                 const collision_detection_bullet::BulletDiscreteBVHManagerPtr& manager) const;

  /** \brief Updates the collision objects saved in the manager to reflect a new padding or scaling of the robot links
   */
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

  /** \brief All of the attached objects in the robot state are wrapped into bullet collision objects */
  void addAttachedOjects(const robot_state::RobotState& state,
                         std::vector<collision_detection_bullet::CollisionObjectWrapperPtr>& cows) const;

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                                const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different continuous checkSelfCollision functions into a single function */
  void checkSelfCollisionCCDHelper(const CollisionRequest& req, CollisionResult& res,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                   const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different checkOtherCollision functions into a single function */
  void checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                 const robot_state::RobotState& other_state, const AllowedCollisionMatrix* acm) const;

  /** \brief Construts a bullet collision object out of a robot link */
  void addLinkAsCollisionObjectWrapper(const urdf::LinkSharedPtr& link);

  /** \brief Handles all self collision checks */
  collision_detection_bullet::BulletDiscreteBVHManagerPtr manager_;
};
}  // namespace collision_detection

#endif  // MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_ROBOT_BULLET_H_
