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

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_WORLD_BULLET_H_
#define MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_WORLD_BULLET_H_

#include <moveit/collision_detection_bullet/collision_robot_bullet.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>
#include <memory>

namespace collision_detection
{
class CollisionWorldBullet : public CollisionWorld
{
public:
  CollisionWorldBullet();

  explicit CollisionWorldBullet(const WorldPtr& world);

  CollisionWorldBullet(const CollisionWorldBullet& other, const WorldPtr& world);

  ~CollisionWorldBullet() override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state1, const robot_state::RobotState& state2) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                           const AllowedCollisionMatrix& acm) const override;
  void checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                           const CollisionWorld& other_world) const override;
  void checkWorldCollision(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                           const AllowedCollisionMatrix& acm) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                     const robot_state::RobotState& state) const override;

  void distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const override;

  void setWorld(const WorldPtr& world) override;

protected:
  /** \brief Bundles the different checkWorldCollision functions into a single function */
  void checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                                 const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different checkRobotCollision functions into a single function */
  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                 const robot_state::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different continuous checkRobotCollision functions into a single function */
  void checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                    const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                    const AllowedCollisionMatrix* acm) const;

  /** \brief Adds a world object to the collision managers */
  void addToManager(const World::Object* obj);

  /** \brief Updates a managed collision object with its world representation.
   *
   * We have three cases: 1) the object is part of the manager and not of world --> delete it
   *                      2) the object is not in the manager, therefore register to manager,
   *                      3) the object is in the manager then delete and add the modified */
  void updateManagedObject(const std::string& id);

  /** \brief Handles all discrete collision checks */
  collision_detection_bullet::BulletDiscreteBVHManagerPtr manager_;

private:
  /** \brief Callback function executed for each change to the world environment */
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);

  World::ObserverHandle observer_handle_;
};
}  // namespace collision_detection

#endif  // MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_WORLD_BULLET_H_
