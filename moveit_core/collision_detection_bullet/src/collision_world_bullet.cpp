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

#include <moveit/collision_detection_bullet/collision_world_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h>
#include <moveit/collision_detection_bullet/bullet_integration/contact_checker_common.h>
#include <boost/bind.hpp>
#include <bullet/btBulletCollisionCommon.h>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorBullet::NAME("Bullet");

CollisionWorldBullet::CollisionWorldBullet()
  : CollisionWorld(), manager_(new collision_detection_bullet::BulletDiscreteBVHManager)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));

  auto fun = std::bind(&collision_detection_bullet::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3);
  manager_->setIsContactAllowedFn(fun);
}

CollisionWorldBullet::CollisionWorldBullet(const WorldPtr& world)
  : CollisionWorld(world), manager_(new collision_detection_bullet::BulletDiscreteBVHManager)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);

  auto fun = std::bind(&collision_detection_bullet::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3);
  manager_->setIsContactAllowedFn(fun);
}

CollisionWorldBullet::CollisionWorldBullet(const CollisionWorldBullet& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
  manager_ = other.manager_->clone();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));
}

CollisionWorldBullet::~CollisionWorldBullet()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionRobot& robot, const robot_state::RobotState& state,
                                               const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionRobot& robot, const robot_state::RobotState& state1,
                                               const robot_state::RobotState& state2) const
{
  checkRobotCollisionHelperCCD(req, res, robot, state1, state2, nullptr);
}

void CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionRobot& robot, const robot_state::RobotState& state1,
                                               const robot_state::RobotState& state2,
                                               const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelperCCD(req, res, robot, state1, state2, &acm);
}

void CollisionWorldBullet::checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res,
                                                        const CollisionRobot& robot,
                                                        const robot_state::RobotState& state1,
                                                        const robot_state::RobotState& state2,
                                                        const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Continuous collision checking not implemented yet");
}

void CollisionWorldBullet::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                     const CollisionRobot& robot, const robot_state::RobotState& state,
                                                     const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBullet& robot_bt = dynamic_cast<const CollisionRobotBullet&>(robot);

  collision_detection_bullet::BulletDiscreteBVHManagerPtr discrete_clone_manager = manager_->clone();
  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> attached_cows;
  robot_bt.addAttachedOjects(state, attached_cows);

  for (const std::pair<const std::string, collision_detection_bullet::CollisionObjectWrapperPtr>& cow :
       robot_bt.manager_->getCollisionObjects())
  {
    collision_detection_bullet::CollisionObjectWrapperPtr new_cow = cow.second->clone();
    discrete_clone_manager->addCollisionObject(new_cow);
    discrete_clone_manager->setCollisionObjectsTransform(new_cow->getName(),
                                                         state.getCollisionBodyTransform(new_cow->getName(), 0));
  }

  discrete_clone_manager->setActiveCollisionObjects(robot_bt.manager_->getActiveCollisionObjects());

  discrete_clone_manager->contactTest(res, req, acm, attached_cows);
}

void CollisionWorldBullet::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void CollisionWorldBullet::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                               const CollisionWorld& other_world,
                                               const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void CollisionWorldBullet::checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                     const CollisionWorld& other_world,
                                                     const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet checking with other world not implemented yet.");
}

void CollisionWorldBullet::addToManager(const World::Object* obj)
{
  std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types;

  for (const shapes::ShapeConstPtr& shape : obj->shapes_)
  {
    if (shape->type == shapes::MESH)
      collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::CONVEX_HULL);
    else
      collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);
  }

  manager_->addCollisionObject(obj->id_, collision_detection::BodyType::WORLD_OBJECT, obj->shapes_, obj->shape_poses_,
                               collision_object_types, true);
}

void CollisionWorldBullet::updateManagedObject(const std::string& id)
{
  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
      addToManager(it->second.get());
    }
    else
    {
      addToManager(it->second.get());
    }
  }
  else
  {
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
    }
  }
}

void CollisionWorldBullet::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldBullet::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    manager_->removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
}

void CollisionWorldBullet::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                         const robot_state::RobotState& state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet distance calculation not implemented yet.");
}

void CollisionWorldBullet::distanceWorld(const DistanceRequest& req, DistanceResult& res,
                                         const CollisionWorld& world) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet distance calculation not implemented yet.");
}

}  // end of namespace collision_detection
