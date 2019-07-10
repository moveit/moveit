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

#include <moveit/collision_detection_bullet/collision_world_bt.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>
#include <moveit/collision_detection_bullet/tesseract/ros_tesseract_utils.h>
#include <moveit/collision_detection_bullet/tesseract/contact_checker_common.h>
#include <boost/bind.hpp>
#include <bullet/btBulletCollisionCommon.h>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorBt::NAME("Bullet");

CollisionWorldBt::CollisionWorldBt()
  : CollisionWorld()
  , bt_manager_(new tesseract::BulletDiscreteBVHManager)
  , bt_manager_CCD_(new tesseract::BulletCastBVHManager)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));

  auto fun =
      std::bind(&tesseract::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  bt_manager_->setIsContactAllowedFn(fun);
  bt_manager_CCD_->setIsContactAllowedFn(fun);
}

CollisionWorldBt::CollisionWorldBt(const WorldPtr& world)
  : CollisionWorld(world)
  , bt_manager_(new tesseract::BulletDiscreteBVHManager)
  , bt_manager_CCD_(new tesseract::BulletCastBVHManager)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);

  auto fun =
      std::bind(&tesseract::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  bt_manager_->setIsContactAllowedFn(fun);
  bt_manager_CCD_->setIsContactAllowedFn(fun);
}

CollisionWorldBt::CollisionWorldBt(const CollisionWorldBt& other, const WorldPtr& world) : CollisionWorld(other, world)
{
  bt_manager_ = other.bt_manager_->clone();
  bt_manager_CCD_ = other.bt_manager_CCD_->clone();

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
  checkRobotCollisionHelperCCD(req, res, robot, state1, state2, nullptr);
}

void CollisionWorldBt::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                           const CollisionRobot& robot, const robot_state::RobotState& state1,
                                           const robot_state::RobotState& state2,
                                           const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelperCCD(req, res, robot, state1, state2, &acm);
}

void CollisionWorldBt::checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res,
                                                    const CollisionRobot& robot, const robot_state::RobotState& state1,
                                                    const robot_state::RobotState& state2,
                                                    const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBt& robot_bt = dynamic_cast<const CollisionRobotBt&>(robot);

  tesseract::BulletCastBVHManagerPtr temp_clone_manager = bt_manager_CCD_->clone();
  std::vector<tesseract::COWPtr> attached_cows;
  robot_bt.addAttachedOjects(state1, attached_cows);
  std::vector<std::string> active_objects;

  for (const tesseract::COWPtr& cow : attached_cows)
  {
    temp_clone_manager->addCollisionObject(cow);
    temp_clone_manager->setCollisionObjectsTransform(
        cow->getName(), state1.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0],
        state2.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0]);
    active_objects.push_back(cow->getName());
  }

  for (const std::pair<std::string, tesseract::COWPtr>& cow : robot_bt.bt_manager_->getCollisionObjects())
  {
    tesseract::COWPtr new_cow = cow.second->clone();
    temp_clone_manager->addCollisionObject(new_cow);
    temp_clone_manager->setCollisionObjectsTransform(new_cow->getName(),
                                                     state1.getCollisionBodyTransform(new_cow->getName(), 0),
                                                     state2.getCollisionBodyTransform(new_cow->getName(), 0));
  }

  tesseract::getActiveLinkNamesRecursive(active_objects, robot_bt.getRobotModel()->getURDF()->getRoot(), false);
  temp_clone_manager->setActiveCollisionObjects(active_objects);
  temp_clone_manager->contactTest(res, req, acm);
}

void CollisionWorldBt::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBt& robot_bt = dynamic_cast<const CollisionRobotBt&>(robot);

  tesseract::BulletDiscreteBVHManagerPtr temp_clone_manager = bt_manager_->clone();
  std::vector<tesseract::COWPtr> attached_cows;
  robot_bt.addAttachedOjects(state, attached_cows);

  for (const std::pair<std::string, tesseract::COWPtr>& cow : robot_bt.bt_manager_->getCollisionObjects())
  {
    tesseract::COWPtr new_cow = cow.second->clone();
    temp_clone_manager->addCollisionObject(new_cow);
    temp_clone_manager->setCollisionObjectsTransform(new_cow->getName(),
                                                     state.getCollisionBodyTransform(new_cow->getName(), 0));
  }

  temp_clone_manager->contactTest(res, req, acm, attached_cows);
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

void CollisionWorldBt::addToManager(const World::Object* obj)
{
  std::vector<tesseract::CollisionObjectType> collision_object_types;

  for (const shapes::ShapeConstPtr& shape : obj->shapes_)
  {
    if (shape->type == shapes::MESH)
      collision_object_types.push_back(tesseract::CollisionObjectType::ConvexHull);
    else
      collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);
  }

  bt_manager_->addCollisionObject(obj->id_, collision_detection::BodyType::WORLD_OBJECT, obj->shapes_,
                                  obj->shape_poses_, collision_object_types, true);

  bt_manager_CCD_->addCollisionObject(obj->id_, collision_detection::BodyType::WORLD_OBJECT, obj->shapes_,
                                      obj->shape_poses_, collision_object_types, true);
}

void CollisionWorldBt::updateManagedObject(const std::string& id)
{
  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (bt_manager_->hasCollisionObject(id))
    {
      bt_manager_->removeCollisionObject(id);
      bt_manager_CCD_->removeCollisionObject(id);
      addToManager(it->second.get());
    }
    else
    {
      addToManager(it->second.get());
    }
  }
  else
  {
    if (bt_manager_->hasCollisionObject(id))
    {
      bt_manager_->removeCollisionObject(id);
      bt_manager_CCD_->removeCollisionObject(id);
    }
  }
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
  if (action == World::DESTROY)
  {
    bt_manager_->removeCollisionObject(obj->id_);
    bt_manager_CCD_->removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
}

void CollisionWorldBt::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                     const robot_state::RobotState& state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet distance calculation not implemented yet.");
}

void CollisionWorldBt::distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet distance calculation not implemented yet.");
}

}  // end of namespace collision_detection
