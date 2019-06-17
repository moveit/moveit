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

#include <boost/bind.hpp>
#include <bullet/btBulletCollisionCommon.h>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorBt::NAME("Bullet");

CollisionWorldBt::CollisionWorldBt() : CollisionWorld()
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));

  bt_manager_.setIsContactAllowedFn(std::bind(&collision_detection::CollisionWorldBt::allowedCollisionCheck, this,
                                              std::placeholders::_1, std::placeholders::_2));
}

CollisionWorldBt::CollisionWorldBt(const WorldPtr& world) : CollisionWorld(world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);

  bt_manager_.setIsContactAllowedFn(std::bind(&collision_detection::CollisionWorldBt::allowedCollisionCheck, this,
                                              std::placeholders::_1, std::placeholders::_2));
}

CollisionWorldBt::CollisionWorldBt(const CollisionWorldBt& other, const WorldPtr& world) : CollisionWorld(other, world)
{
  // TODO add copy constructor for new manager

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBt::notifyObjectChange, this, _1, _2));

  bt_manager_.setIsContactAllowedFn(other.bt_manager_.getIsContactAllowedFn());
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

  tesseract::tesseract_bullet::Link2Cow link2cow;
  link2cow = bt_manager_.getCollisionObjects();

  for (const auto cow : link2cow)
  {

    tesseract::tesseract_bullet::COWPtr new_cow = cow.second->clone();
    new_cow->setWorldTransform(cow.second->getWorldTransform());

    if (!robot_bt.bt_manager_CCD_.hasCollisionObject(cow.first))
    {
      robot_bt.bt_manager_CCD_.addCollisionObject(new_cow);
    }
    else
    {
      robot_bt.bt_manager_CCD_.removeCollisionObject(cow.first);
      robot_bt.bt_manager_CCD_.addCollisionObject(new_cow);
    }
    ROS_DEBUG_STREAM("Added " << cow.first << " to the bullet manager from world");
  }

  robot_bt.bt_manager_CCD_.setActiveCollisionObjects(robot_bt.getRobotModel()->getLinkModelNames());
  robot_bt.bt_manager_CCD_.setContactDistanceThreshold(0.1);

  robot_bt.updateTransformsFromStateCCD(state1, state2);

  robot_bt.acm_ = acm;
  robot_bt.bt_manager_CCD_.contactTest(res, tesseract::ContactTestType::ALL, req);
}

void CollisionWorldBt::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBt& robot_bt = dynamic_cast<const CollisionRobotBt&>(robot);

  robot_bt.updateTransformsFromState(state);

  tesseract::tesseract_bullet::Link2Cow link2cow;
  link2cow = robot_bt.bt_manager_.getCollisionObjects();

  for (const auto cow : link2cow)
  {
    tesseract::tesseract_bullet::COWPtr new_cow = cow.second->clone();
    new_cow->setWorldTransform(cow.second->getWorldTransform());

    ROS_DEBUG_STREAM("Added " << cow.first << " to the bullet manager from robot");
    if (!bt_manager_.hasCollisionObject(cow.first))
    {
      bt_manager_.addCollisionObject(new_cow);
    }
    else
    {
      bt_manager_.removeCollisionObject(cow.first);
      bt_manager_.addCollisionObject(new_cow);
    }
  }

  acm_ = acm;

  bt_manager_.contactTest(res, tesseract::ContactTestType::ALL, req);

  for (const auto cow : link2cow)
  {
    bt_manager_.removeCollisionObject(cow.first);
  }

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(robot.getRobotModel());
    distanceRobot(dreq, dres, robot, state);
    res.distance = dres.minimum_distance.distance;
  }
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

void CollisionWorldBt::addToManager(const World::Object* obj) const
{
  tesseract::CollisionObjectTypeVector collision_object_types;

  for (auto shape : obj->shapes_)
  {
    if (shape->type == shapes::MESH)
      collision_object_types.push_back(tesseract::CollisionObjectType::ConvexHull);
    else
      collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);
  }

  // TODO: Mask id -> 0 what exactly do with it
  bt_manager_.addCollisionObject(obj->id_, 0, obj->shapes_, obj->shape_poses_, collision_object_types, true);
}

void CollisionWorldBt::updateManagedObject(const std::string& id)
{
  // we have three cases: 1) the object is part of the manager and not of world --> delete it
  //                      2) the object is not in the manager, therefore register to manager,
  //                      3) the object is in the manager then delete and add the modified

  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (bt_manager_.hasCollisionObject(id))
    {
      bt_manager_.removeCollisionObject(id);
      addToManager(it->second.get());
    }
    else
    {
      addToManager(it->second.get());
    }
  }
  else
  {
    if (bt_manager_.hasCollisionObject(id))
    {
      bt_manager_.removeCollisionObject(id);
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
    bt_manager_.removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
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

bool CollisionWorldBt::allowedCollisionCheck(const std::string body_1, const std::string body_2) const
{
  collision_detection::AllowedCollision::Type allowed_type;

  if (acm_ != nullptr)
  {
    if (acm_->getEntry(body_1, body_2, allowed_type))
    {
      if (allowed_type == collision_detection::AllowedCollision::Type::NEVER)
      {
        ROS_DEBUG_STREAM("Not allowed entry in ACM found, collision check between " << body_1 << " and " << body_2);
        return false;
      }
      else
      {
        ROS_DEBUG_STREAM("Entry in ACM found, skipping collision check as allowed " << body_1 << " and " << body_2);
        return true;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("No entry in ACM found, collision check between " << body_1 << " and " << body_2);
      return false;
    }
  }
  else
  {
    ROS_DEBUG_STREAM("No ACM, collision check between " << body_1 << " and " << body_2);
    return false;
  }
}

}  // end of namespace collision_detection
