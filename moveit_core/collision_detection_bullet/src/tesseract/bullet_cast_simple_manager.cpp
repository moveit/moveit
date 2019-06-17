/**
 * @file bullet_cast_simple_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) Simple Manager implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "moveit/collision_detection_bullet/tesseract/bullet_cast_simple_manager.h"

namespace tesseract
{
namespace tesseract_bullet
{
BulletCastSimpleManager::BulletCastSimpleManager()
{
  dispatcher_.reset(new btCollisionDispatcher(&coll_config_));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  contact_distance_ = 0;
}

ContinuousContactManagerBasePtr BulletCastSimpleManager::clone() const
{
  BulletCastSimpleManagerPtr manager(new BulletCastSimpleManager());

  for (const auto& cow : link2cow_)
  {
    COWPtr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(cow.second->getWorldTransform());

    new_cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance_));
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(contact_distance_);
  manager->setIsContactAllowedFn(fn_);

  return manager;
}

bool BulletCastSimpleManager::addCollisionObject(const std::string& name, const int& mask_id,
                                                 const std::vector<shapes::ShapeConstPtr>& shapes,
                                                 const VectorIsometry3d& shape_poses,
                                                 const CollisionObjectTypeVector& collision_object_types, bool enabled)
{
  COWPtr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, collision_object_types, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }
  else
  {
    return false;
  }
}

bool BulletCastSimpleManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletCastSimpleManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    cows_.erase(std::find(cows_.begin(), cows_.end(), it->second));
    link2cow_.erase(name);
    link2castcow_.erase(name);
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    link2castcow_[name]->m_enabled = true;
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    link2castcow_[name]->m_enabled = false;
    return true;
  }

  return false;
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    btTransform tf = convertEigenToBt(pose);
    it->second->setWorldTransform(tf);
    link2castcow_[name]->setWorldTransform(tf);
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                           const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose1,
                                                           const Eigen::Isometry3d& pose2)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2castcow_.find(name);
  if (it != link2castcow_.end())
  {
    COWPtr& cow = it->second;
    assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    cow->setWorldTransform(tf1);
    link2cow_[name]->setWorldTransform(tf1);

    // If collision object is disabled dont proceed
    if (cow->m_enabled)
    {
      if (btBroadphaseProxy::isConvex(cow->getCollisionShape()->getShapeType()))
      {
        assert(dynamic_cast<CastHullShape*>(cow->getCollisionShape()) != nullptr);
        static_cast<CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
      }
      else if (btBroadphaseProxy::isCompound(cow->getCollisionShape()->getShapeType()))
      {
        assert(dynamic_cast<btCompoundShape*>(cow->getCollisionShape()) != nullptr);
        btCompoundShape* compound = static_cast<btCompoundShape*>(cow->getCollisionShape());
        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
          {
            assert(dynamic_cast<CastHullShape*>(compound->getChildShape(i)) != nullptr);
            const btTransform& local_tf = compound->getChildTransform(i);

            btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
            static_cast<CastHullShape*>(compound->getChildShape(i))->updateCastTransform(delta_tf);
            compound->updateChildTransform(i, local_tf, false);  // This is required to update the BVH tree
          }
          else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
          {
            assert(dynamic_cast<btCompoundShape*>(compound->getChildShape(i)) != nullptr);
            btCompoundShape* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));

            for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
            {
              assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));
              assert(dynamic_cast<CastHullShape*>(second_compound->getChildShape(j)) != nullptr);
              const btTransform& local_tf = second_compound->getChildTransform(j);

              btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
              static_cast<CastHullShape*>(second_compound->getChildShape(j))->updateCastTransform(delta_tf);
              second_compound->updateChildTransform(j, local_tf, false);  // This is required to update the BVH tree
            }
            second_compound->recalculateLocalAabb();
          }
        }
        compound->recalculateLocalAabb();
      }
      else
      {
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                           const VectorIsometry3d& pose1, const VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const TransformMap& pose1, const TransformMap& pose2)
{
  assert(pose1.size() == pose2.size());
  auto it1 = pose1.begin();
  auto it2 = pose2.begin();
  while (it1 != pose1.end())
  {
    assert(pose1.find(it1->first) != pose2.end());
    setCollisionObjectsTransform(it1->first, it1->second, it2->second);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
}

void BulletCastSimpleManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;
  cows_.clear();
  cows_.reserve(link2cow_.size());

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    // Update with request
    updateCollisionObjectFilters(active_, *cow, false);

    // Get the cast collision object
    COWPtr cast_cow = link2castcow_[cow->getName()];

    // Update with request
    updateCollisionObjectFilters(active_, *cast_cow, true);

    // Add to collision object vector
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      cows_.insert(cows_.begin(), cast_cow);
    }
    else
    {
      cows_.push_back(cow);
    }
  }
}

const std::vector<std::string>& BulletCastSimpleManager::getActiveCollisionObjects() const
{
  return active_;
}
void BulletCastSimpleManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;

  for (auto& co : link2cow_)
    co.second->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));

  for (auto& co : link2castcow_)
    co.second->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
}

double BulletCastSimpleManager::getContactDistanceThreshold() const
{
  return contact_distance_;
}
void BulletCastSimpleManager::setIsContactAllowedFn(IsContactAllowedFn fn)
{
  fn_ = fn;
}
IsContactAllowedFn BulletCastSimpleManager::getIsContactAllowedFn() const
{
  return fn_;
}
void BulletCastSimpleManager::contactTest(collision_detection::CollisionResult& collisions, const ContactTestType& type, const collision_detection::CollisionRequest& req)
{
  ContactTestData cdata(active_, contact_distance_, fn_, type, collisions);

  for (auto cow1_iter = cows_.begin(); cow1_iter != (cows_.end() - 1); cow1_iter++)
  {
    const COWPtr& cow1 = *cow1_iter;

    if (cow1->m_collisionFilterGroup != btBroadphaseProxy::KinematicFilter)
      break;

    if (!cow1->m_enabled)
      continue;

    btVector3 min_aabb[2], max_aabb[2];
    cow1->getAABB(min_aabb[0], max_aabb[0]);

    btCollisionObjectWrapper obA(nullptr, cow1->getCollisionShape(), cow1.get(), cow1->getWorldTransform(), -1, -1);

    CastCollisionCollector cc(cdata, cow1, static_cast<double>(cow1->getContactProcessingThreshold()));
    for (auto cow2_iter = cow1_iter + 1; cow2_iter != cows_.end(); cow2_iter++)
    {
      assert(!cdata.done);

      const COWPtr& cow2 = *cow2_iter;
      cow2->getAABB(min_aabb[1], max_aabb[1]);

      bool aabb_check = (min_aabb[0][0] <= max_aabb[1][0] && max_aabb[0][0] >= min_aabb[1][0]) &&
                        (min_aabb[0][1] <= max_aabb[1][1] && max_aabb[0][1] >= min_aabb[1][1]) &&
                        (min_aabb[0][2] <= max_aabb[1][2] && max_aabb[0][2] >= min_aabb[1][2]);

      if (aabb_check)
      {
        bool needs_collision = needsCollisionCheck(*cow1, *cow2, fn_, false);

        if (needs_collision)
        {
          btCollisionObjectWrapper obB(nullptr, cow2->getCollisionShape(), cow2.get(), cow2->getWorldTransform(), -1,
                                       -1);

          btCollisionAlgorithm* algorithm =
              dispatcher_->findAlgorithm(&obA, &obB, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
          assert(algorithm != nullptr);
          if (algorithm)
          {
            TesseractBridgedManifoldResult contactPointResult(&obA, &obB, cc);
            contactPointResult.m_closestPointDistanceThreshold = cc.m_closestDistanceThreshold;

            // discrete collision detection query
            algorithm->processCollision(&obA, &obB, dispatch_info_, &contactPointResult);

            algorithm->~btCollisionAlgorithm();
            dispatcher_->freeCollisionAlgorithm(algorithm);
          }
        }
      }

      if (cdata.done)
        break;
    }

    if (cdata.done)
      break;
  }
}

void BulletCastSimpleManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  // Create cast collision object
  COWPtr cast_cow = makeCastCollisionObject(cow);

  // Add it to the cast map
  link2castcow_[cast_cow->getName()] = cast_cow;

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cast_cow);
  else
    cows_.push_back(cow);
}
}
}
