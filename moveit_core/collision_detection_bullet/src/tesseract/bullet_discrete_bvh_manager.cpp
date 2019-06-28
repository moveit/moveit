/**
 * @file bullet_discrete_bvh_manager.cpp
 * @brief Tesseract ROS Bullet Discrete BVH Manager implementation.
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

#include "moveit/collision_detection_bullet/tesseract/bullet_discrete_bvh_manager.h"

namespace tesseract
{
namespace tesseract_bullet
{
BulletDiscreteBVHManager::BulletDiscreteBVHManager()
{
  dispatcher_.reset(new btCollisionDispatcher(&coll_config_));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_.reset(new btDbvtBroadphase());

  contact_distance_ = 0;
}

BulletDiscreteBVHManager::~BulletDiscreteBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
  {
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
  }
}

DiscreteContactManagerBasePtr BulletDiscreteBVHManager::clone() const
{
  BulletDiscreteBVHManagerPtr manager(new BulletDiscreteBVHManager());

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

bool BulletDiscreteBVHManager::addCollisionObject(const std::string& name, const collision_detection::BodyType& mask_id,
                                                  const std::vector<shapes::ShapeConstPtr>& shapes,
                                                  const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                                  const std::vector<CollisionObjectType>& collision_object_types,
                                                  bool enabled)
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

bool BulletDiscreteBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletDiscreteBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    removeCollisionObjectFromBroadphase(it->second, broadphase_, dispatcher_);
    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool BulletDiscreteBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool BulletDiscreteBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COWPtr& cow = it->second;
    cow->setWorldTransform(convertEigenToBt(pose));

    // Update Collision Object Broadphase AABB
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const btTransform& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COWPtr& cow = it->second;
    cow->setWorldTransform(pose);

    // Update Collision Object Broadphase AABB
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                            const AlignedVector<Eigen::Isometry3d>& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(
    const AlignedMap<std::string, Eigen::Isometry3d>& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    updateCollisionObjectFilters(active_, *cow, false);
  }
}

const std::vector<std::string>& BulletDiscreteBVHManager::getActiveCollisionObjects() const
{
  return active_;
}
void BulletDiscreteBVHManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;

  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;
    cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
    assert(cow->getBroadphaseHandle() != nullptr);
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

double BulletDiscreteBVHManager::getContactDistanceThreshold() const
{
  return contact_distance_;
}
void BulletDiscreteBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn)
{
  fn_ = fn;
}
IsContactAllowedFn BulletDiscreteBVHManager::getIsContactAllowedFn() const
{
  return fn_;
}

void BulletDiscreteBVHManager::contactTest(collision_detection::CollisionResult& collisions,
                                           const collision_detection::CollisionRequest& req,
                                           const collision_detection::AllowedCollisionMatrix* acm)
{
  ContactTestData cdata(active_, contact_distance_, fn_, collisions, req, acm);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  DiscreteBroadphaseContactResultCallback cc(cdata, contact_distance_);

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());

  ROS_DEBUG_STREAM((collisions.collision ? "In" : "No") << " collision with " << collisions.contact_count << " collisio"
                                                                                                             "ns");
}

void BulletDiscreteBVHManager::contactTest(collision_detection::CollisionResult& collisions,
                                           const collision_detection::CollisionRequest& req,
                                           const collision_detection::AllowedCollisionMatrix* acm,
                                           const std::vector<tesseract::tesseract_bullet::COWPtr> cows_external)
{
  ContactTestData cdata(active_, contact_distance_, fn_, collisions, req, acm);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  DiscreteBroadphaseContactResultCallback cc(cdata, contact_distance_);

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());

  for (auto& cow : cows_external)
  {
    contactTest(cow, cdata);
  }

  ROS_DEBUG_STREAM((collisions.collision ? "In" : "No") << " collision with " << collisions.contact_count << " collisio"
                                                                                                             "ns");
}

void BulletDiscreteBVHManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  // Add collision object to broadphase
  addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
}

const Link2Cow& BulletDiscreteBVHManager::getCollisionObjects() const
{
  return link2cow_;
}

void BulletDiscreteBVHManager::contactTest(const COWPtr& cow, ContactTestData& collisions)
{
  btVector3 min_aabb, max_aabb;
  cow->getAABB(min_aabb, max_aabb);

  DiscreteCollisionCollector cc(collisions, cow, static_cast<double>(cow->getContactProcessingThreshold()));

  TesseractSingleContactCallback contactCB(cow.get(), dispatcher_.get(), dispatch_info_, cc);

  broadphase_->aabbTest(min_aabb, max_aabb, contactCB);
}
}
}
