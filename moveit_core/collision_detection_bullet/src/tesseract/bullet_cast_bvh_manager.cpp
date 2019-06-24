/**
 * @file bullet_cast_bvh_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) BVH Manager implementation.
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

#include "moveit/collision_detection_bullet/tesseract/bullet_cast_bvh_manager.h"

namespace tesseract
{
namespace tesseract_bullet
{
BulletCastBVHManager::BulletCastBVHManager()
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

BulletCastBVHManager::~BulletCastBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);

  // clean up remaining objects
  for (auto& co : link2castcow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
}

ContinuousContactManagerBasePtr BulletCastBVHManager::clone() const
{
  BulletCastBVHManagerPtr manager(new BulletCastBVHManager());

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

bool BulletCastBVHManager::addCollisionObject(const std::string& name, const collision_detection::BodyType& mask_id,
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

bool BulletCastBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletCastBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COWPtr& cow1 = it->second;
    removeCollisionObjectFromBroadphase(cow1, broadphase_, dispatcher_);
    link2cow_.erase(name);

    COWPtr& cow2 = link2castcow_[name];
    removeCollisionObjectFromBroadphase(cow2, broadphase_, dispatcher_);
    link2castcow_.erase(name);

    return true;
  }

  return false;
}

bool BulletCastBVHManager::enableCollisionObject(const std::string& name)
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

bool BulletCastBVHManager::disableCollisionObject(const std::string& name)
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

void BulletCastBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COWPtr& cow = it->second;
    btTransform tf = convertEigenToBt(pose);
    cow->setWorldTransform(tf);
    link2castcow_[name]->setWorldTransform(tf);

    // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const AlignedVector<Eigen::Isometry3d>& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const AlignedMap<std::string, Eigen::Isometry3d>& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose1,
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
        throw std::runtime_error(
            "I can only continuous collision check convex shapes and compound shapes made of convex "
            "shapes");
      }

      // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
    }
  }
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const AlignedVector<Eigen::Isometry3d>& pose1,
                                                        const AlignedVector<Eigen::Isometry3d>& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const AlignedMap<std::string, Eigen::Isometry3d>& pose1,
                                                        const AlignedMap<std::string, Eigen::Isometry3d>& pose2)
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

void BulletCastBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    // Need to check if a collision object is still active
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      // Update with active
      updateCollisionObjectFilters(active_, *cow, false);

      // Get the active collision object
      COWPtr& active_cow = link2castcow_[cow->getName()];

      // Update with active
      updateCollisionObjectFilters(active_, *active_cow, true);

      // Check if the link is still active.
      if (!isLinkActive(active_, cow->getName()))
      {
        // Remove the active collision object from the broadphase
        removeCollisionObjectFromBroadphase(active_cow, broadphase_, dispatcher_);

        // Add the active collision object to the broadphase
        addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
      }
    }
    else
    {
      // Update with active
      updateCollisionObjectFilters(active_, *cow, false);

      // Get the active collision object
      COWPtr& active_cow = link2castcow_[cow->getName()];

      // Update with active
      updateCollisionObjectFilters(active_, *active_cow, true);

      // Check if link is now active
      if (isLinkActive(active_, cow->getName()))
      {
        // Remove the static collision object from the broadphase
        removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);

        // Add the active collision object to the broadphase
        addCollisionObjectToBroadphase(active_cow, broadphase_, dispatcher_);
      }
    }
  }
}

const std::vector<std::string>& BulletCastBVHManager::getActiveCollisionObjects() const
{
  return active_;
}

void BulletCastBVHManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;

  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;
    cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }

  for (auto& co : link2castcow_)
  {
    COWPtr& cow = co.second;
    cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

double BulletCastBVHManager::getContactDistanceThreshold() const
{
  return contact_distance_;
}

void BulletCastBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn)
{
  fn_ = fn;
}

IsContactAllowedFn BulletCastBVHManager::getIsContactAllowedFn() const
{
  return fn_;
}

void BulletCastBVHManager::contactTest(collision_detection::CollisionResult& collisions,
                                       const collision_detection::CollisionRequest& req,
                                       const collision_detection::AllowedCollisionMatrix* acm)
{
  ContactTestData cdata(active_, contact_distance_, fn_, collisions, req, acm);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  CastBroadphaseContactResultCallback cc(cdata, contact_distance_);

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletCastBVHManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  // Create cast collision object
  COWPtr cast_cow = makeCastCollisionObject(cow);

  // Add it to the cast map
  link2castcow_[cast_cow->getName()] = cast_cow;

  const COWPtr& selected_cow = (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter) ? cast_cow : cow;

  btVector3 aabb_min, aabb_max;
  selected_cow->getAABB(aabb_min, aabb_max);

  int type = selected_cow->getCollisionShape()->getShapeType();
  selected_cow->setBroadphaseHandle(broadphase_->createProxy(aabb_min, aabb_max, type, selected_cow.get(),
                                                             selected_cow->m_collisionFilterGroup,
                                                             selected_cow->m_collisionFilterMask, dispatcher_.get()));
}

void BulletCastBVHManager::contactTest(const COWPtr& cow, ContactTestData& collisions)
{
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  CastCollisionCollector cc(collisions, cow, static_cast<double>(cow->getContactProcessingThreshold()));

  TesseractSingleContactCallback contactCB(cow.get(), dispatcher_.get(), dispatch_info_, cc);

  broadphase_->aabbTest(aabb_min, aabb_max, contactCB);
}
}
}
