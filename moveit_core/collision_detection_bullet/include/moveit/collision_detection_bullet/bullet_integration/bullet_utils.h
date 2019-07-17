/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * Copyright (c) 2013, John Schulman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
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
 *********************************************************************/

/* Authors: John Schulman, Levi Armstrong */

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BULLET_UTILS_H_
#define MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BULLET_UTILS_H_

#include <btBulletCollisionCommon.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>
#include <moveit/collision_detection_bullet/bullet_integration/contact_checker_common.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/macros/declare_ptr.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection_bullet
{
#define METERS

const btScalar BULLET_MARGIN = 0.0f;
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = 0.01f METERS;
const btScalar BULLET_LENGTH_TOLERANCE = 0.001f METERS;
const btScalar BULLET_EPSILON = 1e-3f;                   // numerical precision limit
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = 0.00f;  // All pairs closer than this distance get reported
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

MOVEIT_CLASS_FORWARD(CollisionObjectWrapper)

/** \brief Converts eigen vector to bullet vector */
inline btVector3 convertEigenToBt(const Eigen::Vector3d& v)
{
  return btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2]));
}

/** \brief Converts bullet vector to eigen vector */
inline Eigen::Vector3d convertBtToEigen(const btVector3& v)
{
  return Eigen::Vector3d(static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z()));
}

/** \brief Converts eigen quaternion to bullet quaternion */
inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
{
  return btQuaternion(static_cast<btScalar>(q.x()), static_cast<btScalar>(q.y()), static_cast<btScalar>(q.z()),
                      static_cast<btScalar>(q.w()));
}

/** \brief Converts eigen matrix to bullet matrix */
inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
{
  return btMatrix3x3(static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)), static_cast<btScalar>(r(0, 2)),
                     static_cast<btScalar>(r(1, 0)), static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
                     static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)), static_cast<btScalar>(r(2, 2)));
}

/** \brief Converts bullet transform to eigen transform */
inline btTransform convertEigenToBt(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix3d& rot = t.matrix().block<3, 3>(0, 0);
  const Eigen::Vector3d& tran = t.translation();

  btMatrix3x3 mat = convertEigenToBt(rot);
  btVector3 translation = convertEigenToBt(tran);

  return btTransform(mat, translation);
}

/** @brief Tesseract bullet collision object.
 *
 *  A wrapper around bullet's collision object which contains specific information related to bullet */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Standard constructor */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types);

  /** \brief Constructor for attached robot objects */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types,
                         const std::set<std::string>& touch_links);

  /** \brief Bitfield specifies to which group the object belongs */
  short int m_collision_filter_group;

  /** \brief Bitfield specifies against which other groups the object is collision checked */
  short int m_collision_filter_mask;

  /** \brief Indicates if the collision object is used for a collision check */
  bool m_enabled;

  /** \brief The robot links the collision objects is allowed to touch */
  std::set<std::string> m_touch_links;

  /** @brief Get the collision object name */
  const std::string& getName() const
  {
    return m_name;
  }

  /** @brief Get a user defined type */
  const collision_detection::BodyType& getTypeID() const
  {
    return m_type_id;
  }

  /** \brief Check if two CollisionObjectWrapper objects point to the same source object
   *  \return True if same objects, false otherwise */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_name == other.m_name && m_type_id == other.m_type_id && m_shapes.size() == other.m_shapes.size() &&
           m_shape_poses.size() == other.m_shape_poses.size() &&
           std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
           std::equal(m_shape_poses.begin(), m_shape_poses.end(), other.m_shape_poses.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  /** @brief Get the collision objects axis aligned bounding box
   *  @param aabb_min The minimum point
   *  @param aabb_max The maximum point */
  void getAABB(btVector3& aabb_min, btVector3& aabb_max) const
  {
    getCollisionShape()->getAabb(getWorldTransform(), aabb_min, aabb_max);
    const btScalar& d = getContactProcessingThreshold();
    btVector3 contact_threshold(d, d, d);
    aabb_min -= contact_threshold;
    aabb_max += contact_threshold;
  }

  /** @brief Clones the collision objects but not the collision shape wich is const.
   *  @return Shared Pointer to the cloned collision object */
  std::shared_ptr<CollisionObjectWrapper> clone()
  {
    std::shared_ptr<CollisionObjectWrapper> clone_cow(
        new CollisionObjectWrapper(m_name, m_type_id, m_shapes, m_shape_poses, m_collision_object_types, m_data));
    clone_cow->setCollisionShape(getCollisionShape());
    clone_cow->setWorldTransform(getWorldTransform());
    clone_cow->m_collision_filter_group = m_collision_filter_group;
    clone_cow->m_collision_filter_mask = m_collision_filter_mask;
    clone_cow->m_enabled = m_enabled;
    clone_cow->setBroadphaseHandle(nullptr);
    clone_cow->m_touch_links = m_touch_links;
    clone_cow->setContactProcessingThreshold(this->getContactProcessingThreshold());
    return clone_cow;
  }

  /** \brief Manage memory of a raw pointer shape */
  template <class T>
  void manage(T* t)
  {
    m_data.push_back(std::shared_ptr<T>(t));
  }

  /** \brief Manage memory of a shared pointer shape */
  template <class T>
  void manage(std::shared_ptr<T> t)
  {
    m_data.push_back(t);
  }

protected:
  /** @brief Special constructor used by the clone method */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types,
                         const std::vector<std::shared_ptr<void>>& data);

  std::string m_name;
  collision_detection::BodyType m_type_id;

  /** @brief The shapes that define the collison object */
  std::vector<shapes::ShapeConstPtr> m_shapes;

  /** @brief The poses of the shapes */
  AlignedVector<Eigen::Isometry3d> m_shape_poses;

  /** @brief The shape collision object type to be used */
  std::vector<CollisionObjectType> m_collision_object_types;

  /** @brief Manages the collision shape pointer so they get destroyed */
  std::vector<std::shared_ptr<void>> m_data;
};

/** \brief Computes the local supporting vertex of a convex shape.
 *
 *  If multiple vertices with equal support products exists, their average is calculated and returned.
 *
 *  \param shape The convex shape to check
 *  \param localNormal The support direction to search for in shape local coordinates
 *  \param outsupport The value of the calculated support mapping
 *  \param outpt The computed support point */
inline void getAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport,
                              btVector3& outpt)
{
  btVector3 pt_sum(0, 0, 0);
  float pt_count = 0;
  float max_support = -1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int n_pts = pshape->getNumVertices();

    for (int i = 0; i < n_pts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup = pt.dot(localNormal);
      if (sup > max_support + BULLET_EPSILON)
      {
        pt_count = 1;
        pt_sum = pt;
        max_support = sup;
      }
      else if (sup < max_support - BULLET_EPSILON)
      {
      }
      else
      {
        pt_count += 1;
        pt_sum += pt;
      }
    }
    outsupport = max_support;
    outpt = pt_sum / pt_count;
  }
  else
  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}

/** @brief Check if a collision check is required between the provided collision objects
 *  @param cow1 The first collision object
 *  @param cow2 The second collision object
 *  @param acm  The contact allowed function pointer
 *  @param verbose Indicate if verbose information should be printed
 *  @return True if the two collision objects should be checked for collision, otherwise false */
inline bool needsCollisionCheck(const CollisionObjectWrapper& cow1, const CollisionObjectWrapper& cow2,
                                const IsContactAllowedFn& allowed_fn,
                                const collision_detection::AllowedCollisionMatrix* acm, bool verbose = false)
{
  if (!cow1.m_enabled)
    return false;

  if (!cow2.m_enabled)
    return false;

  if (!((cow2.m_collision_filter_group & cow1.m_collision_filter_mask) &&
        (cow1.m_collision_filter_group & cow2.m_collision_filter_mask)))
    return false;

  if (isContactAllowed(cow1.getName(), cow2.getName(), allowed_fn, acm, verbose))
    return false;

  if (cow1.getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cow2.getTypeID() == collision_detection::BodyType::ROBOT_LINK)
    if (cow1.m_touch_links.find(cow2.getName()) != cow1.m_touch_links.end())
      return false;

  if (cow2.getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
      cow1.getTypeID() == collision_detection::BodyType::ROBOT_LINK)
    if (cow2.m_touch_links.find(cow1.getName()) == cow2.m_touch_links.end())
      return false;

  // TODO: Add check for two objects attached to the same link
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Checking collision btw " << cow1.getName() << " vs "
                                                                                 << cow2.getName());

  return true;
}

/** \brief Converts a bullet contact result to MoveIt format and adds it to the result data structure */
inline btScalar addDiscreteSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
                                        const btCollisionObjectWrapper* colObj1Wrap, ContactTestData& collisions)
{
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);
  const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

  std::pair<std::string, std::string> pc = getObjectPairKey(cd0->getName(), cd1->getName());

  const auto& it = collisions.res.contacts.find(pc);
  bool found = (it != collisions.res.contacts.end());

  collision_detection::Contact contact;
  contact.body_name_1 = cd0->getName();
  contact.body_name_2 = cd1->getName();
  contact.depth = static_cast<double>(cp.m_distance1);
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);
  contact.pos = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[0] = contact.pos;
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);

  contact.body_type_1 = cd0->getTypeID();
  contact.body_type_2 = cd0->getTypeID();

  if (!processResult(collisions, contact, pc, found))
  {
    return 0;
  }

  return 1;
}

/** \brief Processes a contact point */
struct TesseractBridgedManifoldResult : public btManifoldResult
{
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap, const btCollisionObjectWrapper* obj1Wrap,
                                 btCollisionWorld::ContactResultCallback& resultCallback)
    : btManifoldResult(obj0Wrap, obj1Wrap), m_resultCallback(resultCallback)
  {
  }

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override
  {
    bool is_swapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 point_a = pointInWorld + normalOnBInWorld * depth;
    btVector3 local_a;
    btVector3 local_b;
    if (is_swapped)
    {
      local_a = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      local_a = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint new_pt(local_a, local_b, normalOnBInWorld, depth);
    new_pt.m_positionWorldOnA = point_a;
    new_pt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (is_swapped)
    {
      new_pt.m_partId0 = m_partId1;
      new_pt.m_partId1 = m_partId0;
      new_pt.m_index0 = m_index1;
      new_pt.m_index1 = m_index0;
    }
    else
    {
      new_pt.m_partId0 = m_partId0;
      new_pt.m_partId1 = m_partId1;
      new_pt.m_index0 = m_index0;
      new_pt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0_wrap = is_swapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1_wrap = is_swapped ? m_body0Wrap : m_body1Wrap;
    m_resultCallback.addSingleResult(new_pt, obj0_wrap, new_pt.m_partId0, new_pt.m_index0, obj1_wrap, new_pt.m_partId1,
                                     new_pt.m_index1);
  }
};

/** @brief Abstract interface for both discrete and continuous reporting of contact points */
struct BroadphaseContactResultCallback
{
  ContactTestData& collisions_;
  double contact_distance_;
  bool verbose_;

  BroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false)
    : collisions_(collisions), contact_distance_(contact_distance), verbose_(verbose)
  {
  }

  virtual ~BroadphaseContactResultCallback() = default;

  virtual bool needsCollision(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) const
  {
    return !collisions_.done && needsCollisionCheck(*cow0, *cow1, collisions_.fn, collisions_.acm, verbose_);
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0,
                                   int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1,
                                   int index1) = 0;
};

/** \brief addSingleResult of this struct is called each time the broadphase check indicates a collision. */
struct DiscreteBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  DiscreteBroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false)
    : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
  {
  }

  btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                           int /*index0*/, const btCollisionObjectWrapper* colObj1Wrap, int /*partId1*/,
                           int /*index1*/) override
  {
    if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not close enough for collision with " << cp.m_distance1);
      return 0;
    }

    return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
  }
};

/** \brief Processes a contact point */
struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
  BroadphaseContactResultCallback& result_callback_;

  TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                           const btCollisionObjectWrapper* obj1Wrap,
                                           BroadphaseContactResultCallback& result_callback)
    : btManifoldResult(obj0Wrap, obj1Wrap), result_callback_(result_callback)
  {
  }

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override
  {
    if (result_callback_.collisions_.done || result_callback_.collisions_.pair_done ||
        depth > static_cast<btScalar>(result_callback_.contact_distance_))
    {
      return;
    }

    bool is_swapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 point_a = pointInWorld + normalOnBInWorld * depth;
    btVector3 local_a;
    btVector3 local_b;
    if (is_swapped)
    {
      local_a = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      local_a = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint new_pt(local_a, local_b, normalOnBInWorld, depth);
    new_pt.m_positionWorldOnA = point_a;
    new_pt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (is_swapped)
    {
      new_pt.m_partId0 = m_partId1;
      new_pt.m_partId1 = m_partId0;
      new_pt.m_index0 = m_index1;
      new_pt.m_index1 = m_index0;
    }
    else
    {
      new_pt.m_partId0 = m_partId0;
      new_pt.m_partId1 = m_partId1;
      new_pt.m_index0 = m_index0;
      new_pt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0_wrap = is_swapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1_wrap = is_swapped ? m_body0Wrap : m_body1Wrap;
    result_callback_.addSingleResult(new_pt, obj0_wrap, new_pt.m_partId0, new_pt.m_index0, obj1_wrap, new_pt.m_partId1,
                                     new_pt.m_index1);
  }
};

/** @brief Check a collision object not in the broadphase to the broadphase which may eventually be exposed.
 *
 *  This is copied directly out of BulletWorld */
struct TesseractSingleContactCallback : public btBroadphaseAabbCallback
{
  btCollisionObject* m_collisionObject;    /**< @brief The bullet collision object */
  btCollisionDispatcher* m_dispatcher;     /**< @brief The bullet collision dispatcher used for getting object to object
                                              collison algorithm */
  const btDispatcherInfo& m_dispatch_info; /**< @brief The bullet collision dispatcher configuration information */
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractSingleContactCallback(btCollisionObject* collisionObject, btCollisionDispatcher* dispatcher,
                                 const btDispatcherInfo& dispatch_info,
                                 btCollisionWorld::ContactResultCallback& resultCallback)
    : m_collisionObject(collisionObject)
    , m_dispatcher(dispatcher)
    , m_dispatch_info(dispatch_info)
    , m_resultCallback(resultCallback)
  {
  }

  bool process(const btBroadphaseProxy* proxy) override
  {
    btCollisionObject* collision_object = static_cast<btCollisionObject*>(proxy->m_clientObject);
    if (collision_object == m_collisionObject)
      return true;

    if (m_resultCallback.needsCollision(collision_object->getBroadphaseHandle()))
    {
      btCollisionObjectWrapper ob0(nullptr, m_collisionObject->getCollisionShape(), m_collisionObject,
                                   m_collisionObject->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper ob1(nullptr, collision_object->getCollisionShape(), collision_object,
                                   collision_object->getWorldTransform(), -1, -1);

      btCollisionAlgorithm* algorithm = m_dispatcher->findAlgorithm(&ob0, &ob1, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
      if (algorithm)
      {
        TesseractBridgedManifoldResult contact_point_result(&ob0, &ob1, m_resultCallback);
        contact_point_result.m_closestPointDistanceThreshold = m_resultCallback.m_closestDistanceThreshold;

        // discrete collision detection query
        algorithm->processCollision(&ob0, &ob1, m_dispatch_info, &contact_point_result);

        algorithm->~btCollisionAlgorithm();
        m_dispatcher->freeCollisionAlgorithm(algorithm);
      }
    }
    return true;
  }
};

/** @brief A callback function that is called as part of the broadphase collision checking.
 *
 *  If the AABB of two collision objects are overlapping the processOverlap method is called and they are checked for
 *  collision/distance and the results are stored in collision_. */
class TesseractCollisionPairCallback : public btOverlapCallback
{
  const btDispatcherInfo& dispatch_info_;
  btCollisionDispatcher* dispatcher_;
  BroadphaseContactResultCallback& results_callback_;

public:
  TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo, btCollisionDispatcher* dispatcher,
                                 BroadphaseContactResultCallback& results_callback)
    : dispatch_info_(dispatchInfo), dispatcher_(dispatcher), results_callback_(results_callback)
  {
  }

  ~TesseractCollisionPairCallback() override = default;

  bool processOverlap(btBroadphasePair& pair) override
  {
    results_callback_.collisions_.pair_done = false;

    if (results_callback_.collisions_.done)
    {
      return false;
    }

    const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
    const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

    std::pair<std::string, std::string> pair_names{ cow0->getName(), cow1->getName() };
    ROS_DEBUG_STREAM("Processing " << cow0->getName() << " vs " << cow1->getName());

    if (results_callback_.needsCollision(cow0, cow1))
    {
      btCollisionObjectWrapper obj0_wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper obj1_wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

      // dispatcher will keep algorithms persistent in the collision pair
      if (!pair.m_algorithm)
      {
        pair.m_algorithm = dispatcher_->findAlgorithm(&obj0_wrap, &obj1_wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
      }

      if (pair.m_algorithm)
      {
        TesseractBroadphaseBridgedManifoldResult contact_point_result(&obj0_wrap, &obj1_wrap, results_callback_);
        contact_point_result.m_closestPointDistanceThreshold =
            static_cast<btScalar>(results_callback_.contact_distance_);

        // discrete collision detection query
        pair.m_algorithm->processCollision(&obj0_wrap, &obj1_wrap, dispatch_info_, &contact_point_result);
      }
    }
    return false;
  }
};

/** \brief Casts a geometric shape into a btCollisionShape */
btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type, CollisionObjectWrapper* cow);

/** @brief Update a collision objects filters
 *  @param active A list of active collision objects
 *  @param cow The collision object to update.
 *  @param continuous Indicate if the object is a continuous collision object.
 *
 *  Currently continuous collision objects can only be checked against static objects. Continuous to Continuous
 *  collision checking is currently not supports. TODO LEVI: Add support for Continuous to Continuous collision
 *  checking. */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active, CollisionObjectWrapper& cow,
                                         bool continuous)
{
  cow.m_collision_filter_group = btBroadphaseProxy::KinematicFilter;

  if (!isLinkActive(active, cow.getName()))
  {
    cow.m_collision_filter_group = btBroadphaseProxy::StaticFilter;
  }

  if (cow.m_collision_filter_group == btBroadphaseProxy::StaticFilter)
  {
    cow.m_collision_filter_mask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    (continuous) ? (cow.m_collision_filter_mask = btBroadphaseProxy::StaticFilter) :
                   (cow.m_collision_filter_mask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
  }

  if (cow.getBroadphaseHandle())
  {
    cow.getBroadphaseHandle()->m_collisionFilterGroup = cow.m_collision_filter_group;
    cow.getBroadphaseHandle()->m_collisionFilterMask = cow.m_collision_filter_mask;
  }
}

/** \brief Wrapper around constructing a CollisionObjectWrapper */
inline CollisionObjectWrapperPtr createCollisionObject(const std::string& name,
                                                       const collision_detection::BodyType& type_id,
                                                       const std::vector<shapes::ShapeConstPtr>& shapes,
                                                       const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                                       const std::vector<CollisionObjectType>& collision_object_types,
                                                       bool enabled = true)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    ROS_DEBUG_NAMED("collision_detection.bullet", "ignoring link %s", name.c_str());
    return nullptr;
  }

  CollisionObjectWrapperPtr new_cow(
      new CollisionObjectWrapper(name, type_id, shapes, shape_poses, collision_object_types));

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  ROS_DEBUG_NAMED("collision_detection.bullet", "Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

// TODO: Unify with other createCollisionObject functions
/** \brief Wrapper around constructing a CollisionObjectWrapper for attached objects */
inline CollisionObjectWrapperPtr createCollisionObject(const std::string& name,
                                                       const collision_detection::BodyType& type_id,
                                                       const std::vector<shapes::ShapeConstPtr>& shapes,
                                                       const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                                       const std::vector<CollisionObjectType>& collision_object_types,
                                                       const std::set<std::string>& touch_links, bool enabled = true)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    ROS_DEBUG_NAMED("collision_detection.bullet", "ignoring link %s", name.c_str());
    return nullptr;
  }

  CollisionObjectWrapperPtr new_cow(
      new CollisionObjectWrapper(name, type_id, shapes, shape_poses, collision_object_types, touch_links));

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  ROS_DEBUG_NAMED("collision_detection.bullet", "Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

/** \brief Processes a contact after positive broadphase check */
struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;
  const CollisionObjectWrapperPtr cow_;
  double contact_distance_;
  bool verbose_;

  DiscreteCollisionCollector(ContactTestData& collisions, const CollisionObjectWrapperPtr& cow, double contact_distance,
                             bool verbose = false)
    : collisions_(collisions), cow_(cow), contact_distance_(contact_distance), verbose_(verbose)
  {
    m_closestDistanceThreshold = static_cast<btScalar>(contact_distance);
    m_collisionFilterGroup = cow->m_collision_filter_group;
    m_collisionFilterMask = cow->m_collision_filter_mask;
  }

  btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                           int /*index0*/, const btCollisionObjectWrapper* colObj1Wrap, int /*partId1*/,
                           int /*index1*/) override
  {
    if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not close enough for collision with " << cp.m_distance1);
      return 0;
    }

    return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const override
  {
    return !collisions_.done &&
           needsCollisionCheck(*cow_, *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)), collisions_.fn,
                               collisions_.acm, verbose_);
  }
};

/** @brief Update the Broadphase AABB for the input collision object
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void updateBroadphaseAABB(const CollisionObjectWrapperPtr& cow,
                                 const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                 const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  // Update the broadphase aabb
  assert(cow->getBroadphaseHandle() != nullptr);
  broadphase->setAabb(cow->getBroadphaseHandle(), aabb_min, aabb_max, dispatcher.get());
}

/** @brief Remove the collision object from broadphase
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void removeCollisionObjectFromBroadphase(const CollisionObjectWrapperPtr& cow,
                                                const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                                const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btBroadphaseProxy* bp = cow->getBroadphaseHandle();
  if (bp)
  {
    // only clear the cached algorithms
    broadphase->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher.get());
    broadphase->destroyProxy(bp, dispatcher.get());
    cow->setBroadphaseHandle(nullptr);
  }
}

/** @brief Add the collision object to broadphase
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void addCollisionObjectToBroadphase(const CollisionObjectWrapperPtr& cow,
                                           const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                           const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Added " << cow->getName() << " to broadphase");
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  // Add the active collision object to the broadphase
  int type = cow->getCollisionShape()->getShapeType();
  cow->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, cow.get(), cow->m_collision_filter_group,
                                                   cow->m_collision_filter_mask, dispatcher.get()));
}
}  // namespace collision_detection_bullet
#endif  //  MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BULLET_UTILS_H_
