/**
 * @file bullet_utils.h
 * @brief Tesseract ROS Bullet environment utility function.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
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
#ifndef TESSERACT_COLLISION_BULLET_UTILS_H
#define TESSERACT_COLLISION_BULLET_UTILS_H

#include <btBulletCollisionCommon.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection_bullet/tesseract/basic_types.h>
#include <moveit/collision_detection_bullet/tesseract/contact_checker_common.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/macros/declare_ptr.h>
#include <moveit/macros/class_forward.h>

namespace tesseract
{
#define METERS

const btScalar BULLET_MARGIN = 0.0f;
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = 0.01f METERS;
const btScalar BULLET_LENGTH_TOLERANCE = 0.001f METERS;
const btScalar BULLET_EPSILON = 1e-3f;
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = 0.00f;  // All pairs closer than this distance get reported
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

MOVEIT_CLASS_FORWARD(CollisionObjectWrapper)
typedef CollisionObjectWrapper COW;
MOVEIT_DECLARE_PTR(COW, CollisionObjectWrapper)

inline btVector3 convertEigenToBt(const Eigen::Vector3d& v)
{
  return btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2]));
}

inline Eigen::Vector3d convertBtToEigen(const btVector3& v)
{
  return Eigen::Vector3d(static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z()));
}

inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
{
  return btQuaternion(static_cast<btScalar>(q.x()), static_cast<btScalar>(q.y()), static_cast<btScalar>(q.z()),
                      static_cast<btScalar>(q.w()));
}

inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
{
  return btMatrix3x3(static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)), static_cast<btScalar>(r(0, 2)),
                     static_cast<btScalar>(r(1, 0)), static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
                     static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)), static_cast<btScalar>(r(2, 2)));
}

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
 *  It is a wrapper around bullet's collision object which contains specific information related to tesseract */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types);

  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types,
                         const std::set<std::string>& touch_links);

  /** \brief Bitfield specifies to which group the object belongs */
  short int m_collisionFilterGroup;

  /** \brief Bitfield specifies against which other groups the object is collision checked */
  short int m_collisionFilterMask;

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

  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
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
    btVector3 contactThreshold(d, d, d);
    aabb_min -= contactThreshold;
    aabb_max += contactThreshold;
  }

  /** @brief Clones the collision objects but not the collision shape wich is const.
   *  @return Shared Pointer to the cloned collision object */
  std::shared_ptr<CollisionObjectWrapper> clone()
  {
    std::shared_ptr<CollisionObjectWrapper> clone_cow(
        new CollisionObjectWrapper(m_name, m_type_id, m_shapes, m_shape_poses, m_collision_object_types, m_data));
    clone_cow->setCollisionShape(getCollisionShape());
    clone_cow->setWorldTransform(getWorldTransform());
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    clone_cow->setBroadphaseHandle(nullptr);
    clone_cow->m_touch_links = m_touch_links;
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

  /**< @brief The shapes that define the collison object */
  std::vector<shapes::ShapeConstPtr> m_shapes;

  /**< @brief The poses of the shapes */
  AlignedVector<Eigen::Isometry3d> m_shape_poses;

  /**< @brief The shape collision object type to be used */
  std::vector<CollisionObjectType> m_collision_object_types;

  /**< @brief Manages the collision shape pointer so they get destroyed */
  std::vector<std::shared_ptr<void>> m_data;
};

/** @brief Casted collision shape used for checking if an object is collision free between two transforms */
struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_shape_transform;

  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_shape_transform(t01)
  {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  }

  void updateCastTransform(const btTransform& t01)
  {
    m_shape_transform = t01;
  }

  btVector3 localGetSupportingVertex(const btVector3& vec) const override
  {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_shape_transform * m_shape->localGetSupportingVertex(vec * m_shape_transform.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }

  // notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* /*vectors*/,
                                                         btVector3* /*supportVerticesOut*/,
                                                         int /*numVectors*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  /// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const override
  {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0 * m_shape_transform, min1, max1);
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  void getAabbSlow(const btTransform& /*t*/, btVector3& /*aabbMin*/, btVector3& /*aabbMax*/) const override
  {
    throw std::runtime_error("shouldn't happen");
  }

  void setLocalScaling(const btVector3& /*scaling*/) override
  {
  }

  const btVector3& getLocalScaling() const override
  {
    static btVector3 out(1, 1, 1);
    return out;
  }

  void setMargin(btScalar /*margin*/) override
  {
  }

  btScalar getMargin() const override
  {
    return 0;
  }

  int getNumPreferredPenetrationDirections() const override
  {
    return 0;
  }

  void getPreferredPenetrationDirection(int /*index*/, btVector3& /*penetrationVector*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  void calculateLocalInertia(btScalar, btVector3&) const override
  {
    throw std::runtime_error("not implemented");
  }

  const char* getName() const override
  {
    return "CastHull";
  }
  btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const override
  {
    return localGetSupportingVertex(v);
  }
};

/** \brief Computes the local supporting vertex of a convex shape.
 *
 *  If multiple vertices with equal support products exists, their average is calculated and returned.
 *
 *  \param shape The convex shape to check
 *  \param localNormal The support direction to search for in shape local coordinates
 *  \param outsupport The value of the calculated support mapping
 *  \param outpt The computed support point */
inline void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport,
                              btVector3& outpt)
{
  btVector3 ptSum(0, 0, 0);
  float ptCount = 0;
  float maxSupport = -1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int nPts = pshape->getNumVertices();

    for (int i = 0; i < nPts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup = pt.dot(localNormal);
      if (sup > maxSupport + BULLET_EPSILON)
      {
        ptCount = 1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - BULLET_EPSILON)
      {
      }
      else
      {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
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
inline bool needsCollisionCheck(const COW& cow1, const COW& cow2, const IsContactAllowedFn allowed_fn,
                                const collision_detection::AllowedCollisionMatrix* acm, bool verbose = false)
{
  if (!cow1.m_enabled)
    return false;

  if (!cow2.m_enabled)
    return false;

  if (!((cow2.m_collisionFilterGroup & cow1.m_collisionFilterMask) &&
        (cow1.m_collisionFilterGroup & cow2.m_collisionFilterMask)))
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

inline btScalar addCastSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*index0*/,
                                    const btCollisionObjectWrapper* colObj1Wrap, int /*index1*/,
                                    ContactTestData& collisions)
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

  contact.body_type_1 = cd0->getTypeID();
  contact.body_type_2 = cd0->getTypeID();

  collision_detection::Contact* col = processResult(collisions, contact, pc, found);

  if (!col)
  {
    return 0;
  }

  assert(!((cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter) &&
           (cd1->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)));

  bool castShapeIsFirst = cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter;

  btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
  const btCollisionObjectWrapper* firstColObjWrap = (castShapeIsFirst ? colObj0Wrap : colObj1Wrap);

  // we want the contact information of the non-casted object come first, therefore swap values
  if (castShapeIsFirst)
  {
    std::swap(col->nearest_points[0], col->nearest_points[1]);
    contact.pos = convertBtToEigen(cp.m_positionWorldOnB);
    std::swap(col->body_name_1, col->body_name_2);
    std::swap(col->body_type_1, col->body_type_2);
    col->normal *= -1;
  }

  btTransform tfWorld0, tfWorld1;
  const CastHullShape* shape = static_cast<const CastHullShape*>(firstColObjWrap->getCollisionShape());

  tfWorld0 = firstColObjWrap->getWorldTransform();
  tfWorld1 = firstColObjWrap->getWorldTransform() * shape->m_shape_transform;

  // transform normals into pose local coordinate systems
  btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
  btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

  btVector3 ptLocal0;
  float localsup0;
  GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
  btVector3 ptWorld0 = tfWorld0 * ptLocal0;
  btVector3 ptLocal1;
  float localsup1;
  GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
  btVector3 ptWorld1 = tfWorld1 * ptLocal1;

  float sup0 = normalWorldFromCast.dot(ptWorld0);
  float sup1 = normalWorldFromCast.dot(ptWorld1);

  // TODO: this section is potentially problematic. think hard about the math
  if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->percent_interpolation = 0;
    col->cc_type = collision_detection::ContinuousCollisionType::Time0;
  }
  else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->percent_interpolation = 1;
    col->cc_type = collision_detection::ContinuousCollisionType::Time1;
  }
  else
  {
    const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
    float l0c = (ptOnCast - ptWorld0).length();
    float l1c = (ptOnCast - ptWorld1).length();

    col->cc_type = collision_detection::ContinuousCollisionType::Between;

    if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
    {
      col->percent_interpolation = .5;
    }
    else
    {
      col->percent_interpolation = static_cast<double>(l0c / (l0c + l1c));
    }
  }

  return 1;
}

/** @brief This is copied directly out of BulletWorld */
struct TesseractBridgedManifoldResult : public btManifoldResult
{
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap, const btCollisionObjectWrapper* obj1Wrap,
                                 btCollisionWorld::ContactResultCallback& resultCallback)
    : btManifoldResult(obj0Wrap, obj1Wrap), m_resultCallback(resultCallback)
  {
  }

  virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld,
                               btScalar depth) override
  {
    bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
    btVector3 localA;
    btVector3 localB;
    if (isSwapped)
    {
      localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.m_positionWorldOnA = pointA;
    newPt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (isSwapped)
    {
      newPt.m_partId0 = m_partId1;
      newPt.m_partId1 = m_partId0;
      newPt.m_index0 = m_index1;
      newPt.m_index1 = m_index0;
    }
    else
    {
      newPt.m_partId0 = m_partId0;
      newPt.m_partId1 = m_partId1;
      newPt.m_index0 = m_index0;
      newPt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
    m_resultCallback.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1,
                                     newPt.m_index1);
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

/** \brief addSingleResult of this struct is called each time the broadphase check indicates a collision. */
struct CastBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  CastBroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false)
    : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
  {
  }

  btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                           int index0, const btCollisionObjectWrapper* colObj1Wrap, int /*partId1*/,
                           int index1) override
  {
    if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not close enough for collision with " << cp.m_distance1);
      return 0;
    }

    return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
  }
};

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
    if (result_callback_.collisions_.done || depth > static_cast<btScalar>(result_callback_.contact_distance_))
      return;

    bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
    btVector3 localA;
    btVector3 localB;
    if (isSwapped)
    {
      localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.m_positionWorldOnA = pointA;
    newPt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (isSwapped)
    {
      newPt.m_partId0 = m_partId1;
      newPt.m_partId1 = m_partId0;
      newPt.m_index0 = m_index1;
      newPt.m_index1 = m_index0;
    }
    else
    {
      newPt.m_partId0 = m_partId0;
      newPt.m_partId1 = m_partId1;
      newPt.m_index0 = m_index0;
      newPt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
    result_callback_.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1,
                                     newPt.m_index1);
  }
};

/** @brief This is copied directly out of BulletWorld
 *
 *  This is currently not used but will remain because it is needed to check a collision object not in the broadphase to
 * the broadphase which may eventually be exposed. */
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
    btCollisionObject* collisionObject = static_cast<btCollisionObject*>(proxy->m_clientObject);
    if (collisionObject == m_collisionObject)
      return true;

    if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
    {
      btCollisionObjectWrapper ob0(nullptr, m_collisionObject->getCollisionShape(), m_collisionObject,
                                   m_collisionObject->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper ob1(nullptr, collisionObject->getCollisionShape(), collisionObject,
                                   collisionObject->getWorldTransform(), -1, -1);

      btCollisionAlgorithm* algorithm = m_dispatcher->findAlgorithm(&ob0, &ob1, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
      if (algorithm)
      {
        TesseractBridgedManifoldResult contactPointResult(&ob0, &ob1, m_resultCallback);
        contactPointResult.m_closestPointDistanceThreshold = m_resultCallback.m_closestDistanceThreshold;

        // discrete collision detection query
        algorithm->processCollision(&ob0, &ob1, m_dispatch_info, &contactPointResult);

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
    if (results_callback_.collisions_.done)
    {
      return false;
    }

    if (results_callback_.collisions_.pair_done)
    {
      results_callback_.collisions_.pair_done = false;
      return false;
    }

    const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
    const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

    if (results_callback_.needsCollision(cow0, cow1))
    {
      btCollisionObjectWrapper obj0Wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper obj1Wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

      // dispatcher will keep algorithms persistent in the collision pair
      if (!pair.m_algorithm)
      {
        pair.m_algorithm = dispatcher_->findAlgorithm(&obj0Wrap, &obj1Wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
      }

      if (pair.m_algorithm)
      {
        TesseractBroadphaseBridgedManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap, results_callback_);
        contactPointResult.m_closestPointDistanceThreshold = static_cast<btScalar>(results_callback_.contact_distance_);

        // discrete collision detection query
        pair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatch_info_, &contactPointResult);
      }
    }
    return false;
  }
};

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type, CollisionObjectWrapper* cow);

/** @brief Update a collision objects filters
 *  @param active A list of active collision objects
 *  @param cow The collision object to update.
 *  @param continuous Indicate if the object is a continuous collision object.
 *
 *  Currently continuous collision objects can only be checked against static objects. Continuous to Continuous
 *  collision checking is currently not supports. TODO LEVI: Add support for Continuous to Continuous collision
 * checking. */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active, COW& cow, bool continuous)
{
  cow.m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;

  if (!isLinkActive(active, cow.getName()))
  {
    cow.m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
  }

  if (cow.m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
  {
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    (continuous) ? (cow.m_collisionFilterMask = btBroadphaseProxy::StaticFilter) :
                   (cow.m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
  }

  if (cow.getBroadphaseHandle())
  {
    cow.getBroadphaseHandle()->m_collisionFilterGroup = cow.m_collisionFilterGroup;
    cow.getBroadphaseHandle()->m_collisionFilterMask = cow.m_collisionFilterMask;
  }
}

inline COWPtr createCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
                                    const std::vector<shapes::ShapeConstPtr>& shapes,
                                    const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                    const std::vector<CollisionObjectType>& collision_object_types, bool enabled = true)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    ROS_DEBUG_NAMED("collision_detection.bullet", "ignoring link %s", name.c_str());
    return nullptr;
  }

  COWPtr new_cow(new COW(name, type_id, shapes, shape_poses, collision_object_types));

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  ROS_DEBUG_NAMED("collision_detection.bullet", "Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

inline COWPtr createCollisionObject(const std::string& name, const collision_detection::BodyType& type_id,
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

  COWPtr new_cow(new COW(name, type_id, shapes, shape_poses, collision_object_types, touch_links));

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  ROS_DEBUG_NAMED("collision_detection.bullet", "Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;
  const COWPtr cow_;
  double contact_distance_;
  bool verbose_;

  DiscreteCollisionCollector(ContactTestData& collisions, const COWPtr cow, double contact_distance,
                             bool verbose = false)
    : collisions_(collisions), cow_(cow), contact_distance_(contact_distance), verbose_(verbose)
  {
    m_closestDistanceThreshold = static_cast<btScalar>(contact_distance);
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
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

struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;
  const COWPtr cow_;
  double contact_distance_;
  bool verbose_;

  CastCollisionCollector(ContactTestData& collisions, const COWPtr cow, double contact_distance, bool verbose = false)
    : collisions_(collisions), cow_(cow), contact_distance_(contact_distance), verbose_(verbose)
  {
    m_closestDistanceThreshold = static_cast<btScalar>(contact_distance);
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                           int index0, const btCollisionObjectWrapper* colObj1Wrap, int /*partId1*/,
                           int index1) override
  {
    if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not close enough for collision with " << cp.m_distance1);
      return 0;
    }

    return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const override
  {
    return !collisions_.done &&
           needsCollisionCheck(*cow_, *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)), collisions_.fn,
                               collisions_.acm, verbose_);
  }
};

inline COWPtr makeCastCollisionObject(const COWPtr& cow)
{
  COWPtr new_cow = cow->clone();

  btTransform tf;
  tf.setIdentity();

  if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
  {
    assert(dynamic_cast<btConvexShape*>(new_cow->getCollisionShape()) != nullptr);
    btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());

    // This checks if the collision object is already a cast collision object
    assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    CastHullShape* shape = new CastHullShape(convex, tf);

    new_cow->manage(shape);
    new_cow->setCollisionShape(shape);
  }
  else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
  {
    btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
    btCompoundShape* new_compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

    for (int i = 0; i < compound->getNumChildShapes(); ++i)
    {
      if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
        assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

        btTransform geomTrans = compound->getChildTransform(i);

        btCollisionShape* subshape = new CastHullShape(convex, tf);

        new_cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geomTrans, subshape);
      }
      else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
      {
        btCompoundShape* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));
        btCompoundShape* new_second_compound =
            new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, second_compound->getNumChildShapes());
        for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
        {
          assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));

          btConvexShape* convex = static_cast<btConvexShape*>(second_compound->getChildShape(j));
          assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

          btTransform geomTrans = second_compound->getChildTransform(j);

          btCollisionShape* subshape = new CastHullShape(convex, tf);

          new_cow->manage(subshape);
          subshape->setMargin(BULLET_MARGIN);
          new_second_compound->addChildShape(geomTrans, subshape);
        }

        btTransform geomTrans = compound->getChildTransform(i);

        new_cow->manage(new_second_compound);
        new_second_compound->setMargin(
            BULLET_MARGIN);  // margin: compound. seems to have no effect when positive but has an effect when negative
        new_compound->addChildShape(geomTrans, new_second_compound);
      }
      else
      {
        ROS_ERROR_NAMED("collision_detection.bullet",
                        "I can only collision check convex shapes and compound shapes made of convex shapes");
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }

    new_compound->setMargin(
        BULLET_MARGIN);  // margin: compound. seems to have no effect when positive but has an effect when negative
    new_cow->manage(new_compound);
    new_cow->setCollisionShape(new_compound);
    new_cow->setWorldTransform(cow->getWorldTransform());
  }
  else
  {
    ROS_ERROR_NAMED("collision_detection.bullet",
                    "I can only collision check convex shapes and compound shapes made of convex shapes");
    throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
  }

  return new_cow;
}

/** @brief Update the Broadphase AABB for the input collision object
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void updateBroadphaseAABB(const COWPtr& cow, const std::unique_ptr<btBroadphaseInterface>& broadphase,
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
inline void removeCollisionObjectFromBroadphase(const COWPtr& cow,
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
inline void addCollisionObjectToBroadphase(const COWPtr& cow, const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                           const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Added " << cow->getName() << " to broadphase");
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  // Add the active collision object to the broadphase
  int type = cow->getCollisionShape()->getShapeType();
  cow->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup,
                                                   cow->m_collisionFilterMask, dispatcher.get()));
}
}
#endif  // TESSERACT_COLLISION_BULLET_UTILS_H
