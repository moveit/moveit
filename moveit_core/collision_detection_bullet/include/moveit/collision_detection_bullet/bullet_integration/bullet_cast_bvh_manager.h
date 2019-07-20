/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
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

/* Author: Levi Armstrong */

#ifndef TESSERACT_COLLISION_BULLET_CAST_BVH_MANAGERS_H
#define TESSERACT_COLLISION_BULLET_CAST_BVH_MANAGERS_H

#include <moveit/collision_detection_bullet/bullet_integration/bullet_utils.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection_bullet
{
MOVEIT_CLASS_FORWARD(BulletCastBVHManager)

/** @brief A bounding volume hierarchy (BVH) implementation of a tesseract contact manager */
class BulletCastBVHManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor */
  BulletCastBVHManager();

  ~BulletCastBVHManager();

  /** @brief Clone the manager
   *
   * This is to be used for multi threaded applications. A user should make a clone for each thread. */
  BulletCastBVHManagerPtr clone() const;

  /**@brief Add a collision object to the checker
   *
   * All objects are added should initially be added as static objects. Use the setContactRequest method of defining
   * which collision objects are moving.
   *
   * @param name            The name of the object, must be unique.
   * @param mask_id         User defined id which gets stored in the results structure.
   * @param shapes          A vector of shapes that make up the collision object.
   * @param shape_poses     A vector of poses for each shape, must be same length as shapes
   * @param shape_types     A vector of shape types for encode the collision object. If the vector is of length 1 it is
   * used for all shapes.
   * @param collision_object_types A int identifying a conversion mode for the object. (ex. convert meshes to convex
   * hulls)
   * @param enabled         Indicate if the object is enabled for collision checking.
   * @return true if successfully added, otherwise false. */
  bool addCollisionObject(const std::string& name, const collision_detection::BodyType& mask_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const AlignedVector<Eigen::Isometry3d>& shape_poses,
                          const std::vector<CollisionObjectType>& collision_object_types, bool enabled = true);

  /**@brief Find if a collision object already exists
   * @param name The name of the collision object
   * @return true if it exists, otherwise false. */
  bool hasCollisionObject(const std::string& name) const;

  /**@brief Remove an object from the checker
   * @param name The name of the object
   * @return true if successfully removed, otherwise false. */
  bool removeCollisionObject(const std::string& name);

  /**@brief Enable an object
   * @param name The name of the object
   * @return true if successfully enabled, otherwise false. */
  bool enableCollisionObject(const std::string& name);

  /**@brief Disable an object
   * @param name The name of the object
   * @return true if successfully disabled, otherwise false. */
  bool disableCollisionObject(const std::string& name);

  /**@brief Set a single static collision object's tansform
   * @param name The name of the object
   * @param pose The tranformation in world */
  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose);

  /**@brief Set a series of static collision objects' tranforms
   * @param names The names of the objects
   * @param poses The tranformations in world */
  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const AlignedVector<Eigen::Isometry3d>& poses);

  /**@brief Set a series of static collision objects' tranforms
   * @param transforms A transform map <name, pose> */
  void setCollisionObjectsTransform(const AlignedMap<std::string, Eigen::Isometry3d>& transforms);

  /**@brief Set a single cast (moving) collision object's tansforms
   *
   * This should only be used for moving objects.
   *
   * @param name The name of the object
   * @param pose1 The start tranformation in world
   * @param pose2 The end tranformation in world */
  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose1,
                                    const Eigen::Isometry3d& pose2);

  /**@brief Set a series of cast (moving) collision objects' tranforms
   *
   * This should only be used for moving objects. Use the base class methods for static objects.
   *
   * @param names The name of the object
   * @param pose1 The start tranformations in world
   * @param pose2 The end tranformations in world */
  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const AlignedVector<Eigen::Isometry3d>& pose1,
                                    const AlignedVector<Eigen::Isometry3d>& pose2);

  /**@brief Set a series of cast (moving) collision objects' tranforms
   *
   * This should only be used for moving objects. Use the base class methods for static objects.
   *
   * @param pose1 A start transform map <name, pose>
   * @param pose2 A end transform map <name, pose> */
  void setCollisionObjectsTransform(const AlignedMap<std::string, Eigen::Isometry3d>& pose1,
                                    const AlignedMap<std::string, Eigen::Isometry3d>& pose2);

  /**@brief Set which collision objects are active
   * @param names A vector of collision object names */
  void setActiveCollisionObjects(const std::vector<std::string>& names);

  /**@brief Get which collision objects are active
   * @return A vector of collision object names */
  const std::vector<std::string>& getActiveCollisionObjects() const;

  /**@brief Set the contact distance threshold for which collision should be considered.
   * @param contact_distance The contact distance */
  void setContactDistanceThreshold(double contact_distance);

  /**@brief Get the contact distance threshold
   * @return The contact distance */
  double getContactDistanceThreshold() const;

  /** @brief Set the function for determining if two links are allowed to be in collision */
  void setIsContactAllowedFn(IsContactAllowedFn fn);

  /** @brief Get the function for determining if two links are allowed to be in collision */
  IsContactAllowedFn getIsContactAllowedFn() const;

  /**@brief Perform a contact test for all objects
   * @param collisions The Contact results data
   * @param req The collision request data
   * @param acm The allowed collision matrix */
  void contactTest(collision_detection::CollisionResult& collisions, const collision_detection::CollisionRequest& req,
                   const collision_detection::AllowedCollisionMatrix* acm);

  /**@brief Add a tesseract collision object to the manager
   * @param cow The tesseract bullet collision object */
  void addCollisionObject(const CollisionObjectWrapperPtr& cow);

  /** \brief Callback function for culling objects before a broadphase check */
  BroadphaseFilterCallback filter_callback_;

private:
  /** @brief A list of the active collision objects */
  std::vector<std::string> active_;

  /** @brief The contact distance threshold */
  double contact_distance_;

  /** @brief The is allowed collision function */
  IsContactAllowedFn fn_;

  /** @brief The bullet collision dispatcher used for getting object to object collison algorithm */
  std::unique_ptr<btCollisionDispatcher> dispatcher_;

  /** @brief The bullet collision dispatcher configuration information */
  btDispatcherInfo dispatch_info_;

  /** @brief The bullet collision configuration */
  btDefaultCollisionConfiguration coll_config_;

  /** @brief The bullet broadphase interface */
  std::unique_ptr<btBroadphaseInterface> broadphase_;

  /** @brief A map of collision objects being managed */
  std::map<std::string, CollisionObjectWrapperPtr> link2cow_;

  /** @brief A map of cast collision objects being managed. */
  std::map<std::string, CollisionObjectWrapperPtr> link2castcow_;

  /**@brief Perform a contact test for the provided object which is not part of the manager
   * @param cow The Collision object
   * @param collisions The collision results */
  void contactTest(const CollisionObjectWrapperPtr& cow, ContactTestData& collisions);
};
}  // namespace collision_detection_bullet
#endif  //  TESSERACT_COLLISION_BULLET_CAST_BVH_MANAGERS_H
