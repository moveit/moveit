/**
 * @file discrete_contact_manager_base.h
 * @brief This is the discrete contact manager base class
 *
 * It should be used to perform discrete contact checking.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_BASE_H
#define TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_BASE_H

#include <geometric_shapes/shapes.h>
#include <memory>

#include <moveit/collision_detection_bullet/tesseract/basic_types.h>
#include <btBulletCollisionCommon.h>

namespace tesseract
{
class DiscreteContactManagerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~DiscreteContactManagerBase() = default;
  /**
   * @brief Clone the manager
   *
   * This is to be used for multi threaded application. A user should
   * make a cone for each thread.
   */
  virtual std::shared_ptr<DiscreteContactManagerBase> clone() const = 0;

  /**
   * @brief Add a object to the checker
   * @param name            The name of the object, must be unique.
   * @param mask_id         User defined id which gets stored in the results structure.
   * @param shapes          A vector of shapes that make up the collision object.
   * @param shape_poses     A vector of poses for each shape, must be same length as shapes
   * @param shape_types     A vector of shape types for encode the collision object. If the vector is of length 1 it is
   * used for all shapes.
   * @param collision_object_types A int identifying a conversion mode for the object. (ex. convert meshes to
   * convex_hulls)
   * @return true if successfully added, otherwise false.
   */
  virtual bool addCollisionObject(const std::string& name, const collision_detection::BodyType& mask_id,
                                  const std::vector<shapes::ShapeConstPtr>& shapes,
                                  const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                  const std::vector<CollisionObjectType>& collision_object_types,
                                  bool enabled = true) = 0;

  /**
   * @brief Find if a collision object already exists
   * @param name The name of the collision object
   * @return true if it exists, otherwise false.
   */
  virtual bool hasCollisionObject(const std::string& name) const = 0;

  /**
   * @brief Remove an object from the checker
   * @param name The name of the object
   * @return true if successfully removed, otherwise false.
   */
  virtual bool removeCollisionObject(const std::string& name) = 0;

  /**
   * @brief Enable an object
   * @param name The name of the object
   */
  virtual bool enableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Disable an object
   * @param name The name of the object
   */
  virtual bool disableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Set a single collision object's tansforms
   * @param name The name of the object
   * @param pose The tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

  virtual void setCollisionObjectsTransform(const std::string& name, const btTransform& pose) = 0;

  /**
   * @brief Set a series of collision object's tranforms
   * @param names The name of the object
   * @param poses The tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const AlignedVector<Eigen::Isometry3d>& poses) = 0;

  /**
   * @brief Set a series of collision object's tranforms
   * @param transforms A transform map <name, pose>
   */
  virtual void setCollisionObjectsTransform(const AlignedMap<std::string, Eigen::Isometry3d>& transforms) = 0;

  /**
   * @brief Set which collision objects can move
   * @param names A vector of collision object names
   */
  virtual void setActiveCollisionObjects(const std::vector<std::string>& names) = 0;

  /**
   * @brief Get which collision objects can move
   * @return A list of collision object names
   */
  virtual const std::vector<std::string>& getActiveCollisionObjects() const = 0;

  /**
   * @brief Set the contact distance threshold for which collision should be considered.
   * @param contact_distance The contact distance
   */
  virtual void setContactDistanceThreshold(double contact_distance) = 0;

  /**
   * @brief Get the contact distance threshold
   * @return The contact distance
   */
  virtual double getContactDistanceThreshold() const = 0;

  /** @brief Set the active function for determining if two links are allowed to be in collision */
  virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

  /** @brief Get the active function for determining if two links are allowed to be in collision */
  virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;

  /**
   * @brief Perform a contact test for all objects based
   * @param collisions The Contact results data
   * @param type The type of check (ALL, FIRST...)
   * @param acm The allowed collision matrix
   * @param req The contact request
   */
  virtual void contactTest(collision_detection::CollisionResult& collisions,
                           const collision_detection::CollisionRequest& req,
                           const collision_detection::AllowedCollisionMatrix* acm) = 0;
};
typedef std::shared_ptr<DiscreteContactManagerBase> DiscreteContactManagerBasePtr;
typedef std::shared_ptr<const DiscreteContactManagerBase> DiscreteContactManagerBaseConstPtr;
}
#endif  // TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_BASE_H
