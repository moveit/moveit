/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

/* Author: Levi Armstrong */

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BASIC_TYPES_H_
#define MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BASIC_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometric_shapes/shapes.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>
#include <map>
#include <moveit/collision_detection/collision_common.h>

namespace collision_detection_bullet
{
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename Key, typename Value>
using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key, Value, std::hash<Key>, std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

/** @brief Checks if contact is allowed or not. Returns true if yes, otherwise false.
 *
 * The order of strings should not matter, the function should handled by the function. */
typedef std::function<bool(const std::string&, const std::string&, const collision_detection::AllowedCollisionMatrix*)>
    IsContactAllowedFn;

enum class CollisionObjectType
{
  USE_SHAPE_TYPE = 0, /**< @brief Infer the type from the type specified in the shapes::Shape class */

  // all of the following convert the meshes to custom collision objects
  CONVEX_HULL = 1,  /**< @brief Use the mesh in shapes::Shape but make it a convex hulls collision object (if not convex
                      it will be converted) */
  MULTI_SPHERE = 2, /**< @brief Use the mesh and represent it by multiple spheres collision object */
  SDF = 3           /**< @brief Use the mesh and rpresent it by a signed distance fields collision object */
};

/** \brief Bundles the data for a collision query */
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData(const std::vector<std::string>& active, const double& contact_distance, const IsContactAllowedFn& fn,
                  collision_detection::CollisionResult& res, const collision_detection::CollisionRequest& req,
                  const collision_detection::AllowedCollisionMatrix* acm)
    : active(active)
    , contact_distance(contact_distance)
    , fn(fn)
    , acm(acm)
    , res(res)
    , req(req)
    , done(false)
    , pair_done(false)
  {
  }

  const std::vector<std::string>& active;

  /** \brief If after a positive broadphase check the distance is below this threshold, a contact is added. */
  const double& contact_distance;

  /** \brief User defined function which checks if contact is allowed between two objects */
  const IsContactAllowedFn& fn;

  /** \brief Indicates collision objects which are allowed to be in contact */
  const collision_detection::AllowedCollisionMatrix* acm;

  collision_detection::CollisionResult& res;
  const collision_detection::CollisionRequest& req;

  /// Indicates if search is finished
  bool done;

  /// Indicates if search between a single pair is finished
  bool pair_done;
};

}  // namespace collision_detection_bullet

#endif  // MOVEIT_COLLISION_DETECTION_BULLET_BULLET_INTEGRATION_BASIC_TYPES_H_
