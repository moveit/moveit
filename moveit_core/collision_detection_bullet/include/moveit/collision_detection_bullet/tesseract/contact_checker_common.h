/**
 * @file contact_checker_common.h
 * @brief This is a collection of common methods
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
#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H

#include <LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <Eigen/Geometry>
#include <fstream>

#include <moveit/collision_detection_bullet/tesseract/basic_types.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace tesseract
{
typedef std::pair<std::string, std::string> ObjectPairKey;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool isContactAllowed(const std::string& name1, const std::string& name2, const IsContactAllowedFn acm_fn,
                             const collision_detection::AllowedCollisionMatrix* acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm_fn != nullptr && acm_fn(name1, name2, acm))
  {
    if (verbose)
    {
      ROS_DEBUG("Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    ROS_DEBUG("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

/** \brief Stores a single contact result in the requested way.
*   \param found Indicates if a contact for this pair of objects has already been found
*   \return Pointer to the newly inserted contact */
inline collision_detection::Contact* processResult(ContactTestData& cdata, collision_detection::Contact& contact,
                                                   const std::pair<std::string, std::string>& key, bool found)
{
  // TODO: Add check for max_number_contact between single pair how to skip those two objects then
  if (!found)
  {
    ROS_DEBUG_STREAM("Contact btw " << key.first << " and " << key.second << " dist: " << contact.depth);
    cdata.res.collision = true;
    std::vector<collision_detection::Contact> data;
    if (!cdata.req.contacts)
    {
      cdata.done = true;
      return nullptr;
    }
    else
    {
      data.reserve(cdata.req.max_contacts_per_pair);
      data.emplace_back(contact);
      cdata.res.contact_count++;
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      cdata.done = true;
    }

    return &(cdata.res.contacts.insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    std::vector<collision_detection::Contact>& dr = cdata.res.contacts[key];
    dr.emplace_back(contact);
    cdata.res.contact_count++;

    if (dr.size() >= cdata.req.max_contacts_per_pair)
    {
      // TODO: Somehow abort collision check between this pair of objects
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      cdata.done = true;
    }

    return &(dr.back());
  }

  return nullptr;
}

/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice
 * index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occured when trying to create the convex hull
 */
inline int createConvexHull(AlignedVector<Eigen::Vector3d>& vertices, std::vector<int>& faces,
                            const AlignedVector<Eigen::Vector3d>& input, double shrink = -1, double shrinkClamp = -1)
{
  vertices.clear();
  faces.clear();

  btConvexHullComputer conv;
  btAlignedObjectArray<btVector3> points;
  points.reserve(static_cast<int>(input.size()));
  for (const auto& v : input)
  {
    points.push_back(btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));
  }

  btScalar val = conv.compute(&points[0].getX(), sizeof(btVector3), points.size(), static_cast<btScalar>(shrink),
                              static_cast<btScalar>(shrinkClamp));
  if (val < 0)
  {
    ROS_ERROR("Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(static_cast<size_t>(num_verts));
  for (int i = 0; i < num_verts; i++)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(Eigen::Vector3d(v.getX(), v.getY(), v.getZ()));
  }

  int num_faces = conv.faces.size();
  faces.reserve(static_cast<size_t>(3 * num_faces));
  for (int i = 0; i < num_faces; i++)
  {
    std::vector<int> face;
    face.reserve(3);

    const btConvexHullComputer::Edge* sourceEdge = &(conv.edges[conv.faces[i]]);
    int a = sourceEdge->getSourceVertex();
    face.push_back(a);

    int b = sourceEdge->getTargetVertex();
    face.push_back(b);

    const btConvexHullComputer::Edge* edge = sourceEdge->getNextEdgeOfFace();
    int c = edge->getTargetVertex();
    face.push_back(c);

    edge = edge->getNextEdgeOfFace();
    c = edge->getTargetVertex();
    while (c != a)
    {
      face.push_back(c);

      edge = edge->getNextEdgeOfFace();
      c = edge->getTargetVertex();
    }
    faces.push_back(static_cast<int>(face.size()));
    faces.insert(faces.end(), face.begin(), face.end());
  }

  return num_faces;
}

inline bool allowedCollisionCheck(const std::string body_1, const std::string body_2,
                                  const collision_detection::AllowedCollisionMatrix* acm)
{
  collision_detection::AllowedCollision::Type allowed_type;

  if (acm != nullptr)
  {
    if (acm->getEntry(body_1, body_2, allowed_type))
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
}

#endif  // TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
