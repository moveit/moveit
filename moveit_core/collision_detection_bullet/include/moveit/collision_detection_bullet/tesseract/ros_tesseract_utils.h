/**
 * @file ros_tesseract_utils.h
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_ROS_UTILS_H
#define TESSERACT_ROS_UTILS_H

#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection_bullet/tesseract/basic_types.h>

namespace tesseract
{
namespace tesseract_ros
{
static inline bool isIdentical(const shapes::Shape& shape1, const shapes::Shape& shape2)
{
  if (shape1.type != shape2.type)
    return false;

  switch (shape1.type)
  {
    case shapes::BOX:
    {
      const shapes::Box& s1 = static_cast<const shapes::Box&>(shape1);
      const shapes::Box& s2 = static_cast<const shapes::Box&>(shape2);

      for (unsigned i = 0; i < 3; ++i)
        if (std::abs(s1.size[i] - s2.size[i]) > std::numeric_limits<double>::epsilon())
          return false;

      break;
    }
    case shapes::SPHERE:
    {
      const shapes::Sphere& s1 = static_cast<const shapes::Sphere&>(shape1);
      const shapes::Sphere& s2 = static_cast<const shapes::Sphere&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::CYLINDER:
    {
      const shapes::Cylinder& s1 = static_cast<const shapes::Cylinder&>(shape1);
      const shapes::Cylinder& s2 = static_cast<const shapes::Cylinder&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.length - s2.length) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::CONE:
    {
      const shapes::Cone& s1 = static_cast<const shapes::Cone&>(shape1);
      const shapes::Cone& s2 = static_cast<const shapes::Cone&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.length - s2.length) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::MESH:
    {
      const shapes::Mesh& s1 = static_cast<const shapes::Mesh&>(shape1);
      const shapes::Mesh& s2 = static_cast<const shapes::Mesh&>(shape2);

      if (s1.vertex_count != s2.vertex_count)
        return false;

      if (s1.triangle_count != s2.triangle_count)
        return false;

      break;
    }
    case shapes::OCTREE:
    {
      const shapes::OcTree& s1 = static_cast<const shapes::OcTree&>(shape1);
      const shapes::OcTree& s2 = static_cast<const shapes::OcTree&>(shape2);

      if (s1.octree->getTreeType() != s2.octree->getTreeType())
        return false;

      if (s1.octree->size() != s2.octree->size())
        return false;

      if (s1.octree->getTreeDepth() != s2.octree->getTreeDepth())
        return false;

      if (s1.octree->memoryUsage() != s2.octree->memoryUsage())
        return false;

      if (s1.octree->memoryFullGrid() != s2.octree->memoryFullGrid())
        return false;

      break;
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(shape1.type));
      return false;
  }

  return true;
}

static inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                               const urdf::LinkConstSharedPtr urdf_link, bool active)
{
  // recursively get all active links
  if (active)
  {
    active_links.push_back(urdf_link->name);
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], active);
    }
  }
  else
  {
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      const urdf::LinkConstSharedPtr child_link = urdf_link->child_links[i];
      if ((child_link->parent_joint) && (child_link->parent_joint->type != urdf::Joint::FIXED))
        getActiveLinkNamesRecursive(active_links, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, child_link, active);
    }
  }
}

inline shapes::ShapePtr constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* result = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                    static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
    default:
      ROS_ERROR("Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
  }

  return shapes::ShapePtr(result);
}

inline Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}
}
}
#endif  // TESSERACT_ROS_UTILS_H
