/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/joint_model.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model/aabb.h>

namespace moveit
{
namespace core
{
LinkModel::LinkModel(const std::string& name)
  : name_(name)
  , parent_joint_model_(nullptr)
  , parent_link_model_(nullptr)
  , is_parent_joint_fixed_(false)
  , joint_origin_transform_is_identity_(true)
  , first_collision_body_transform_index_(-1)
  , link_index_(-1)
{
  joint_origin_transform_.setIdentity();
}

LinkModel::~LinkModel() = default;

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& transform)
{
  joint_origin_transform_ = transform;
  joint_origin_transform_is_identity_ =
      joint_origin_transform_.rotation().isIdentity() &&
      joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}

void LinkModel::setParentJointModel(const JointModel* joint)
{
  parent_joint_model_ = joint;
  is_parent_joint_fixed_ = joint->getType() == JointModel::FIXED;
}

void LinkModel::setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Isometry3d& origins)
{
  shapes_ = shapes;
  collision_origin_transform_ = origins;
  collision_origin_transform_is_identity_.resize(collision_origin_transform_.size());

  core::AABB aabb;

  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    collision_origin_transform_is_identity_[i] =
        (collision_origin_transform_[i].rotation().isIdentity() &&
         collision_origin_transform_[i].translation().norm() < std::numeric_limits<double>::epsilon()) ?
            1 :
            0;
    Eigen::Isometry3d transform = collision_origin_transform_[i];

    if (shapes_[i]->type != shapes::MESH)
    {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes_[i].get());
      aabb.extendWithTransformedBox(transform, extents);
    }
    else
    {
      // we cannot use shapes::computeShapeExtents() for meshes, since that method does not provide information about
      // the offset of the mesh origin
      const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shapes_[i].get());
      for (unsigned int j = 0; j < mesh->vertex_count; ++j)
      {
        aabb.extend(transform * Eigen::Map<Eigen::Vector3d>(&mesh->vertices[3 * j]));
      }
    }
  }

  centered_bounding_box_offset_ = aabb.center();
  if (shapes_.empty())
    shape_extents_.setZero();
  else
    shape_extents_ = aabb.sizes();
}

void LinkModel::setVisualMesh(const std::string& visual_mesh, const Eigen::Isometry3d& origin,
                              const Eigen::Vector3d& scale)
{
  visual_mesh_filename_ = visual_mesh;
  visual_mesh_origin_ = origin;
  visual_mesh_scale_ = scale;
}

// Calculating hash for LinkModel
namespace std
{
  template<> struct hash<LinkModel>
  {
    std::size_t operator()(LinkModel const& link) const noexcept
    {
			std::size_t h =0;
      // use Link name_
			boost::hash_combine(h, link.name_);
      boost::hash_combine(h, link.parent_joint_model_.hash());
      boost::hash_combine(h, link.parent_link_model_.hash());
      boost::hash_combine(h, link.is_parent_joint_fixed_);
      boost::hash_combine(h, link.joint_origin_transform_is_identity);

      // rows = 3 and cols = 4 since it is Isometry3d
      const MatrixType* transform_matrix = link.joint_origin_transform_.matrix()
      for (auto i = 0; i < 3; i++) {
        for (auto j = 0; j < 4; j++) {
          boost::hash_combine(h, transform_matrix[i][j]);
        }
      }

      for (auto& collision_isometry : link.collision_origin_transform_) {
        boost::hash_combine(h, collision_isometry);
      }

      for (auto& identity : link.collision_origin_transform_is_identity_) {
        boost:hash_combine(h, identity);
      }

      for (auto const& x: link.associated_fixed_transforms_) {
        const Eigen::Isometry3d matrix = x->second;
            boost::hash_combine(h, matrix.hash());
        }
    
      for(auto const& x : link.shapes_) {
        boost::hash_combine(h, h.hash())
      }
      boost::hash_combine(h, link.shape_extents_.hash());
      boost::hash_combine(h, link.centered_bounding_box_offset_.hash());
      
      boost::hash_combine(h, link.visual_mesh_filename_.hash());

      boost::hash_combine(h, link.visual_mesh_origin.hash());

      boost::hash_combine(h, link.visual_mesh_scale_.hash());

      boost::hash_combine(h, link.first_collision_body_transform_index_);

      boost::hash_combine(h, link.link_index_);
      return h;

    }
  }   


  // Eigen Isometry3d
  template<>
  struct hash<Eigen::Isometry3d>
  {
    std::size_t operator(const Eigen::Isometry3d& transform_matrix) const 
    {
        std::size_t h = 0;
        const MatrixType* transform_matrix : transform_matrix.matrix();
        for (auto i = 0; i < 3; i++) 
        {
          for (auto j = 0; j < 4; j++) 
          {
            boost::hash_combine(h, matrix[i][j]);
          }
        }
        return h;
    }
  }

  // Eigen Vector3d
  template<>
  struct hash<Eigen::Vector3d>
  {
    std::size_t operator(const Eigen::Vector3d& v) const 
    {
        std::size_t h = 0;
        for (auto i = 0; i < 3; i++) 
        {
          boost::hash_combine(h, v[i][0]);
        }
    }
  }

  // ROS Shapes
  template<> 
  struct hash<shapes::Box>
  {
    std::size_t operator()(const& shape::Box box) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, Box::STRING_NAME);
      // box size
      for (auto i = 0; i < 3; i++)
        boost::hash_combine(h, size[i])
      return h;
    }
  }

  template<> 
  struct hash<shapes::Cylinder>
  {
    std::size_t operator()(const& shape::Cylinder c) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, Cylinder::STRING_NAME);
      boost::hash_combine(h, cone.length);
      boost::hash_combine(h, cone.radius);
      return h;
    }
  }

  template<> 
  struct hash<shapes::Mesh>
  {
    std::size_t operator()(const& shape::Mesh m) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, Mesh::STRING_NAME);
      boost::hash_combine(h, m.triangle_count);
      n = 3 * m.triangle_count;
      for(auto i  = 0; i < n; i++) {
        boost::hash_combine(h, m.triangles[i]);
        boost::hash_combine(h, m.triangle_normals[i]);
      }
      boost::hash_combine(h, m.vertex_count);
      n = 3 * m.vertex_count;
      for(auto i = 0; i < n; i++) {
        boost::hash_combine(h, m.vertices[i]);
        boost::hash_combine(h, m.vertex_normals[i]);
      }
      return h;
    }
  }

  template<> 
  struct hash<shapes::Sphere>
  {
    std::size_t operator()(const& shape::Sphere s) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, Sphere::STRING_NAME);
      boost::hash_combine(h, s.radius);
      return h;
    }
  }
} // end of namespace std

}  // end of namespace core
}  // end of namespace moveit