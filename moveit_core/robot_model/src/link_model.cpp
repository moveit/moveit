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

moveit::core::LinkModel::LinkModel(const std::string &name)
  : name_(name)
  , parent_joint_model_(NULL)
  , parent_link_model_(NULL)
  , is_parent_joint_fixed_(false)
  , joint_origin_transform_is_identity_(true)
  , first_collision_body_transform_index_(-1)
  , link_index_(-1)
{
  joint_origin_transform_.setIdentity();
}

moveit::core::LinkModel::~LinkModel()
{
}

void moveit::core::LinkModel::setJointOriginTransform(const Eigen::Affine3d &transform)
{
  joint_origin_transform_ = transform;
  joint_origin_transform_is_identity_ = joint_origin_transform_.rotation().isIdentity() &&
    joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}

void moveit::core::LinkModel::setParentJointModel(const JointModel *joint)
{
  parent_joint_model_ = joint;
  is_parent_joint_fixed_ = joint->getType() == JointModel::FIXED;
}

void moveit::core::LinkModel::setGeometry(const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &origins)
{
  shapes_ = shapes;
  collision_origin_transform_ = origins;
  collision_origin_transform_is_identity_.resize(collision_origin_transform_.size());

  Eigen::Vector3d a = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d b = Eigen::Vector3d(0.0, 0.0, 0.0);

  for (std::size_t i = 0 ; i < shapes_.size() ; ++i)
  {
    collision_origin_transform_is_identity_[i] = (collision_origin_transform_[i].rotation().isIdentity() &&
                                                  collision_origin_transform_[i].translation().norm() < std::numeric_limits<double>::epsilon()) ? 1 : 0;
    Eigen::Vector3d ei = shapes::computeShapeExtents(shapes_[i].get());
    Eigen::Vector3d p1 = collision_origin_transform_[i] * (-ei / 2.0);
    Eigen::Vector3d p2 = collision_origin_transform_[i] * (-p1);

    if (i == 0)
    {
      a = p1;
      b = p2;
    }
    else
    {
      for (int i = 0 ; i < 3 ; ++i)
        a[i] = std::min(a[i], p1[i]);
      for (int i = 0 ; i < 3 ; ++i)
        b[i] = std::max(b[i], p2[i]);
    }
  }

  shape_extents_ = b - a;
}

void moveit::core::LinkModel::setVisualMesh(const std::string &visual_mesh, const Eigen::Affine3d &origin, const Eigen::Vector3d &scale)
{
  visual_mesh_filename_ = visual_mesh;
  visual_mesh_origin_ = origin;
  visual_mesh_scale_ = scale;
}
