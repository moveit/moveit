/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef MOVEIT_ROBOT_MODEL_LINK_MODEL_
#define MOVEIT_ROBOT_MODEL_LINK_MODEL_

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>

namespace robot_model
{

class JointModel;

/** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
class LinkModel
{
  friend class RobotModel;
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::map<const LinkModel*, Eigen::Affine3d, std::less<const LinkModel*>,
                   Eigen::aligned_allocator<std::pair<const LinkModel*, Eigen::Affine3d> > > AssociatedFixedTransformMap;

  LinkModel();
  ~LinkModel();

  /** \brief The name of this link */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief The index of this joint when traversing the kinematic tree in depth first fashion */
  int getTreeIndex() const
  {
    return tree_index_;
  }

  /** \brief Get the joint model whose child this link is. There will always be a parent joint */
  const JointModel* getParentJointModel() const
  {
    return parent_joint_model_;
  }

  /** \brief A link may have 0 or more child joints. From those joints there will certainly be other descendant links */
  const std::vector<JointModel*>& getChildJointModels() const
  {
    return child_joint_models_;
  }

  /** \brief When transforms are computed for this link,
      they are usually applied to the link's origin. The
      joint origin transform acts as an offset -- it is
      pre-applied before any other transform */
  const Eigen::Affine3d& getJointOriginTransform() const
  {
    return joint_origin_transform_;
  }


  /** \brief In addition to the link transform, the geometry
      of a link that is used for collision checking may have
      a different offset itself, with respect to the origin */
  const Eigen::Affine3d& getCollisionOriginTransform() const
  {
    return collision_origin_transform_;
  }

  /** \brief Get shape associated to the collision geometry for this link */
  const shapes::ShapeConstPtr& getShape() const
  {
    return shape_;
  }

  /** \brief Get shape associated to the collision geometry for this link */
  const shapes::ShapeMsg& getShapeMsg() const
  {
    return shape_msg_;
  }

  /** \brief Get the extenrs of the link's geometry (dimensions of axis-aligned bounding box when the link is positioned at origin) */
  const Eigen::Vector3d& getShapeExtentsAtOrigin() const
  {
    return shape_extents_;
  }

  /** \brief Get the set of links that are attached to this one via fixed transforms */
  const AssociatedFixedTransformMap& getAssociatedFixedTransforms() const
  {
    return associated_fixed_transforms_;
  }

  /** \brief Get the filename of the mesh resource used for visual display of this link */
  const std::string& getVisualMeshFilename() const
  {
    return visual_mesh_filename_;
  }

  /** \brief Get the scale of the mesh resource for this link */
  const Eigen::Vector3d& getVisualMeshScale() const
  {
    return visual_mesh_scale_;
  }

private:

  /** \brief Name of the link */
  std::string               name_;

  /** \brief JointModel that connects this link to the parent link */
  JointModel               *parent_joint_model_;

  /** \brief List of descending joints (each connects to a child link) */
  std::vector<JointModel*>  child_joint_models_;

  /** \brief The set of links that are attached to this one via fixed transforms */
  AssociatedFixedTransformMap associated_fixed_transforms_;

  /** \brief The constant transform applied to the link (local) */
  Eigen::Affine3d           joint_origin_transform_;

  /** \brief The constant transform applied to the collision geometry of the link (local) */
  Eigen::Affine3d           collision_origin_transform_;

  /** \brief The collision geometry of the link */
  shapes::ShapeConstPtr     shape_;

  /** \brief The collision geometry of the link as a message */
  shapes::ShapeMsg          shape_msg_;

  /** \brief The extents if shape (dimensions of axis aligned bounding box when shape is at origin */
  Eigen::Vector3d           shape_extents_;

  /** \brief Filename associated with the visual geometry mesh of this link. If empty, no mesh was used. */
  std::string               visual_mesh_filename_;

  /** \brief Scale factor associated with the visual geometry mesh of this link. */
  Eigen::Vector3d           visual_mesh_scale_;

  /** \brief The index assigned to this link when traversing the kinematic tree in depth first fashion */
  int                       tree_index_;

};
}

#endif
