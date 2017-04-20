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

#ifndef MOVEIT_ROBOT_STATE_ATTACHED_BODY_
#define MOVEIT_ROBOT_STATE_ATTACHED_BODY_

#include <moveit/robot_model/link_model.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <boost/function.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <set>

namespace moveit
{
namespace core
{
class AttachedBody;
typedef boost::function<void(AttachedBody* body, bool attached)> AttachedBodyCallback;

/** @brief Object defining bodies that can be attached to robot
 *  links. This is useful when handling objects picked up by
 *  the robot. */
class AttachedBody
{
public:
  /** \brief Construct an attached body for a specified \e link. The name of this body is \e id and it consists of \e
     shapes that
      attach to the link by the transforms \e attach_trans. The set of links that are allowed to be touched by this
     object is specified by \e touch_links. */
  AttachedBody(const LinkModel* link, const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
               const EigenSTL::vector_Affine3d& attach_trans, const std::set<std::string>& touch_links,
               const trajectory_msgs::JointTrajectory& attach_posture);

  ~AttachedBody();

  /** \brief Get the name of the attached body */
  const std::string& getName() const
  {
    return id_;
  }

  /** \brief Get the name of the link this body is attached to */
  const std::string& getAttachedLinkName() const
  {
    return parent_link_model_->getName();
  }

  /** \brief Get the model of the link this body is attached to */
  const LinkModel* getAttachedLink() const
  {
    return parent_link_model_;
  }

  /** \brief Get the shapes that make up this attached body */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const
  {
    return shapes_;
  }

  /** \brief Get the links that the attached body is allowed to touch */
  const std::set<std::string>& getTouchLinks() const
  {
    return touch_links_;
  }

  /** \brief Return the posture that is necessary for the object to be released, (if any). This is useful for example
     when storing
      the configuration of a gripper holding an object */
  const trajectory_msgs::JointTrajectory& getDetachPosture() const
  {
    return detach_posture_;
  }

  /** \brief Get the fixed transform (the transforms to the shapes associated with this body) */
  const EigenSTL::vector_Affine3d& getFixedTransforms() const
  {
    return attach_trans_;
  }

  /** \brief Get the global transforms for the collision bodies */
  const EigenSTL::vector_Affine3d& getGlobalCollisionBodyTransforms() const
  {
    return global_collision_body_transforms_;
  }

  /** \brief Set the padding for the shapes of this attached object */
  void setPadding(double padding);

  /** \brief Set the scale for the shapes of this attached object */
  void setScale(double scale);

  /** \brief Recompute global_collision_body_transform given the transform of the parent link*/
  void computeTransform(const Eigen::Affine3d& parent_link_global_transform)
  {
    for (std::size_t i = 0; i < global_collision_body_transforms_.size(); ++i)
      global_collision_body_transforms_[i] = parent_link_global_transform * attach_trans_[i];
  }

private:
  /** \brief The link that owns this attached body */
  const LinkModel* parent_link_model_;

  /** \brief string id for reference */
  std::string id_;

  /** \brief The geometries of the attached body */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /** \brief The constant transforms applied to the link (needs to be specified by user) */
  EigenSTL::vector_Affine3d attach_trans_;

  /** \brief The set of links this body is allowed to touch */
  std::set<std::string> touch_links_;

  /** \brief Posture of links for releasing the object (if any). This is useful for example when storing
      the configuration of a gripper holding an object */
  trajectory_msgs::JointTrajectory detach_posture_;

  /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
  EigenSTL::vector_Affine3d global_collision_body_transforms_;
};
}
}

#endif
