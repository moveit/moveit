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

/*------------------------------------------------------*/
/*   DO NOT INCLUDE THIS FILE DIRECTLY                  */
/*------------------------------------------------------*/

/** @brief Object defining bodies that can be attached to robot
 *  links. This is useful when handling objects picked up by
 *  the robot. */
class AttachedBody
{
  friend class KinematicState;
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /** \brief Construct an attached body for a specified \e link. The name of this body is \e id and it consists of \e shapes that
      attach to the link by the transforms \e attach_trans. The set of links that are allowed to be touched by this object is specified by \e touch_links. */
  AttachedBody(const LinkState *link, const std::string &id,
               const std::vector<shapes::ShapeConstPtr> &shapes,
               const std::vector<Eigen::Affine3d> &attach_trans,
               const std::vector<std::string> &touch_links);
  
  ~AttachedBody(void);
  
  /** \brief Get the name of the attached body */
  const std::string& getName(void) const
  {
    return id_;
  }
  
  /** \brief Get the name of the link this body is attached to */
  const std::string& getAttachedLinkName(void) const
  {
    return parent_link_state_->getName();
  }
  
  /** \brief Get the shapes that make up this attached body */
  const std::vector<shapes::ShapeConstPtr>& getShapes(void) const
  {
    return shapes_;
  }
  
  /** \brief Get the fixed transform (the transforms to the shapes associated with this body) */
  
  /** \brief Get the links that the attached body is allowed to touch */
  const std::set<std::string>& getTouchLinks(void) const
  {
    return touch_links_;
  }
  
  const std::vector<Eigen::Affine3d>& getFixedTransforms(void) const
  {
    return attach_trans_;
  }
  
  /** \brief Get the global transforms for the collision bodies */
  const std::vector<Eigen::Affine3d>& getGlobalCollisionBodyTransforms(void) const
  {
    return global_collision_body_transforms_;
  }
  
  /** \brief Set the padding for the shapes of this attached object */
  void setPadding(double padding);
  
  /** \brief Set the scale for the shapes of this attached object */
  void setScale(double scale);
  
  /** \brief Recompute global_collision_body_transform */
  void computeTransform(void);
  
private:
  
  /** \brief The link that owns this attached body */
  const LinkState                   *parent_link_state_;
  
  /** \brief string id for reference */
  std::string                        id_;
  
  /** \brief The geometries of the attached body */
  std::vector<shapes::ShapeConstPtr> shapes_;
  
  /** \brief The set of links this body is allowed to touch */
  std::set<std::string>              touch_links_;
  
  /** \brief The constant transforms applied to the link (needs to be specified by user) */
  std::vector<Eigen::Affine3d>       attach_trans_;
  
  /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
  std::vector<Eigen::Affine3d>       global_collision_body_transforms_;
};

