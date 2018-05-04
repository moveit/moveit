/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_state/attached_body.h>
#include <geometric_shapes/shapes.h>

moveit::core::AttachedBody::AttachedBody(const LinkModel* parent_link_model, const std::string& id,
                                         const std::vector<shapes::ShapeConstPtr>& shapes,
                                         const EigenSTL::vector_Affine3d& attach_trans,
                                         const std::set<std::string>& touch_links,
                                         const trajectory_msgs::JointTrajectory& detach_posture)
  : parent_link_model_(parent_link_model)
  , id_(id)
  , shapes_(shapes)
  , attach_trans_(attach_trans)
  , touch_links_(touch_links)
  , detach_posture_(detach_posture)
{
  global_collision_body_transforms_.resize(attach_trans.size());
  for (std::size_t i = 0; i < global_collision_body_transforms_.size(); ++i)
    global_collision_body_transforms_[i].setIdentity();
}

moveit::core::AttachedBody::~AttachedBody() = default;

void moveit::core::AttachedBody::setScale(double scale)
{
  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    // if this shape is only owned here (and because this is a non-const function), we can safely const-cast:
    if (shapes_[i].unique())
      const_cast<shapes::Shape*>(shapes_[i].get())->scale(scale);
    else
    {
      // if the shape is owned elsewhere, we make a copy:
      shapes::Shape* copy = shapes_[i]->clone();
      copy->scale(scale);
      shapes_[i].reset(copy);
    }
  }
}

void moveit::core::AttachedBody::setPadding(double padding)
{
  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    // if this shape is only owned here (and because this is a non-const function), we can safely const-cast:
    if (shapes_[i].unique())
      const_cast<shapes::Shape*>(shapes_[i].get())->padd(padding);
    else
    {
      // if the shape is owned elsewhere, we make a copy:
      shapes::Shape* copy = shapes_[i]->clone();
      copy->padd(padding);
      shapes_[i].reset(copy);
    }
  }
}
