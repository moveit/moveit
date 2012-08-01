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

/* Author: Ioan Sucan, E. Gil Jones */

#ifndef GEOMETRIC_SHAPES_BODY_OPERATIONS_
#define GEOMETRIC_SHAPES_BODY_OPERATIONS_

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/bodies.h"
#include "geometric_shapes/shape_messages.h"
#include <geometry_msgs/Pose.h>
#include <vector>

namespace bodies
{

/** \brief Create a body from a given shape */
Body* createBodyFromShape(const shapes::Shape *shape);

/** \brief Create a body from a given shape */
Body* constructBodyFromMsg(const shape_msgs::Mesh &shape, const geometry_msgs::Pose &pose);

/** \brief Create a body from a given shape */
Body* constructBodyFromMsg(const shape_msgs::SolidPrimitive &shape, const geometry_msgs::Pose &pose);

/** \brief Create a body from a given shape */
Body* constructBodyFromMsg(const shapes::ShapeMsg &shape, const geometry_msgs::Pose &pose);

/** \brief Compute a bounding sphere to enclose a set of bounding spheres */
void mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere);

/** \brief Compute the bounding sphere for a set of \e bodies and store the resulting sphere in \e mergedSphere */
void computeBoundingSphere(const std::vector<const Body*>& bodies, BoundingSphere &mergedSphere);

}
#endif
