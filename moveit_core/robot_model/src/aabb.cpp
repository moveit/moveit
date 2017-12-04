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

/* Author: Martin Pecka */

#include <moveit/robot_model/aabb.h>

void moveit::core::AABB::extendWithTransformedBox(const Eigen::Affine3d& transform, const Eigen::Vector3d& box)
{
  Eigen::Vector3d center = box / 2.0;
  Eigen::Vector3d maxCorner = transform * center;
  center = -center;
  Eigen::Vector3d minCorner = transform * center;

  // minCorner is the minimum corner and maxCorner is the maximum corner before applying transform;
  // but the transform can make another corner maximum/minimum in the AABB frame, so we need to check for all 8 corners

  // now we walk around all the box's 8 vertices by changing 1 coordinate a time
  Eigen::Vector3d corner = maxCorner;
  extend(corner);
  corner[0] = minCorner[0];
  extend(corner);
  corner[1] = minCorner[1];
  extend(corner);
  corner[2] = minCorner[2];  // corner == minCorner
  extend(corner);

  corner[1] = maxCorner[1];
  extend(corner);
  corner[0] = maxCorner[0];
  extend(corner);
  corner[1] = minCorner[1];
  extend(corner);
  corner[2] = maxCorner[2];
  extend(corner);
}
