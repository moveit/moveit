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

/* Author: E. Gil Jones */

#ifndef MOVEIT_DISTANCE_FIELD_DISTANCE_FIELD_COMMON_
#define MOVEIT_DISTANCE_FIELD_DISTANCE_FIELD_COMMON_

#include <eigen_stl_containers/eigen_stl_containers.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>

namespace distance_field
{

/**
 * \brief Determines a set of obstacle points given a body and a particular resolution.
 *
 * This function computes a bounding sphere and iterates through it in
 * 3D at the specified resolution.  It tests each iterated for
 * inclusion in the body, returning the set of points that are
 * reported as being included in the body.
 *
 * @param [in] body The body to discretize
 * @param [in] resolution The resolution at which to test
 *
 * @return The set of points in the bounding sphere that return as
 * included in the body
 */
EigenSTL::vector_Vector3d static inline determineCollisionPoints(const bodies::Body* body, double resolution)
{
  EigenSTL::vector_Vector3d ret_vec;
  if(!body) return ret_vec;
  bodies::BoundingSphere sphere;
  body->computeBoundingSphere(sphere);
  //ROS_INFO_STREAM("Radius is " << sphere.radius);
  //ROS_INFO_STREAM("Center is " << sphere.center.z() << " " << sphere.center.y() << " " << sphere.center.z());
  for(double xval = sphere.center.x()-sphere.radius-resolution; xval <= sphere.center.x()+sphere.radius+resolution; xval += resolution) {
    for(double yval = sphere.center.y()-sphere.radius-resolution; yval <= sphere.center.y()+sphere.radius+resolution; yval += resolution) {
      for(double zval = sphere.center.z()-sphere.radius-resolution; zval <= sphere.center.z()+sphere.radius+resolution; zval += resolution) {
        Eigen::Vector3d rel_vec(xval, yval, zval);
        if(body->containsPoint(rel_vec)) {
          ret_vec.push_back(rel_vec);
        }
      }
    }
  }
  return ret_vec;
}

}

#endif
