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

#ifndef GEOMETRIC_SHAPES_SHAPE_MESSAGES_
#define GEOMETRIC_SHAPES_SHAPE_MESSAGES_

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>
#include <boost/variant.hpp>

namespace shapes
{

/** \brief Type that can hold any of the desired shape message types */
typedef boost::variant<shape_msgs::SolidPrimitive, shape_msgs::Mesh, shape_msgs::Plane> ShapeMsg;
template<int>
struct SolidPrimitiveDimCount
{
  enum { value = 0 };
};

template<>
struct SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>
{
  enum
    {
      value = static_cast<int>(shape_msgs::SolidPrimitive::SPHERE_RADIUS) + 1
    };
};

template<>
struct SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>
{
  enum
    {
      value = (static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) && static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z)) ?
      static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) :
      (((static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) && static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z))) ?
       static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) :
       ((static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) && static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y)) ?
        static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) : 0)) + 1
    };
};

template<>
struct SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>
{
  enum
    {
      value = (static_cast<int>(shape_msgs::SolidPrimitive::CONE_RADIUS) >= static_cast<int>(shape_msgs::SolidPrimitive::CONE_HEIGHT) ?
               static_cast<int>(shape_msgs::SolidPrimitive::CONE_RADIUS) : static_cast<int>(shape_msgs::SolidPrimitive::CONE_HEIGHT)) + 1
    };
};

template<>
struct SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>
{
  enum
    {
      value = (static_cast<int>(shape_msgs::SolidPrimitive::CYLINDER_RADIUS) >= static_cast<int>(shape_msgs::SolidPrimitive::CYLINDER_HEIGHT) ?
               static_cast<int>(shape_msgs::SolidPrimitive::CYLINDER_RADIUS) : static_cast<int>(shape_msgs::SolidPrimitive::CYLINDER_HEIGHT)) + 1
    };
};

}

#endif
