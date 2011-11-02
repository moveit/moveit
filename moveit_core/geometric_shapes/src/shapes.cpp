/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/** \author Ioan Sucan */

#include "geometric_shapes/shapes.h"
#include <ros/console.h>

shapes::ShapeVector::ShapeVector(void)
{
}

shapes::ShapeVector::~ShapeVector(void)
{
    clear();
}

void shapes::ShapeVector::addShape(Shape* shape)
{
    shapes_.push_back(shape);
}

void shapes::ShapeVector::addShape(StaticShape* shape)
{
    sshapes_.push_back(shape);
}

void shapes::ShapeVector::clear(void)
{
    for (std::size_t i = 0 ; i < shapes_.size() ; ++i)
        delete shapes_[i];
    shapes_.clear();
    for (std::size_t i = 0 ; i < sshapes_.size() ; ++i)
        delete sshapes_[i];
    sshapes_.clear();
}

std::size_t shapes::ShapeVector::getCount(void) const
{
    return shapes_.size();
}

std::size_t shapes::ShapeVector::getStaticCount(void) const
{
    return sshapes_.size();
}

const shapes::Shape* shapes::ShapeVector::getShape(unsigned int i) const
{
    if (i >= shapes_.size())
    {
        ROS_ERROR("There is no shape at index %u", i);
        return NULL;
    }
    return shapes_[i];
}

const shapes::StaticShape* shapes::ShapeVector::getStaticShape(unsigned int i) const
{
    if (i >= sshapes_.size())
    {
        ROS_ERROR("There is no static shape at index %u", i);
        return NULL;
    }
    return sshapes_[i];
}
