/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#include <moveit/point_containment_filter/shape_mask.h>
#include <geometric_shapes/body_operations.h>
#include <ros/console.h>

point_containment_filter::ShapeMask::ShapeMask(const TransformCallback& transform_callback) :
  transform_callback_(transform_callback),
  next_handle_ (1),
  min_handle_ (1)
{
}

point_containment_filter::ShapeMask::~ShapeMask()
{
  freeMemory();
}

void point_containment_filter::ShapeMask::freeMemory()
{   
  for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() ; ++it)
    delete it->body;
  bodies_.clear();
}

void point_containment_filter::ShapeMask::setTransformCallback(const TransformCallback& transform_callback)
{
  transform_callback_ = transform_callback;
}

point_containment_filter::ShapeHandle point_containment_filter::ShapeMask::addShape(const shapes::ShapeConstPtr &shape, double scale, double padding)
{
  SeeShape ss;
  ss.body = bodies::createBodyFromShape(shape.get());
  if (ss.body)
  {
    ss.body->setScale(scale);
    ss.body->setPadding(padding);
    ss.volume = ss.body->computeVolume();
    ss.handle = next_handle_;
    used_handles_[next_handle_] = bodies_.insert(ss).first;
  }
  else
    return 0;
  
  ShapeHandle ret = next_handle_;
  const std::size_t sz = min_handle_ + bodies_.size() + 1;
  for (std::size_t i = min_handle_ ; i < sz ; ++i)
    if (used_handles_.find(i) == used_handles_.end())
    {
      next_handle_ = i;
      break;
    }
  min_handle_ = next_handle_;
  
  return ret;
}

void point_containment_filter::ShapeMask::removeShape(ShapeHandle handle)
{
  std::map<ShapeHandle, std::set<SeeShape, SortBodies>::iterator>::iterator it = used_handles_.find(handle);
  if (it != used_handles_.end())
  {
    delete it->second->body;
    bodies_.erase(it->second);
    used_handles_.erase(it);
    min_handle_ = handle;
  }
}

void point_containment_filter::ShapeMask::maskContainment(const pcl::PointCloud<pcl::PointXYZ>& data_in,
                                                          const Eigen::Vector3d &sensor_origin,
                                                          const double min_sensor_dist, const double max_sensor_dist, 
                                                          std::vector<int> &mask)
{
  mask.resize(data_in.points.size());
  if (bodies_.empty())
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  else
  {   
    Eigen::Affine3d tmp;
    bspheres_.resize(bodies_.size());
    std::size_t j = 0;
    for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() ; ++it)
    {
      if (transform_callback_(it->handle, tmp))
      {        
        it->body->setPose(tmp);
        it->body->computeBoundingSphere(bspheres_[j++]);
      }
    }
    
    const unsigned int np = data_in.points.size();
    
    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    const double radiusSquared = bound.radius * bound.radius;
    
    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic)
    for (int i = 0 ; i < (int)np ; ++i)
    {
      Eigen::Vector3d pt = Eigen::Vector3d(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;
      if ((bound.center - pt).squaredNorm() < radiusSquared)
      {
        double d = (sensor_origin - pt).norm();
        if (d < min_sensor_dist || d > max_sensor_dist)
        {
          out = CLIP;
        }
        else
          for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() && out == OUTSIDE ; ++it)
          {
            if (it->body->containsPoint(pt))
            {
              out = INSIDE;
            }
          }
      }
      mask[i] = out;
    }    
  }
}

int point_containment_filter::ShapeMask::getMaskContainment(const Eigen::Vector3d &pt) const
{
  int out = OUTSIDE;        
  for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() && out == OUTSIDE ; ++it)
    if (it->body->containsPoint(pt))
      out = INSIDE;
  return out;
}

int point_containment_filter::ShapeMask::getMaskContainment(double x, double y, double z) const
{
  return getMaskContainment(Eigen::Vector3d(x, y, z));
}
