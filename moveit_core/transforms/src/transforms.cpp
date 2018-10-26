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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/transforms/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string/trim.hpp>
#include <ros/console.h>

namespace moveit
{
namespace core
{
Transforms::Transforms(const std::string& target_frame) : target_frame_(target_frame)
{
  boost::trim(target_frame_);
  if (target_frame_.empty())
    ROS_ERROR_NAMED("transforms", "The target frame for MoveIt! Transforms cannot be empty.");
  else
  {
    if (target_frame_[0] != '/')
    {
      ROS_WARN_NAMED("transforms",
                     "Frame '%s' specified as target frame for MoveIt! Transforms. Assuming '/%s' instead.",
                     target_frame_.c_str(), target_frame_.c_str());
      target_frame_ = '/' + target_frame_;
    }
    transforms_[target_frame_] = Eigen::Affine3d::Identity();
  }
}

bool Transforms::sameFrame(const std::string& frame1, const std::string& frame2)
{
  if (frame1.empty() || frame2.empty())
    return false;
  if (frame1[0] != '/')
    return sameFrame('/' + frame1, frame2);
  if (frame2[0] != '/')
    return sameFrame(frame1, '/' + frame2);
  return frame1 == frame2;
}

Transforms::~Transforms() = default;

const std::string& Transforms::getTargetFrame() const
{
  return target_frame_;
}

const FixedTransformsMap& Transforms::getAllTransforms() const
{
  return transforms_;
}

void Transforms::setAllTransforms(const FixedTransformsMap& transforms)
{
  transforms_ = transforms;
}

bool Transforms::isFixedFrame(const std::string& frame) const
{
  if (frame.empty())
    return false;
  else
    return (frame[0] == '/' ? transforms_.find(frame) : transforms_.find('/' + frame)) != transforms_.end();
}

const Eigen::Affine3d& Transforms::getTransform(const std::string& from_frame) const
{
  if (!from_frame.empty())
  {
    FixedTransformsMap::const_iterator it =
        (from_frame[0] == '/' ? transforms_.find(from_frame) : transforms_.find('/' + from_frame));
    if (it != transforms_.end())
      return it->second;
  }

  ROS_ERROR_NAMED("transforms", "Unable to transform from frame '%s' to frame '%s'. Returning identity.",
                  from_frame.c_str(), target_frame_.c_str());

  // return identity
  static const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  return identity;
}

bool Transforms::canTransform(const std::string& from_frame) const
{
  if (from_frame.empty())
    return false;
  else
    return (from_frame[0] == '/' ? transforms_.find(from_frame) : transforms_.find('/' + from_frame)) !=
           transforms_.end();
}

void Transforms::setTransform(const Eigen::Affine3d& t, const std::string& from_frame)
{
  if (from_frame.empty())
    ROS_ERROR_NAMED("transforms", "Cannot record transform with empty name");
  else
  {
    if (from_frame[0] != '/')
    {
      ROS_WARN_NAMED("transforms", "Transform specified for frame '%s'. Assuming '/%s' instead", from_frame.c_str(),
                     from_frame.c_str());
      transforms_['/' + from_frame] = t;
    }
    else
      transforms_[from_frame] = t;
  }
}

void Transforms::setTransform(const geometry_msgs::TransformStamped& transform)
{
  if (sameFrame(transform.child_frame_id, target_frame_))
  {
    Eigen::Affine3d t;
    tf::transformMsgToEigen(transform.transform, t);
    setTransform(t, transform.header.frame_id);
  }
  else
  {
    ROS_ERROR_NAMED("transforms", "Given transform is to frame '%s', but frame '%s' was expected.",
                    transform.child_frame_id.c_str(), target_frame_.c_str());
  }
}

void Transforms::setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
  for (std::size_t i = 0; i < transforms.size(); ++i)
    setTransform(transforms[i]);
}

void Transforms::copyTransforms(std::vector<geometry_msgs::TransformStamped>& transforms) const
{
  transforms.resize(transforms_.size());
  std::size_t i = 0;
  for (FixedTransformsMap::const_iterator it = transforms_.begin(); it != transforms_.end(); ++it, ++i)
  {
    transforms[i].child_frame_id = target_frame_;
    transforms[i].header.frame_id = it->first;
    tf::transformEigenToMsg(it->second, transforms[i].transform);
  }
}

}  // end of namespace core
}  // end of namespace moveit