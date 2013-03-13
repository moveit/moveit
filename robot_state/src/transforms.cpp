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

/* Author: Ioan Sucan */

#include <moveit/robot_state/transforms.h>
#include <eigen_conversions/eigen_msg.h>

robot_state::Transforms::Transforms(const std::string &target_frame) : target_frame_(target_frame)
{
  Eigen::Affine3d t;
  t.setIdentity();
  transforms_[target_frame_] = t;
}

robot_state::Transforms::Transforms(const Transforms &other) : target_frame_(other.target_frame_), transforms_(other.transforms_)
{
}

robot_state::Transforms::~Transforms()
{
}

const std::string& robot_state::Transforms::getTargetFrame() const
{
  return target_frame_;
}

const robot_state::FixedTransformsMap& robot_state::Transforms::getAllTransforms() const
{
  return transforms_;
}

bool robot_state::Transforms::isFixedFrame(const std::string &frame) const
{
  return transforms_.find(frame) != transforms_.end();
}

const Eigen::Affine3d& robot_state::Transforms::getTransform(const std::string &from_frame) const
{
  FixedTransformsMap::const_iterator it = transforms_.find(from_frame);
  if (it != transforms_.end())
    return it->second;
  logError("Unable to transform from frame '%s' to frame '%s'", from_frame.c_str(), target_frame_.c_str());
  // return identity
  return transforms_.find(target_frame_)->second;
}

const Eigen::Affine3d& robot_state::Transforms::getTransform(const RobotState& kstate, const std::string &from_frame) const
{
  FixedTransformsMap::const_iterator it = transforms_.find(from_frame);
  if (it != transforms_.end())
    return it->second;
  if (kstate.getRobotModel()->getModelFrame() != target_frame_)
    logError("Target frame is assumed to be '%s' but the model of the kinematic state places the robot in frame '%s'",
             target_frame_.c_str(), kstate.getRobotModel()->getModelFrame().c_str());
  return kstate.getFrameTransform(from_frame);
}

void robot_state::Transforms::transformVector3(const std::string &from_frame, const Eigen::Vector3d &v_in, Eigen::Vector3d &v_out) const
{
  v_out = getTransform(from_frame).rotation() * v_in;
}

void robot_state::Transforms::transformQuaternion(const std::string &from_frame, const Eigen::Quaterniond &q_in, Eigen::Quaterniond &q_out) const
{
  q_out = getTransform(from_frame).rotation() * q_in;
}

void robot_state::Transforms::transformRotationMatrix(const std::string &from_frame, const Eigen::Matrix3d &m_in, Eigen::Matrix3d &m_out) const
{
  m_out = getTransform(from_frame).rotation() * m_in;
}

void robot_state::Transforms::transformPose(const std::string &from_frame, const Eigen::Affine3d &t_in, Eigen::Affine3d &t_out) const
{
  t_out = getTransform(from_frame) * t_in;
}

// specify the kinematic state
void robot_state::Transforms::transformVector3(const RobotState& kstate, const std::string &from_frame,
                                               const Eigen::Vector3d &v_in, Eigen::Vector3d &v_out) const
{
  v_out = getTransform(kstate, from_frame).rotation() * v_in;
}

void robot_state::Transforms::transformQuaternion(const RobotState& kstate, const std::string &from_frame,
                                                  const Eigen::Quaterniond &q_in, Eigen::Quaterniond &q_out) const
{
  q_out = getTransform(kstate, from_frame).rotation() * q_in;
}

void robot_state::Transforms::transformRotationMatrix(const RobotState& kstate, const std::string &from_frame,
                                                      const Eigen::Matrix3d &m_in, Eigen::Matrix3d &m_out) const
{
  m_out = getTransform(kstate, from_frame).rotation() * m_in;
}

void robot_state::Transforms::transformPose(const RobotState& kstate, const std::string &from_frame,
                                            const Eigen::Affine3d &t_in, Eigen::Affine3d &t_out) const
{
  t_out = getTransform(kstate, from_frame) * t_in;
}

void robot_state::Transforms::setTransform(const Eigen::Affine3d &t, const std::string &from_frame)
{
  transforms_[from_frame] = t;
}

void robot_state::Transforms::setTransform(const geometry_msgs::TransformStamped &transform)
{
  if (transform.child_frame_id.rfind(target_frame_) == transform.child_frame_id.length() - target_frame_.length())
  {
    Eigen::Translation3d o(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(transform.transform.rotation, q);
    setTransform(Eigen::Affine3d(o*q.toRotationMatrix()), transform.header.frame_id);
  } else {
    logError("Given transform is to frame '%s', but frame '%s' was expected.", transform.child_frame_id.c_str(), target_frame_.c_str());
  }
}

void robot_state::Transforms::setTransforms(const std::vector<geometry_msgs::TransformStamped> &transforms)
{
  for (std::size_t i = 0 ; i < transforms.size() ; ++i)
    setTransform(transforms[i]);
}

void robot_state::Transforms::getTransforms(std::vector<geometry_msgs::TransformStamped> &transforms) const
{
  transforms.resize(transforms_.size());
  std::size_t i = 0;
  for (FixedTransformsMap::const_iterator it = transforms_.begin() ; it != transforms_.end() ; ++it, ++i)
  {
    transforms[i].child_frame_id = target_frame_;
    transforms[i].header.frame_id = it->first;
    transforms[i].transform.translation.x = it->second.translation().x();
    transforms[i].transform.translation.y = it->second.translation().y();
    transforms[i].transform.translation.z = it->second.translation().z();
    Eigen::Quaterniond q(it->second.rotation());
    transforms[i].transform.rotation.x = q.x();
    transforms[i].transform.rotation.y = q.y();
    transforms[i].transform.rotation.z = q.z();
    transforms[i].transform.rotation.w = q.w();
  }
}
