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

#include <planning_models/transforms.h>
#include <ros/console.h>

bool planning_models::quatFromMsg(const geometry_msgs::Quaternion &qmsg, Eigen::Quaterniond &q)
{
  q = Eigen::Quaterniond(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
  if (fabs(q.squaredNorm() - 1.0) > 1e-3)
  {
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    return false;
  }
  return true;
}

double planning_models::normalizeAngle(double angle) {

  double ret = angle;

  while(ret < -2*M_PI) {
    ret += 2*M_PI;
  }
  while(ret > 2*M_PI) {
    ret -= 2*M_PI;
  }
  return ret;
}
/*
void planning_models::getEulerAngles(const Eigen::Affine3d& t, double& r, double& p, double& y) 
{
  //taken from pcl - x,y,z convention
  r  = atan2f(t(2,1), t(2,2));
  p = asinf(-t(2,0));
  y   = atan2f(t(1,0), t(0,0));
}
*/

bool planning_models::poseFromMsg(const geometry_msgs::Pose &tmsg, Eigen::Affine3d &t)
{
  Eigen::Quaterniond q; bool r = quatFromMsg(tmsg.orientation, q);
  t = Eigen::Affine3d(Eigen::Translation3d(tmsg.position.x, tmsg.position.y, tmsg.position.z)*q.toRotationMatrix());
  return r;
}

void planning_models::msgFromPose(const Eigen::Affine3d &t, geometry_msgs::Pose &tmsg)
{
  tmsg.position.x = t.translation().x(); tmsg.position.y = t.translation().y(); tmsg.position.z = t.translation().z();
  Eigen::Quaterniond q(t.rotation());
  tmsg.orientation.x = q.x(); tmsg.orientation.y = q.y(); tmsg.orientation.z = q.z(); tmsg.orientation.w = q.w();
}

planning_models::Transforms::Transforms(const std::string &target_frame) : target_frame_(target_frame)
{
  Eigen::Affine3d t;
  t.setIdentity();
  transforms_[target_frame_] = t;
}

planning_models::Transforms::Transforms(const Transforms &other) : target_frame_(other.target_frame_), transforms_(other.transforms_)
{
}

planning_models::Transforms::~Transforms(void)
{
}

const std::string& planning_models::Transforms::getTargetFrame(void) const
{
  return target_frame_;
}

const planning_models::EigenAffine3dMapType& planning_models::Transforms::getAllTransforms(void) const
{
  return transforms_;
}

bool planning_models::Transforms::isFixedFrame(const std::string &frame) const
{
  return transforms_.find(frame) != transforms_.end();
}

const Eigen::Affine3d& planning_models::Transforms::getTransform(const std::string &from_frame) const
{
  EigenAffine3dMapType::const_iterator it = transforms_.find(from_frame);
  if (it != transforms_.end())
    return it->second;
  ROS_ERROR_STREAM("Unable to transform from frame '" + from_frame + "' to frame '" + target_frame_ + "'");
  // return identity
  return transforms_.find(target_frame_)->second;
}

const Eigen::Affine3d& planning_models::Transforms::getTransform(const planning_models::KinematicState &kstate, const std::string &from_frame) const
{
  std::map<std::string, Eigen::Affine3d>::const_iterator it = transforms_.find(from_frame);
  if (it != transforms_.end())
    return it->second;
  if (kstate.getKinematicModel()->getModelFrame() != target_frame_)
    ROS_ERROR("Target frame is assumed to be '%s' but the model of the kinematic state places the robot in frame '%s'",
              target_frame_.c_str(), kstate.getKinematicModel()->getModelFrame().c_str());
  if (const planning_models::KinematicState::LinkState *state = kstate.getLinkState(from_frame))
    return state->getGlobalLinkTransform();
  ROS_ERROR_STREAM("Unable to transform from frame '" + from_frame + "' to frame '" + target_frame_ + "'");
  // return identity
  return transforms_.find(target_frame_)->second;
}

void planning_models::Transforms::transformVector3(const std::string &from_frame, const Eigen::Vector3d &v_in, Eigen::Vector3d &v_out) const
{
  v_out = getTransform(from_frame) * v_in;
}

void planning_models::Transforms::transformQuaternion(const std::string &from_frame, const Eigen::Quaterniond &q_in, Eigen::Quaterniond &q_out) const
{
  q_out = getTransform(from_frame).rotation() * q_in;
}

void planning_models::Transforms::transformRotationMatrix(const std::string &from_frame, const Eigen::Matrix3d &m_in, Eigen::Matrix3d &m_out) const
{
  m_out = getTransform(from_frame).rotation() * m_in;
}

void planning_models::Transforms::transformPose(const std::string &from_frame, const Eigen::Affine3d &t_in, Eigen::Affine3d &t_out) const
{
  ROS_INFO_STREAM("Get transform x " << getTransform(from_frame).translation().x());
  t_out = getTransform(from_frame) * t_in;
}

// specify the kinematic state
void planning_models::Transforms::transformVector3(const planning_models::KinematicState &kstate,
                                                   const std::string &from_frame, const Eigen::Vector3d &v_in, Eigen::Vector3d &v_out) const
{
  v_out = getTransform(kstate, from_frame).rotation() * v_in;
}

void planning_models::Transforms::transformQuaternion(const planning_models::KinematicState &kstate,
                                                      const std::string &from_frame, const Eigen::Quaterniond &q_in, Eigen::Quaterniond &q_out) const
{
  q_out = getTransform(kstate, from_frame).rotation() * q_in;
}

void planning_models::Transforms::transformRotationMatrix(const planning_models::KinematicState &kstate,
                                                          const std::string &from_frame, const Eigen::Matrix3d &m_in, Eigen::Matrix3d &m_out) const
{
  m_out = getTransform(kstate, from_frame).rotation() * m_in;
}

void planning_models::Transforms::transformPose(const planning_models::KinematicState &kstate,
                                                const std::string &from_frame, const Eigen::Affine3d &t_in, Eigen::Affine3d &t_out) const
{
  t_out = getTransform(kstate, from_frame) * t_in;
}

void planning_models::Transforms::setTransform(const Eigen::Affine3d &t, const std::string &from_frame)
{
  transforms_[from_frame] = t;
}

void planning_models::Transforms::setTransform(const geometry_msgs::TransformStamped &transform)
{
  if (transform.child_frame_id.rfind(target_frame_) == transform.child_frame_id.length() - target_frame_.length())
  {
    Eigen::Translation3d o(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    Eigen::Quaterniond q;
    quatFromMsg(transform.transform.rotation, q);
    setTransform(Eigen::Affine3d(o*q.toRotationMatrix()), transform.header.frame_id);
  } else {
    ROS_ERROR("Given transform is to frame '%s', but frame '%s' was expected.", transform.child_frame_id.c_str(), target_frame_.c_str());
  }
}

void planning_models::Transforms::setTransforms(const std::vector<geometry_msgs::TransformStamped> &transforms)
{
  for (std::size_t i = 0 ; i < transforms.size() ; ++i)
    setTransform(transforms[i]);
}

void planning_models::Transforms::getTransforms(std::vector<geometry_msgs::TransformStamped> &transforms) const
{
  transforms.resize(transforms_.size());
  std::size_t i = 0;
  for (EigenAffine3dMapType::const_iterator it = transforms_.begin() ; it != transforms_.end() ; ++it, ++i)
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
