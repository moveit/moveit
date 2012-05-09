/*********************************************************************
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


// Author: Adam E Leeper

#ifndef _MSG_TF_H_
#define _MSG_TF_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>


namespace conversion_utilities
{

namespace msg
{
    // A suite of simple msg "constructor" functions. It would be great if this feature was added to ROS msg generation...
    // Also includes a number of converter functions, for convenience...

    inline std_msgs::ColorRGBA createColorMsg(const float red, const float green, const float blue, const float alpha = 1.0) {
        std_msgs::ColorRGBA color;
        color.r = std::min(red,   1.0f);
        color.g = std::min(green, 1.0f);
        color.b = std::min(blue,  1.0f);
        color.a = alpha;
        return color;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::Vector3 createVector3Msg(const float x, const float y, const float z) {
      geometry_msgs::Vector3 msg;
      msg.x = x;
      msg.y = y;
      msg.z = z;
      return msg;
    }

    inline geometry_msgs::Vector3 createVector3Msg(const tf::Vector3 &v) {
      geometry_msgs::Vector3 msg;
      msg.x = v.x();
      msg.y = v.y();
      msg.z = v.z();
      return msg;
    }

    inline geometry_msgs::Vector3 createVector3Msg(const geometry_msgs::Point &v) {
      geometry_msgs::Vector3 msg;
      msg.x = v.x;
      msg.y = v.y;
      msg.z = v.z;
      return msg;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::Point createPointMsg(const float x, const float y, const float z) {
      geometry_msgs::Point msg;
      msg.x = x;
      msg.y = y;
      msg.z = z;
      return msg;
    }

    inline geometry_msgs::Point createPointMsg(const tf::Point &v) {
      geometry_msgs::Point msg;
      msg.x = v.x();
      msg.y = v.y();
      msg.z = v.z();
      return msg;
    }

    inline geometry_msgs::Point createPointMsg(const geometry_msgs::Vector3 &v) {
      geometry_msgs::Point msg;
      msg.x = v.x;
      msg.y = v.y;
      msg.z = v.z;
      return msg;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::Quaternion createQuaternionMsg(const float x, const float y, const float z, const float w) {
      geometry_msgs::Quaternion msg;
      msg.x = x;
      msg.y = y;
      msg.z = z;
      msg.w = w;
      return msg;
    }

    inline geometry_msgs::Quaternion createQuaternionMsg(const tf::Quaternion &q) {
      geometry_msgs::Quaternion msg;
      msg.x = q.x();
      msg.y = q.y();
      msg.z = q.z();
      msg.w = q.w();
      return msg;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::Pose createPoseMsg(const geometry_msgs::Point &v,  const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Pose msg;
      msg.position = v;
      msg.orientation = q;
      return msg;
    }

    inline geometry_msgs::Pose createPoseMsg(const geometry_msgs::Vector3 &v,  const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Pose msg;
      msg.position = msg::createPointMsg(v.x, v.y, v.z);
      msg.orientation = q;
      return msg;
    }

    inline geometry_msgs::Pose createPoseMsg(const tf::Pose &p)
    {
      geometry_msgs::Pose msg;
      tf::poseTFToMsg(p, msg);
      return msg;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::Pose &p, const std::string &frame_id, const ros::Time &stamp)
    {
      geometry_msgs::PoseStamped msg;
      msg.pose = p;
      msg.header.frame_id = frame_id;
      msg.header.stamp = stamp;
      return msg;
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::Point &v,  const geometry_msgs::Quaternion &q, const std::string &frame_id, const ros::Time &stamp)
    {
      return createPoseStampedMsg( createPoseMsg(v, q), frame_id, stamp );
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::TransformStamped ts)
    {

      return createPoseStampedMsg( createPoseMsg(ts.transform.translation, ts.transform.rotation), ts.header.frame_id, ts.header.stamp );
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const tf::Transform &pose, const std::string &frame_id, const ros::Time &stamp)
    {

      return createPoseStampedMsg( createPoseMsg(pose), frame_id, stamp );
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::Transform createTransformMsg(const geometry_msgs::Point &v, const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Transform msg;
      msg.translation = msg::createVector3Msg(v.x, v.y, v.z);
      msg.rotation = q;
      return msg;
    }

    inline geometry_msgs::Transform createTransformMsg(const geometry_msgs::Vector3 &v, const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Transform msg;
      msg.translation = v;
      msg.rotation = q;
      return msg;
    }

    inline geometry_msgs::TransformStamped createTransformStampedMsg(const geometry_msgs::Transform &t, const std::string &frame_id, const ros::Time &stamp)
    {
      geometry_msgs::TransformStamped msg;
      msg.transform = t;
      msg.header.frame_id = frame_id;
      msg.header.stamp = stamp;
      return msg;
    }

    inline geometry_msgs::TransformStamped createTransformStampedMsg(const geometry_msgs::Vector3 &v,  const geometry_msgs::Quaternion &q, const std::string &frame_id, const ros::Time &stamp)
    {
      return createTransformStampedMsg( createTransformMsg(v, q), frame_id, stamp );
    }

    //! Returns a pose that has been translated along the basis vectors of the quaternion in the input pose.
    inline geometry_msgs::Pose applyShift(const geometry_msgs::Pose &pose, const tf::Vector3 &shift)
    {
      geometry_msgs::Pose out;
      tf::Transform P;
      tf::poseMsgToTF(pose, P);
      tf::Transform T = tf::Transform::getIdentity();
      T.setOrigin(shift.x()*P.getBasis().getColumn(0) +
                  shift.y()*P.getBasis().getColumn(1) +
                  shift.z()*P.getBasis().getColumn(2));
      P = T*P;
      tf::poseTFToMsg(P, out);
      return out;
    }

    inline tf::Matrix3x3 createMatrix(const tf::Vector3 &X, const tf::Vector3 &Y, const tf::Vector3 &Z)
    {
      return tf::Matrix3x3( X.x(), Y.x(), Z.x(),
                          X.y(), Y.y(), Z.y(),
                          X.z(), Y.z(), Z.z() );

    }

    inline void setMatrix(tf::Matrix3x3 &mat, const tf::Vector3 &X, const tf::Vector3 &Y, const tf::Vector3 &Z)
    {
      mat.setValue( X.x(), Y.x(), Z.x(),
                    X.y(), Y.y(), Z.y(),
                    X.z(), Y.z(), Z.z() );

    }

} // namepsace msg

} // namespace conversion_utilities

#endif
