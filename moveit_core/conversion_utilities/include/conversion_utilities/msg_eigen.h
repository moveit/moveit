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

#ifndef _MSG_EIGEN_H_
#define _MSG_EIGEN_H_

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>  // TODO
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>

namespace conversion_utilities
{

namespace msg
{

  void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Matrix<double,3,1> &e)
  {
    e[0] = m.x;
    e[1] = m.y;
    e[2] = m.z;
  }

  Eigen::Matrix<double,3,1> pointMsgToEigen(const geometry_msgs::Point &m)
  {
    return Eigen::Matrix<double,3,1>(m.x, m.y, m.z);
    //e[0] = m.x;
    //e[1] = m.y;
    //e[2] = m.z;
  }

  void pointEigenToMsg(const Eigen::Matrix<double,3,1> &e, geometry_msgs::Point &m )
  {
    m.x = e[0];
    m.y = e[1];
    m.z = e[2];
  }

  geometry_msgs::Point pointEigenToMsg(const Eigen::Matrix<double,3,1> &e )
  {
    geometry_msgs::Point m;
    m.x = e[0];
    m.y = e[1];
    m.z = e[2];
    return m;
  }

  void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e)
  {
    e = Eigen::Translation3d(m.position.x,
                             m.position.y,
                             m.position.z) *
      Eigen::Quaterniond(m.orientation.w,
                         m.orientation.x,
                         m.orientation.y,
                         m.orientation.z);
  }

  void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m)
  {
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0) {
      m.orientation.x *= -1;
      m.orientation.y *= -1;
      m.orientation.z *= -1;
      m.orientation.w *= -1;
    }
  }

  void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e)
  {
    e = Eigen::Quaterniond( m.w, m.x, m.y, m.z);
  }

  void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::Quaternion &m )
  {
    m.x = e.x();
    m.y = e.y();
    m.z = e.z();
    m.w = e.w();
  }

  void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Affine3d &e)
  {
    e = Eigen::Translation3d(m.translation.x,
                             m.translation.y,
                             m.translation.z) *
      Eigen::Quaterniond(m.rotation.w,
                         m.rotation.x,
                         m.rotation.y,
                         m.rotation.z);
  }

  void transformEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Transform &m)
  {
    m.translation.x = e.translation()[0];
    m.translation.y = e.translation()[1];
    m.translation.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    m.rotation.w = q.w();
    if (m.rotation.w < 0) {
      m.rotation.x *= -1;
      m.rotation.y *= -1;
      m.rotation.z *= -1;
      m.rotation.w *= -1;
    }
  }

  void twistMsgToEigen(const geometry_msgs::Twist &m, Eigen::Matrix<double,6,1> &e)
  {
    e[0] = m.linear.x;
    e[1] = m.linear.y;
    e[2] = m.linear.z;
    e[3] = m.angular.x;
    e[4] = m.angular.y;
    e[5] = m.angular.z;
  }

  void twistEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Twist &m)
  {
    m.linear.x = e[0];
    m.linear.y = e[1];
    m.linear.z = e[2];
    m.angular.x = e[3];
    m.angular.y = e[4];
    m.angular.z = e[5];
  }

  void vectorMsgToEigen(const geometry_msgs::Vector3 &m, Eigen::Matrix<double,3,1> &e)
  {
    e[0] = m.x;
    e[1] = m.y;
    e[2] = m.z;
  }

  void vectorEigenToMsg(const Eigen::Matrix<double,3,1> &e, geometry_msgs::Vector3 &m )
  {
    m.x = e[0];
    m.y = e[1];
    m.z = e[2];
  }


  void wrenchMsgToEigen(const geometry_msgs::Wrench &m, Eigen::Matrix<double,6,1> &e)
  {
    e[0] = m.force.x;
    e[1] = m.force.y;
    e[2] = m.force.z;
    e[3] = m.torque.x;
    e[4] = m.torque.y;
    e[5] = m.torque.z;
  }

  void wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Wrench &m)
  {
    m.force.x = e[0];
    m.force.y = e[1];
    m.force.z = e[2];
    m.torque.x = e[3];
    m.torque.y = e[4];
    m.torque.z = e[5];
  }

  /// Converts an Eigen matrix into a Float64MultiArray message
  template <class Derived>
  void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
  {
    if (m.layout.dim.size() != 2)
      m.layout.dim.resize(2);
    m.layout.dim[0].stride = e.rows() * e.cols();
    m.layout.dim[0].size = e.rows();
    m.layout.dim[1].stride = e.cols();
    m.layout.dim[1].size = e.cols();
    if ((int)m.data.size() != e.size())
      m.data.resize(e.size());
    int ii = 0;
    for (int i = 0; i < e.rows(); ++i)
      for (int j = 0; j < e.cols(); ++j)
        m.data[ii++] = e.coeff(i, j);
  }

} //namespace msg


} // namespace conversion_utilities

#endif
