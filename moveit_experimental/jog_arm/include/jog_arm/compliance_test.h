///////////////////////////////////////////////////////////////////////////////
//      Title     : compliance_test.h
//      Project   : compliance_test
//      Created   : 4/2/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

// Demonstrate compliance on a stationary robot. The robot should act like a
// spring
// when pushed.

#ifndef COMPLIANCE_TEST_H
#define COMPLIANCE_TEST_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <jog_arm/compliant_control.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace compliance_test
{
class ComplianceClass
{
public:
  ComplianceClass();

private:
  // CB for halt warnings from the jog_arm nodes
  void haltCB(const std_msgs::Bool::ConstPtr& msg);

  // CB for force/torque data
  void ftCB(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Transform a wrench to the EE frame
  geometry_msgs::WrenchStamped transformToEEF(const geometry_msgs::WrenchStamped wrench_in,
                                              const std::string desired_ee_frame);

  ros::NodeHandle n_;

  ros::AsyncSpinner spinner_;

  // Publish a velocity cmd to the jog_arm node
  ros::Publisher vel_pub_;

  ros::Subscriber jog_arm_warning_sub_, ft_sub_;

  geometry_msgs::WrenchStamped ft_data_;

  // Did one of the jog nodes halt motion?
  bool jog_is_halted_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // end namespace compliance_test

#endif