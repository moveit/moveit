/*******************************************************************************
 * Title     : joint_state_subscriber.h
 * Project   : moveit_servo
 * Created   : 06/11/2020
 * Author    : Tyler Weaver
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit_servo/servo_parameters.h>

namespace moveit_servo
{
class JointStateSubscriber
{
public:
  /** \brief Constructor */
  JointStateSubscriber(ros::NodeHandle& nh, const std::string& joint_state_topic_name);

  /** \brief Get the latest joint state message */
  sensor_msgs::JointStateConstPtr getLatest() const;

private:
  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);

  ros::Subscriber joint_state_sub_;

  // Latest joint state, updated by ROS callback
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::JointStateConstPtr latest_joint_state_;
};
}  // namespace moveit_servo
