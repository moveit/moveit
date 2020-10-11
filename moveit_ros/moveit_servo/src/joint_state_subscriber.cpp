/*******************************************************************************
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

/*      Title     : joint_state_subscriber.cpp
 *      Project   : moveit_servo
 *      Created   : 06/11/2020
 *      Author    : Tyler Weaver
 */

#include <moveit_servo/joint_state_subscriber.h>

namespace moveit_servo
{
constexpr char LOGNAME[] = "joint_state_subscriber";

// Constructor for the class that handles collision checking
JointStateSubscriber::JointStateSubscriber(ros::NodeHandle& nh, const std::string& joint_state_topic_name)
{
  // subscribe to joints
  joint_state_sub_ = nh.subscribe(joint_state_topic_name, ROS_QUEUE_SIZE, &JointStateSubscriber::jointStateCB, this);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Wait for initial messages
  ROS_INFO_NAMED(LOGNAME, "Waiting for first joint msg.");
  while (latest_joint_state_ == nullptr)
  {
    ros::Duration(0.01).sleep();
  }
  ROS_INFO_NAMED(LOGNAME, "Received first joint msg.");
}

sensor_msgs::JointStateConstPtr JointStateSubscriber::getLatest() const
{
  const std::lock_guard<std::mutex> lock(joint_state_mutex_);
  return latest_joint_state_;
}

void JointStateSubscriber::jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
}

}  // namespace moveit_servo
