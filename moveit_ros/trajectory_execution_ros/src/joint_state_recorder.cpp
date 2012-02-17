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

/** \author E. Gil Jones */

#include <ros/ros.h>

#include <trajectory_execution_monitor_ros/joint_state_recorder.h>

JointStateTrajectoryRecorder::JointStateTrajectoryRecorder(const std::string& topic_name) :
  TrajectoryRecorder(topic_name)
{
  ros::NodeHandle nh;
  joint_state_subscriber_ = nh.subscribe(topic_name, 25, &JointStateTrajectoryRecorder::jointStateCallback, this);
}

void JointStateTrajectoryRecorder::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state) {
  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;
  for(unsigned int i = 0; i < joint_state->name.size(); i++) {
    joint_positions[joint_state->name[i]] = joint_state->position[i];
    joint_velocities[joint_state->name[i]] = joint_state->velocity[i];
  }
  
  callCallbacks(joint_state->header.stamp,
                joint_positions,
                joint_velocities);
}
