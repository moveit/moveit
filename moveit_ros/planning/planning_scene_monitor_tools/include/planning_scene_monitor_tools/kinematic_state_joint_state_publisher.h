/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: E. Gil Jones

#ifndef MOVEIT_ROBOT_STATE_JOINT_STATE_PUBLISHER_H_
#define MOVEIT_ROBOT_STATE_JOINT_STATE_PUBLISHER_H_

#include <ros/ros.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

class RobotStateJointStatePublisher 
{

public:

  RobotState *JointStatePublisher(const std::string& joint_state_topic = "/joint_states")
  {
    
    joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 10);
  };

  void publishRobotState(const planning_models::RobotState& state)
  {  
    sensor_msgs::JointState js;
    state.getStateValues(js);
    js.header.stamp = ros::Time(ros::WallTime::now().toSec());
    joint_state_publisher_.publish(js);
  }

  void broadcastRootTransform(const planning_models::RobotState& state)
  {
    geometry_msgs::TransformStamped root_transform;
    root_transform.header.frame_id = state.getRobotModel()->getModelFrame();
    root_transform.header.stamp = ros::Time(ros::WallTime::now().toSec());
    root_transform.child_frame_id = state.getRobotModel()->getRootLinkName(); 
    if(root_transform.header.frame_id == root_transform.child_frame_id) return;
    planning_models::msgFromPose(state.getRootTransform(), root_transform.transform);
    transform_broadcaster_.sendTransform(root_transform);
  }

private:

  ros::NodeHandle nh_;
  ros::Publisher joint_state_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;

};

#endif
