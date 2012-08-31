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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/*
 * Author: Sachin Chitta
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_datatypes.h>
#include <kinematics_reachability/kinematics_reachability.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_workspace_tests");
  ros::AsyncSpinner spinner(2); 
  spinner.start();

  ros::NodeHandle node_handle("~");
  std::string group_name, root_name;
  node_handle.param<std::string>("group", group_name, std::string());
  node_handle.param<std::string>(group_name+"/root_name", root_name, std::string());
  ros::NodeHandle root_handle;

  /**** WORKSPACE PARAMETERS - These are the parameters you need to change to specify a different 
region in the workspace for which reachability is to be computed****/
  kinematics_reachability::KinematicsReachability aw;
  kinematics_reachability::WorkspacePoints workspace;

  workspace.position_resolution = 0.05;
  workspace.header.frame_id = root_name;

  workspace.parameters.min_corner.x = 0.4;
  workspace.parameters.min_corner.y = -0.75;
  workspace.parameters.min_corner.z = 0.38;

  workspace.parameters.max_corner.x = 0.8;
  workspace.parameters.max_corner.y = 0.75;
  workspace.parameters.max_corner.z = 0.42;
  
  //SET OF ORIENTATIONS TO TEST FOR REACHABILITY

  geometry_msgs::Quaternion quaternion;
  quaternion.w = 1.0;
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromYaw(M_PI/2.0);
  workspace.orientations.push_back(quaternion);
  /*
  quaternion = tf::createQuaternionMsgFromYaw(M_PI);
  workspace.orientations.push_back(quaternion);
  */
  quaternion = tf::createQuaternionMsgFromYaw(-M_PI/2.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,0.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2.0,0.0);
  workspace.orientations.push_back(quaternion);

  // The octants
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,-M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,-M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  while(!aw.isActive())
  {
    sleep(1.0);
    ROS_INFO("Waiting for planning scene to be set");
  }

  aw.computeWorkspace(workspace);
  aw.visualize(workspace,"full");
  //  aw.visualizeWithArrows(workspace,"full");
  //  aw.visualize(workspace,"RPY(0,0,0)",zero_orientation);
  ROS_INFO("Success");

  aw.publishWorkspace(workspace);

  ros::waitForShutdown();

  return(0);
}
