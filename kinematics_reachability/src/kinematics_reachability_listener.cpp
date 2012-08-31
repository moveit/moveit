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

#include <arm_kinematics_reachability/arm_kinematics_reachability_listener.h>

namespace arm_kinematics_reachability
{

ArmKinematicsReachabilityListener::ArmKinematicsReachabilityListener():ArmKinematicsReachability(),has_workspace_(false)
{
  workspace_subscriber_ = node_handle_.subscribe("/workspace_tests/workspace", 1, &arm_kinematics_reachability::ArmKinematicsReachabilityListener::workspaceCallback, this);
}

void ArmKinematicsReachabilityListener::workspaceCallback(const arm_kinematics_reachability::WorkspacePointsConstPtr &msg)
{
  workspace_ = *msg;
  has_workspace_ = true;
}

} // namespace


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_workspace_tests");
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  arm_kinematics_reachability::ArmKinematicsReachabilityListener listener;
  while(!listener.hasWorkspace() && ros::ok())
  {
    sleep(0.1);
  }

  //  listener.visualize(listener.getWorkspace(),"recorded",listener.getWorkspace().orientations[0]);
  listener.visualize(listener.getWorkspace(),"recorded");
  ROS_INFO("Success");
  ros::waitForShutdown();
  return(0);
}
