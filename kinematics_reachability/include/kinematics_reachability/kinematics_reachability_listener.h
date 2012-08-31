/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef KINEMATICS_REACHABILITY_LISTENER_H
#define KINEMATICS_REACHABILITY_LISTENER_H

#include <kinematics_reachability/kinematics_reachability.h>

namespace kinematics_reachability
{
class KinematicsReachabilityListener : public KinematicsReachability
{
public:

  /** @class
   *  @brief Compute and visualize reachable workspace for an arm
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   */
  KinematicsReachabilityListener();

  virtual ~KinematicsReachabilityListener()
  {
  };

  bool hasWorkspace()
  {
    return has_workspace_;
  }

  kinematics_reachability::WorkspacePoints& getWorkspace()
  {
    return workspace_;
  }

private:

  void workspaceCallback(const kinematics_reachability::WorkspacePointsConstPtr &msg);
  ros::Subscriber workspace_subscriber_;
  kinematics_reachability::WorkspacePoints workspace_;
  bool has_workspace_;
};

}
#endif
