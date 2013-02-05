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

/* Author: Ioan Sucan, Sachin Chitta */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>

class EnvironmentServer
{
public:
  EnvironmentServer() : 
    tf_(new tf::TransformListener()),
    planning_scene_monitor_("robot_description", tf_)
  {
    pub_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene_diff", 128);
    parent_scene_ = planning_scene_monitor_.getPlanningScene();
    // this will create a new planning scene whose parent is the current planning scene
    planning_scene_monitor_.monitorDiffs(true);
    planning_scene_monitor_.addUpdateCallback(boost::bind(&EnvironmentServer::onSceneUpdate, this));
    planning_scene_monitor_.startWorldGeometryMonitor();
    planning_scene_monitor_.startStateMonitor();
  }
  
private:
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor_;
  planning_scene::PlanningScenePtr parent_scene_;
  ros::Publisher pub_diff_;
  
  void onSceneUpdate()
  {
    moveit_msgs::PlanningScene diff;
    
    planning_scene_monitor_.lockScene();
    try
    {
      planning_scene_monitor_.getPlanningScene()->getPlanningSceneDiffMsg(diff);
      planning_scene_monitor_.getPlanningScene()->pushDiffs(parent_scene_);
      planning_scene_monitor_.getPlanningScene()->clearDiffs();
    }
    catch(...)
    {
      planning_scene_monitor_.unlockScene();
      throw;
    }
    planning_scene_monitor_.unlockScene();
    pub_diff_.publish(diff);
    ROS_DEBUG("Published scene update");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environment_server");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  EnvironmentServer es;
  
  ros::waitForShutdown();
  
  return 0;
}
