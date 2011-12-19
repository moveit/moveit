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

#include "planning_scene_monitor/planning_scene_monitor.h"
#include <tf/transform_listener.h>

class EnvironmentServer
{
public:
    EnvironmentServer(void) : psm_("robot_description", &tf_)
    {
	pub_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene_diff", 2);
	psm_.monitorDiffs(true);
	psm_.setUpdateCallback(boost::bind(&EnvironmentServer::onSceneUpdate, this));
	psm_.startWorldGeometryMonitor();
	psm_.startStateMonitor();
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    planning_scene_monitor::PlanningSceneMonitor psm_;
    ros::Publisher pub_diff_;
    
    void onSceneUpdate(void)
    {
	moveit_msgs::PlanningScene diff;
	
	psm_.lockScene();
	try
	{
	    psm_.getPlanningScene()->getPlanningSceneDiffMsg(diff);
	    psm_.monitorDiffs(true); // \todo this should work, but is SLOW
	}
	catch(...)
	{
	    psm_.unlockScene();
	    throw;
	}
	psm_.unlockScene();
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
