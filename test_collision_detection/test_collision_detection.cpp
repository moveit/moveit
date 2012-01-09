/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <planning_models/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void testSimple()
{
    ros::NodeHandle nh;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
    
    ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_valid_states", 10);
    sleep(1);
    
    collision_detection::CollisionRequest req;
    req.verbose = true;
    for (int i = 0 ; i < 10 ; ++i)
    {
	collision_detection::CollisionResult res;
	do
	{
	    ROS_INFO("Trying new state...");
	    res.collision = false;
	    scene->getCurrentState().setToRandomValues();
	    scene->checkSelfCollision(req, res);
	}
	while (res.collision);

	ROS_INFO("Displaying valid state...");

	moveit_msgs::DisplayTrajectory d;
	d.model_id = scene->getKinematicModel()->getName();
	planning_models::kinematicStateToRobotState(scene->getCurrentState(), d.robot_state);
	pub.publish(d);
	for (int j = 0 ; j < 10 ; ++j)
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
	}    
    }

    req.verbose = false;
    ros::WallTime start = ros::WallTime::now();
    unsigned int N = 500000;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
	collision_detection::CollisionResult res;
	scene->getCurrentState().setToRandomValues();
	scene->checkSelfCollision(req, res);
    }
    ROS_INFO("%lf self-collision checks per second", (double)N / (ros::WallTime::now() - start).toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_collision_detection");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    testSimple();
    return 0;
}
