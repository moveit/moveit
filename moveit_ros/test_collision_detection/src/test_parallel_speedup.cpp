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

/* Author: Ioan Sucan, Sachin Chitta */

#include "planning_scene_monitor/planning_scene_monitor.h"

static const std::string ROBOT_DESCRIPTION="robot_description";

void runCollisionDetection(const planning_scene::PlanningScene *scene, const planning_models::KinematicState *state)
{
    ROS_INFO("Starting thread");
    static const unsigned int N = 100000;
    collision_detection::CollisionRequest req;
    ros::WallTime start = ros::WallTime::now();
    for (unsigned int i = 0 ; i < N ; ++i)
    {
	collision_detection::CollisionResult res;
	scene->checkCollision(req, res, *state);
    }
    double duration = (ros::WallTime::now() - start).toSec();
    ROS_INFO("Thread average is %lf collision checks per second", (double)N / duration);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_parallel_speedup");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
    if (psm.getPlanningScene()->isConfigured())
    {
	ros::Duration(0.5).sleep();
	
	std::vector<planning_models::KinematicStatePtr> states;
	unsigned int nthreads = 8;
	
	for (unsigned int i = 0 ; i < nthreads ; ++i)
	{
	    // sample a valid state
	    planning_models::KinematicState *state = new planning_models::KinematicState(psm.getPlanningScene()->getKinematicModel());
	    collision_detection::CollisionRequest req;
	    do
	    {
		state->setToRandomValues();
		collision_detection::CollisionResult res;
		psm.getPlanningScene()->checkCollision(req, res);
		if (!res.collision)
		    break;
	    } while (true);
	    states.push_back(planning_models::KinematicStatePtr(state));
	}
	
	std::vector<boost::thread*> threads;
	
	for (unsigned int i = 0 ; i < states.size() ; ++i)
	    threads.push_back(new boost::thread(boost::bind(&runCollisionDetection, psm.getPlanningScene().get(), states[i].get())));
	
	for (unsigned int i = 0 ; i < states.size() ; ++i)
	{
	    threads[i]->join();
	    delete threads[i];
	}
	
    }
    else
	ROS_ERROR("Planning scene not configured");
    
    return 0;
}
