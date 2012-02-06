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
#include <collision_detection/collision_tools.h>
#include <planning_models/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void findSelfCollisionAndDisplayContacts()
{
    ros::NodeHandle nh;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
    
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);
	
    sleep(1);

    collision_detection::CollisionRequest req;
    do
    {
	collision_detection::CollisionResult res;
	scene->getCurrentState().setToRandomValues();
	scene->checkSelfCollision(req, res);
	if (res.collision)
	    break;
    } while (true);

    moveit_msgs::PlanningScene psmsg;
    scene->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);

    req.contacts = true;
    req.max_contacts = 1000;
    req.max_contacts_per_pair = 500;
    collision_detection::CollisionResult res;
    req.verbose = true;
    scene->checkSelfCollision(req, res);
    std_msgs::ColorRGBA color;
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    visualization_msgs::MarkerArray arr;
    collision_detection::getCollisionMarkersFromContacts(arr, "/base_link", res.contacts, color, ros::Duration(30.0));
    pub_markers.publish(arr);
    ros::Duration(10).sleep();
}

void testSimple()
{
    ros::NodeHandle nh;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
    
    ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 20);
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    sleep(1);

    std::vector<shapes::Shape*> attached_shapes(1, new shapes::Box(0.2, 0.1, 0.1));
    Eigen::Affine3d t;
    t.setIdentity();
    std::vector<Eigen::Affine3d> attached_poses(1, t);
    std::vector<std::string> touch;
    touch.push_back("r_wrist_roll_link");
    touch.push_back("r_forearm_link");
    touch.push_back("r_gripper_palm_link");
    touch.push_back("r_gripper_l_finger_link");
    touch.push_back("r_gripper_r_finger_link");

    scene->getCurrentState().getLinkState("r_wrist_roll_link")->attachBody("attached", attached_shapes,
									   attached_poses, touch);
    
    collision_detection::CollisionRequest req;
    req.verbose = true;
    unsigned int N = 2;
    
    for (unsigned int i = 0 ; i < N ; ++i)
    {
	collision_detection::CollisionResult res;
	do
	{
	    ROS_INFO("Trying new state...");
	    res.collision = false;
	    if (i + 1 == N)
		scene->getCurrentState().setToDefaultValues();
	    else
		scene->getCurrentState().setToRandomValues();
	    scene->checkSelfCollision(req, res);
	}
	while (res.collision);

	ROS_INFO("Displaying valid state...");
	moveit_msgs::PlanningScene psmsg;
	scene->getPlanningSceneMsg(psmsg);
	pub_scene.publish(psmsg);
	ros::Duration(0.5).sleep();

	/*
	moveit_msgs::DisplayTrajectory d;
	d.model_id = scene->getKinematicModel()->getName();
	planning_models::kinematicStateToRobotState(scene->getCurrentState(), d.robot_state);
	pub_state.publish(d);
	for (int j = 0 ; j < 10 ; ++j)
	{
	    //	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
	} 
	*/   
    }

    sleep(1);
    
    planning_scene::PlanningScenePtr colliding = clone(scene);
    // construct a planning scene with 100 objects and no collisions
    random_numbers::RandomNumberGenerator rng;
    req.verbose = false;
    for (int i = 0 ; i < 100000 ; ++i)
    {
	t.translation() = Eigen::Vector3d(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(0, 2));
	scene->getCollisionWorld()->clearObjects();
	scene->getCollisionWorld()->addToObject("spere1", new shapes::Sphere(0.05), t);
	collision_detection::CollisionResult res;
	scene->checkCollision(req, res);
	if (!res.collision)
	{
	    int x = colliding->getCollisionWorld()->getObjectIds().size();
	    colliding->getCollisionWorld()->addToObject("speres" + boost::lexical_cast<std::string>(x), new shapes::Sphere(0.05), t);
	    std::cout << x << "\n";
	    if (x == 100)
		break;
	}
    }
    
    moveit_msgs::PlanningScene psmsg;
    colliding->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);
    
    ros::WallTime start = ros::WallTime::now();
    unsigned int M = 1000;
    for (unsigned int i = 0 ; i < M ; ++i)
    {
	collision_detection::CollisionResult res;
	colliding->checkCollision(req, res);
	if (res.collision)
	    ROS_ERROR("PROBLEM");
    }
    ROS_INFO("%lf full-collision checks per second", (double)M / (ros::WallTime::now() - start).toSec());
    

    /*
    req.verbose = false;
    ros::WallTime start = ros::WallTime::now();
    //    unsigned int N = 50000;
N = 50000;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
	collision_detection::CollisionResult res;
	scene->getCurrentState().setToRandomValues();
	scene->checkSelfCollision(req, res);
    }
    ROS_INFO("%lf self-collision checks per second", (double)N / (ros::WallTime::now() - start).toSec());
    */
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_collision_detection");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //    testSimple();
    findSelfCollisionAndDisplayContacts();
    
    ros::waitForShutdown();
    
    return 0;
}
