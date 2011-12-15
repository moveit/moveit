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

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>

TEST(PlanningScene, LoadRestore)
{ 
    ros::NodeHandle nh;
    ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    tf::TransformListener tf;
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description", &tf);
    planning_scene::PlanningScenePtr ps = psm.getPlanningScene();

    EXPECT_TRUE(ps->isConfigured());

    collision_detection::CollisionWorld &cw = *ps->getCollisionWorld();
    btTransform id = btTransform::getIdentity();
    cw.addObject("sphere", new shapes::Sphere(0.4), id);
    
    moveit_msgs::PlanningScene ps_msg;
    ps->getPlanningSceneMsg(ps_msg);
    ps->setPlanningSceneMsg(ps_msg);
    EXPECT_TRUE(ps->getCollisionWorld()->haveNamespace("sphere"));
    
    planning_scene::PlanningScene next(ps);
    EXPECT_TRUE(next.isConfigured());
    EXPECT_TRUE(next.getCollisionWorld()->haveNamespace("sphere"));
    next.getCollisionWorld()->addObject("sphere2", new shapes::Sphere(0.5), id);
    EXPECT_EQ(next.getCollisionWorld()->getNamespaces().size(), 2);
    EXPECT_EQ(ps->getCollisionWorld()->getNamespaces().size(), 1);
    next.getPlanningSceneDiffMsg(ps_msg);
    EXPECT_EQ(ps_msg.collision_objects.size(), 1);
    next.decoupleParent();
    next.getPlanningSceneDiffMsg(ps_msg);	
    EXPECT_EQ(ps_msg.collision_objects.size(), 2);
    next.getPlanningSceneMsg(ps_msg);	
    EXPECT_EQ(ps_msg.collision_objects.size(), 2);
    ps->setPlanningSceneMsg(ps_msg);
    EXPECT_EQ(ps->getCollisionWorld()->getNamespaces().size(), 2);
    
    sleep(1);
    scene_pub.publish(ps_msg); 
    sleep(1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv); 
    ros::init(argc, argv, "test_publish_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    return RUN_ALL_TESTS();
}
