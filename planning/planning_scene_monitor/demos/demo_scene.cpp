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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <shape_tools/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void constructScene(const planning_scene::PlanningScenePtr &scene)
{
  scene->setName("pole_blocking_right_arm_pan");
  
  Eigen::Affine3d t;
  t = Eigen::Translation3d(0.7, -0.5, 0.7);
  scene->getWorldNonConst()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.1, 0.1, 1.4)), t);
  
  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_roll_link";
  aco.touch_links.push_back("r_wrist_roll_link");
  aco.touch_links.push_back("r_gripper_palm_link");
  aco.touch_links.push_back("r_gripper_led_frame");
  aco.touch_links.push_back("r_gripper_motor_accelerometer_link");
  aco.touch_links.push_back("r_gripper_tool_frame");
  aco.touch_links.push_back("r_gripper_motor_slider_link");
  aco.touch_links.push_back("r_gripper_motor_screw_link");
  aco.touch_links.push_back("r_gripper_l_finger_link");
  aco.touch_links.push_back("r_gripper_l_finger_tip_link");
  aco.touch_links.push_back("r_gripper_r_finger_link");
  aco.touch_links.push_back("r_gripper_r_finger_tip_link");
  aco.touch_links.push_back("r_gripper_l_finger_tip_frame");
  
  random_numbers::RandomNumberGenerator rng;
  moveit_msgs::CollisionObject &co = aco.object;
  co.id = "bubu";
  co.header.stamp = ros::Time::now();
  co.header.frame_id = aco.link_name;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.push_back(0.3);
  co.primitives[0].dimensions.push_back(0.01);
  co.primitives[0].dimensions.push_back(0.3);
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.28;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0;
  co.primitive_poses[0].orientation.w = 1.0;

  scene->processAttachedCollisionObjectMsg(aco);
  std_msgs::ColorRGBA cl;
  cl.r = 1.0f;
  cl.g = 0.0f;
  cl.b = 0.0f; 
  cl.a = 1.0f;
  scene->setObjectColor(co.id, cl);   
}

void sendScene()
{  
  ros::NodeHandle nh;
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  
  constructScene(psm.getPlanningScene());
  ros::Duration(0.5).sleep();
  
  moveit_msgs::PlanningScene psmsg;
  psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
  pub_scene.publish(psmsg);
  ROS_INFO("Scene published.");
}

void sendCollisionObject()
{
  ros::NodeHandle nh;
  tf::TransformListener tf;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  sleep(1);
  random_numbers::RandomNumberGenerator rng;
  
  moveit_msgs::CollisionObject co;
  co.id = "test" + boost::lexical_cast<std::string>(rng.uniformReal(0,100000));
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "odom";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.05;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = rng.uniformReal(-1.5, 1.5);
  co.primitive_poses[0].position.y = rng.uniformReal(-1.5, 1.5);
  co.primitive_poses[0].position.z = rng.uniformReal(0.1, 2.0);
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);
  ROS_INFO("Object published.");
  ros::Duration(0.5).sleep();
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo", ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sendScene();
  //    sendCollisionObject();    
  
  ros::waitForShutdown();
  
  return 0;
}
