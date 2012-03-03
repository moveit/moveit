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
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>

static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";
static const std::string ROBOT_DESCRIPTION="robot_description";

TEST(OmplPlanning, PathConstrainedSimplePlan)
{
  ros::NodeHandle nh;
  ros::service::waitForService(PLANNER_SERVICE_NAME);
  ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
  
  ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
  EXPECT_TRUE(planning_service_client.exists());
  EXPECT_TRUE(planning_service_client.isValid());
  
  moveit_msgs::GetMotionPlan::Request mplan_req;
  moveit_msgs::GetMotionPlan::Response mplan_res;
  
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
  planning_scene::PlanningScene &scene = *psm.getPlanningScene();
  EXPECT_TRUE(scene.isConfigured());

  mplan_req.motion_plan_request.planner_id = "RRTConnectkConfigDefault";
  mplan_req.motion_plan_request.group_name = "arms";
  mplan_req.motion_plan_request.num_planning_attempts = 1;
  mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  
  // set the goal constraints

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm.constraint_region_shape.dimensions.push_back(0.001);
  pcm.constraint_region_shape.dimensions.push_back(0.001);
  pcm.constraint_region_shape.dimensions.push_back(0.001);
  
  pcm.constraint_region_pose.header.frame_id = scene.getKinematicModel()->getModelFrame();
  pcm.constraint_region_pose.pose.position.x = 0.5;
  pcm.constraint_region_pose.pose.position.y = 0.4;
  pcm.constraint_region_pose.pose.position.z = 0.7;
  pcm.constraint_region_pose.pose.orientation.x = 0.0;
  pcm.constraint_region_pose.pose.orientation.y = 0.0;
  pcm.constraint_region_pose.pose.orientation.z = 0.0;
  pcm.constraint_region_pose.pose.orientation.w = 1.0;
  pcm.weight = 1.0;
  
  mplan_req.motion_plan_request.goal_constraints.resize(1);
  mplan_req.motion_plan_request.goal_constraints[0].position_constraints.push_back(pcm);


  // add path constraints
  moveit_msgs::Constraints &c = mplan_req.motion_plan_request.path_constraints;  

  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = 0.7;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0;
  pcm2.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm2.constraint_region_shape.dimensions.push_back(0.1);
  pcm2.constraint_region_shape.dimensions.push_back(0.1);
  pcm2.constraint_region_shape.dimensions.push_back(0.1);
  
  pcm2.constraint_region_pose.header.frame_id = "l_wrist_roll_link";
  pcm2.constraint_region_pose.pose.position.x = 0.0;
  pcm2.constraint_region_pose.pose.position.y = 0.0;
  pcm2.constraint_region_pose.pose.position.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.x = 0.0;
  pcm2.constraint_region_pose.pose.orientation.y = 0.0;
  pcm2.constraint_region_pose.pose.orientation.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = scene.getKinematicModel()->getModelFrame();
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 1.0;
  ocm.orientation.quaternion.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);




  // sample a start state that meets the path constraints
  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  kinematic_constraints::KinematicsSubgroupAllocator sa;
  sa[scene.getKinematicModel()->getJointModelGroup("left_arm")] = kinematics_allocator;
  sa[scene.getKinematicModel()->getJointModelGroup("right_arm")] = kinematics_allocator;

  kinematic_constraints::ConstraintSamplerPtr s = kinematic_constraints::ConstraintSampler::constructFromMessage
    (scene.getKinematicModel()->getJointModelGroup("arms"), c, scene.getKinematicModel(), scene.getTransforms(), kinematic_constraints::KinematicsAllocator(), sa);
  
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(scene.getKinematicModel(), scene.getTransforms());
  kset.add(c);
  
  bool found = false;
  planning_models::KinematicState ks(scene.getKinematicModel());
  ks.setToDefaultValues();
  for (int i = 0 ; i < 100 ; ++i)
  {
    std::vector<double> values;
    if (s->sample(values, ks, 10))
    {
      ks.getJointStateGroup("arms")->setStateValues(values);
      planning_models::kinematicStateToRobotState(ks, mplan_req.motion_plan_request.start_state); 
      moveit_msgs::DisplayTrajectory d;
      d.model_id = scene.getKinematicModel()->getName();
      d.trajectory_start = mplan_req.motion_plan_request.start_state;
      pub.publish(d);
      ros::Duration(0.5).sleep();
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
  
  // run planner
  
  ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
  ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
  EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
  
  
  moveit_msgs::DisplayTrajectory d;
  d.model_id = scene.getKinematicModel()->getName();
  d.trajectory_start = mplan_res.trajectory_start;
  d.trajectory = mplan_res.trajectory;
  pub.publish(d);
  ros::Duration(0.5).sleep(); 
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  ros::init(argc, argv, "test_ompl_planning");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  return RUN_ALL_TESTS();
}
