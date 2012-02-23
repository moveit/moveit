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
#include <kinematic_constraints/utils.h>

static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";
static const std::string ROBOT_DESCRIPTION="robot_description";

TEST(OmplInterface, JointModelStateConversion)
{
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
  planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
  EXPECT_TRUE(scene->isConfigured());
  ompl_interface_ros::OMPLInterfaceROS oi(scene->getKinematicModel());
  ompl_interface::ModelBasedPlanningContextPtr pc = oi.getPlanningContext("arms", "JointModel");
  planning_models::KinematicState kstate(scene->getKinematicModel());
  ompl::base::ScopedState<> ostate1(pc->getOMPLStateSpace());
  ompl::base::ScopedState<> ostate2(ostate1.getSpace());
  for (int i = 0 ; i < 100 ; ++i)
  {
    ostate1.random();
    kstate.setToRandomValues();
    pc->getOMPLStateSpace()->copyToKinematicState(kstate, ostate1.get());
    pc->getOMPLStateSpace()->copyToOMPLState(ostate2.get(), kstate);
    EXPECT_EQ(ostate1, ostate2);
  }
}

TEST(OmplPlanning, JointGoal)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    
    ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    EXPECT_TRUE(planning_service_client.exists());
    EXPECT_TRUE(planning_service_client.isValid());

    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    EXPECT_TRUE(scene.isConfigured());


    // try a Joint goal
    mplan_req.motion_plan_request.group_name = "right_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 5;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    const std::vector<std::string>& joint_names = scene.getKinematicModel()->getJointModelGroup("right_arm")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = 0.0;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -2.0;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -.2;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -.2;

    ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
    ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}

TEST(OmplPlanning, PositionGoal)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    EXPECT_TRUE(planning_service_client.exists());
    EXPECT_TRUE(planning_service_client.isValid());

    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    EXPECT_TRUE(scene.isConfigured());

    // try a position goal
    mplan_req.motion_plan_request.group_name = "right_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(25.0);

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "r_wrist_roll_link";
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;
    pcm.constraint_region_shape.type = moveit_msgs::Shape::SPHERE;
    pcm.constraint_region_shape.dimensions.push_back(0.001);

    pcm.constraint_region_pose.header.frame_id = scene.getKinematicModel()->getModelFrame();
    pcm.constraint_region_pose.pose.position.x = 0.55;
    pcm.constraint_region_pose.pose.position.y = -0.2;
    pcm.constraint_region_pose.pose.position.z = 1.25;
    pcm.constraint_region_pose.pose.orientation.x = 0.0;
    pcm.constraint_region_pose.pose.orientation.y = 0.0;
    pcm.constraint_region_pose.pose.orientation.z = 0.0;
    pcm.constraint_region_pose.pose.orientation.w = 1.0;
    pcm.weight = 1.0;
    
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].position_constraints.push_back(pcm);

    ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
    ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}

TEST(OmplPlanning, OrientationGoal)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    EXPECT_TRUE(planning_service_client.exists());
    EXPECT_TRUE(planning_service_client.isValid());

    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    EXPECT_TRUE(scene.isConfigured());

    // try a position goal
    mplan_req.motion_plan_request.group_name = "left_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);


    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "l_wrist_roll_link";
    ocm.orientation.header.frame_id = scene.getKinematicModel()->getModelFrame();
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.4;
    ocm.weight = 1.0;

    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].orientation_constraints.push_back(ocm);

    ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
    ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}



TEST(OmplPlanning, PoseGoal)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    EXPECT_TRUE(planning_service_client.exists());
    EXPECT_TRUE(planning_service_client.isValid());

    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    EXPECT_TRUE(scene.isConfigured());

    // try a position goal
    mplan_req.motion_plan_request.group_name = "left_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "l_wrist_roll_link";
    ocm.orientation.header.frame_id = scene.getKinematicModel()->getModelFrame();
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.4;
    ocm.weight = 1.0;  

    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].orientation_constraints.push_back(ocm);

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "l_wrist_roll_link";
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;
    pcm.constraint_region_shape.type = moveit_msgs::Shape::SPHERE;
    pcm.constraint_region_shape.dimensions.push_back(0.001);

    pcm.constraint_region_pose.header.frame_id = scene.getKinematicModel()->getModelFrame();
    pcm.constraint_region_pose.pose.position.x = 0.55;
    pcm.constraint_region_pose.pose.position.y = 0.2;
    pcm.constraint_region_pose.pose.position.z = 1.25;
    pcm.constraint_region_pose.pose.orientation.x = 0.0;
    pcm.constraint_region_pose.pose.orientation.y = 0.0;
    pcm.constraint_region_pose.pose.orientation.z = 0.0;
    pcm.constraint_region_pose.pose.orientation.w = 1.0;
    pcm.weight = 1.0;
    mplan_req.motion_plan_request.goal_constraints[0].position_constraints.push_back(pcm);

    ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
    ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}

TEST(OmplPlanning, SimplePoseGoal)
{ 
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    EXPECT_TRUE(scene.isConfigured());

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = scene.getKinematicModel()->getModelFrame();
    pose.pose.position.x = 0.55;
    pose.pose.position.y = 0.2;
    pose.pose.position.z = 1.25;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints("l_wrist_roll_link", pose);
    
    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;
    
    mplan_req.motion_plan_request.group_name = "left_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    mplan_req.motion_plan_request.goal_constraints.push_back(goal);
    

    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    EXPECT_TRUE(planning_service_client.exists());
    EXPECT_TRUE(planning_service_client.isValid());

    ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
    ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ompl_planning");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  return RUN_ALL_TESTS();
}
