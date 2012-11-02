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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <plan_execution/plan_execution.h>
#include <kinematics_planner/kinematics_planner.h>
#include <trajectory_processing/trajectory_tools.h>

#include <moveit_msgs/PositionConstraint.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";    // name of the robot description (a param name, so it can be changed externally)
static const std::string NODE_NAME = "pick_place_node";                   // name of node
static const std::string GROUP_NAME = "left_arm";
static const std::string END_EFFECTOR_FRAME_NAME = "l_wrist_roll_link";
static const std::string END_EFFECTOR_LINK_NAME = "l_wrist_roll_link";
static const ros::Duration ALLOWED_PLANNING_TIME(5.0);
static const int NUM_PLANNING_ATTEMPTS = 1;
static const ros::Duration WAIT_LOOP_DURATION(0.01);
static const double INTERPOLATED_IK_PLANNING_TIME = 5.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));

    if (planning_scene_monitor->getPlanningScene() && planning_scene_monitor->getPlanningScene()->isConfigured())
    {
        planning_scene_monitor->startWorldGeometryMonitor();
        planning_scene_monitor->startSceneMonitor();
        planning_scene_monitor->startStateMonitor();
    }
    else
    {
        ROS_ERROR("Planning scene not configured");
        return -1;
    }

    plan_execution::PlanExecution plan_execution(planning_scene_monitor);

    /* wait until we know our current joint state */
    while(!planning_scene_monitor->getStateMonitor()->haveCompleteState())
    {
        ROS_INFO("Waiting for current joint states");
        WAIT_LOOP_DURATION.sleep();
    }

    /* get the current gripper pose */
    planning_scene_monitor->updateFrameTransforms();
    planning_models::KinematicState kinematic_state = planning_scene_monitor->getPlanningScene()->getCurrentState();
    const Eigen::Affine3d *gripper_pose = kinematic_state.getFrameTransform(END_EFFECTOR_LINK_NAME);
    ROS_INFO_STREAM("Current gripper pose:" << (gripper_pose->matrix()));

    Eigen::Vector3d t = gripper_pose->translation();
    Eigen::Quaterniond q;
    q = gripper_pose->rotation();

    ROS_INFO("Current gripper trans: %f %f %f", t.x(), t.y(), t.z());
    ROS_INFO("Current gripper rot: %f %f %f %f", q.x(), q.y(), q.z(), q.w());

    /* start position is current position */
    geometry_msgs::PoseStamped ps_start, ps_end;
    ps_start.header.frame_id = "base_link";
    ps_start.header.stamp = ros::Time::now();
    ps_start.pose.position.x = t.x();
    ps_start.pose.position.y = t.y();
    ps_start.pose.position.z = t.z();
    ps_start.pose.orientation.x = q.x();
    ps_start.pose.orientation.y = q.y();
    ps_start.pose.orientation.z = q.z();
    ps_start.pose.orientation.w = q.w();

    /* end position is start position shifted a bit */
    ps_end = ps_start;
    ps_end.pose.position.x += 0.1;

    /* create an interpolated IK planner */
    kinematics_planner::KinematicsPlanner kin_planner;
    std::vector<std::string> group_names;
    group_names.push_back(GROUP_NAME);
    kin_planner.initialize(group_names, planning_scene_monitor->getKinematicModel());

    std::map<std::string,geometry_msgs::PoseStamped> start_request;
    start_request[GROUP_NAME] = ps_start;
    std::map<std::string,geometry_msgs::PoseStamped> goal_request;
    goal_request[GROUP_NAME] = ps_end;
    moveit_msgs::Constraints path_constraints;
    moveit_msgs::RobotTrajectory robot_trajectory;
    moveit_msgs::MoveItErrorCodes error_code;

    /* create plan using interpolated IK planner */
    bool r = kin_planner.solve(start_request,
                      goal_request,
                      planning_scene_monitor->getPlanningScene(),
                      path_constraints,
                      INTERPOLATED_IK_PLANNING_TIME,
                      robot_trajectory,
                      error_code);
    if(r)
    {
        ROS_INFO("Got solution from kinematics planner");
        ROS_INFO_STREAM("Solution: " << robot_trajectory);
    }
    else
    {
        ROS_INFO("Unable to solve. Error code: %d", error_code.val);
    }

    ROS_INFO("Trajectory execution succeeded");

    planning_scene_monitor->stopSceneMonitor();

    ros::spin();
    return 0;
}
