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

  /* move the arm */
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;

  /* fill in motion planning request */
  req.motion_plan_request.allowed_planning_time = ALLOWED_PLANNING_TIME;
  req.motion_plan_request.group_name = GROUP_NAME;
  req.motion_plan_request.num_planning_attempts = NUM_PLANNING_ATTEMPTS;

  /* for the robot state, we just need to fill in the current joint names and positions */
  const std::vector<std::string> joint_names = planning_scene_monitor->getKinematicModel()->getJointModelGroup(GROUP_NAME)->getJointModelNames();
  std::map<std::string, double> state_values_map = planning_scene_monitor->getStateMonitor()->getCurrentStateValues();
  for(unsigned int joint_i=0; joint_i < joint_names.size(); ++joint_i)
  {
    req.motion_plan_request.start_state.joint_state.name.push_back(joint_names[joint_i]);
    req.motion_plan_request.start_state.joint_state.position.push_back(state_values_map[joint_names[joint_i]]);
  }

  /* add a constraint on the end effector position */
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "base_link"; //END_EFFECTOR_FRAME_NAME;
  position_constraint.header.stamp = ros::Time::now();
  position_constraint.link_name = END_EFFECTOR_LINK_NAME;
  shape_msgs::SolidPrimitive bbox;
  bbox.type = shape_msgs::SolidPrimitive::BOX;
  bbox.dimensions.push_back(10.0);
  bbox.dimensions.push_back(10.0);
  bbox.dimensions.push_back(10.0);
  geometry_msgs::Pose p;
  p.position.x = 0.0;
  p.position.y = 0.0;
  p.position.z = 0.1;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 1.0;
  position_constraint.weight = 1.0;
  position_constraint.constraint_region.primitives.push_back(bbox);
  position_constraint.constraint_region.primitive_poses.push_back(p);
  moveit_msgs::Constraints constraints;
  constraints.position_constraints.push_back(position_constraint);
  req.motion_plan_request.goal_constraints.push_back(constraints);

  ROS_INFO("Planning motion....");
  planning_scene_monitor->updateFrameTransforms();

  bool solved = false;
  planning_scene_monitor->lockScene();

  try
  {
    solved = plan_execution.getPlanningPipeline().generatePlan(planning_scene_monitor->getPlanningScene(), req, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
  }
  planning_scene_monitor->unlockScene();

  if(solved)
  {
    ROS_INFO("Planning successful");
  }
  else
  {
    ROS_INFO("Planning failed");
    return 0;
  }

  /* execute the trajectory */
  ROS_INFO("Executing trajectory");
  plan_execution.getTrajectoryExecutionManager().clear();
  if(plan_execution.getTrajectoryExecutionManager().push(res.trajectory))
  {
    plan_execution.getTrajectoryExecutionManager().execute();

    /* wait for the trajectory to complete */
    moveit_controller_manager::ExecutionStatus es = plan_execution.getTrajectoryExecutionManager().waitForExecution();
    if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
      if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      else
        if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
          res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
        else
          res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
  else
  {
    ROS_INFO("Failed to push trajectory");
    return -1;
  }

  ROS_INFO("Trajectory execution succeeded");

  planning_scene_monitor->stopSceneMonitor();

  ros::spin();
  return 0;
}
