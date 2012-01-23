/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <moveit_visualization_ros/planning_visualization.h>
#include <moveit_visualization_ros/kinematic_constraints/util.h>

PlanningVisualization::PlanningVisualization(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                             ros::Publisher& marker_publisher)
  : ompl_interface_(planning_scene_monitor->getPlanningScene()),
    group_visualization_(planning_scene_monitor,
                         interactive_marker_server,
                         "right_arm",                                                                        "pr2_arm_kinematics/PR2ArmKinematicsPlugin",
                         marker_publisher)

{
  group_visualization_.addMenuEntry("Plan", boost::bind(&PlanningVisualization::generatePlan, this));
}                        

void PlanningVisualization::generatePlan(void) {

  const planing_models::KinematicState& start_state = group_visualization_->getStartState();

  const planing_models::KinematicState& goal_state = group_visualization_->getGoalState();
  
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;

  req.group_name = "right_arm";
  planning_models::kinematicStateToRobotState(start_state,req.start_state);
  req.goal_constraints = kinematic_constraints::constructGoalConstraints(goal_state->getJointStateGroup("right_arm"));
  req.num_planning_attempts = 1;
  req.allowed_planning_time = ros::Duration(3.0);

  ompl_interface_.solve(req, res);
}
