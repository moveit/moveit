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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#include <ros/ros.h>
#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace chomp
{
ChompPlanner::ChompPlanner()
{
}

bool ChompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::MotionPlanRequest& req, const chomp::ChompParameters& params,
                         moveit_msgs::MotionPlanDetailedResponse& res) const
{
  if (!planning_scene)
    ROS_ERROR_STREAM("No planning scene initialized.");
  else
  {
    ROS_INFO_STREAM("Planning scene is configured. Beginning to plan.");
  }
  ros::WallTime start_time = ros::WallTime::now();
  ChompTrajectory trajectory(planning_scene->getRobotModel(), 3.0, .03, req.group_name);
  jointStateToArray(planning_scene->getRobotModel(), req.start_state.joint_state, req.group_name,
                    trajectory.getTrajectoryPoint(0));

  int goal_index = trajectory.getNumPoints() - 1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  sensor_msgs::JointState js;
  for (unsigned int i = 0; i < req.goal_constraints[0].joint_constraints.size(); i++)
  {
    js.name.push_back(req.goal_constraints[0].joint_constraints[i].joint_name);
    js.position.push_back(req.goal_constraints[0].joint_constraints[i].position);
    ROS_INFO_STREAM("Setting joint " << req.goal_constraints[0].joint_constraints[i].joint_name << " to position "
                                     << req.goal_constraints[0].joint_constraints[i].position);
  }
  jointStateToArray(planning_scene->getRobotModel(), js, req.group_name, trajectory.getTrajectoryPoint(goal_index));

  const moveit::core::JointModelGroup* model_group =
      planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < model_group->getActiveJointModels().size(); i++)
  {
    const moveit::core::JointModel* model = model_group->getActiveJointModels()[i];
    const moveit::core::RevoluteJointModel* revolute_joint =
        dynamic_cast<const moveit::core::RevoluteJointModel*>(model);

    if (revolute_joint != NULL)
    {
      if (revolute_joint->isContinuous())
      {
        double start = (trajectory)(0, i);
        double end = (trajectory)(goal_index, i);
        ROS_INFO_STREAM("Start is " << start << " end " << end << " short " << shortestAngularDistance(start, end));
        (trajectory)(goal_index, i) = start + shortestAngularDistance(start, end);
      }
    }
  }

  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // optimize!
  moveit::core::RobotState start_state(planning_scene->getCurrentState());
  moveit::core::robotStateMsgToRobotState(req.start_state, start_state);
  start_state.update();

  ros::WallTime create_time = ros::WallTime::now();
  ChompOptimizer optimizer(&trajectory, planning_scene, req.group_name, &params, start_state);
  if (!optimizer.isInitialized())
  {
    ROS_WARN_STREAM("Could not initialize optimizer");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  optimizer.optimize();
  ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());
  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_INFO("Output trajectory has %d joints", trajectory.getNumJoints());

  res.trajectory.resize(1);

  for (size_t i = 0; i < model_group->getActiveJointModels().size(); i++)
  {
    res.trajectory[0].joint_trajectory.joint_names.push_back(model_group->getActiveJointModels()[i]->getName());
  }

  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;  // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory[0].joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i = 0; i < trajectory.getNumPoints(); i++)
  {
    res.trajectory[0].joint_trajectory.points[i].positions.resize(trajectory.getNumJoints());
    for (size_t j = 0; j < res.trajectory[0].joint_trajectory.points[i].positions.size(); j++)
    {
      res.trajectory[0].joint_trajectory.points[i].positions[j] = trajectory.getTrajectoryPoint(i)(j);
      if (i == trajectory.getNumPoints() - 1)
      {
        ROS_INFO_STREAM("Joint " << j << " " << res.trajectory[0].joint_trajectory.points[i].positions[j]);
      }
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory[0].joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  ROS_INFO("Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f",
           (ros::WallTime::now() - start_time).toSec(),
           res.trajectory[0].joint_trajectory.points[goal_index].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());
  return true;
}
}
