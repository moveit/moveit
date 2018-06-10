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
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "No planning scene initialized.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  if (req.start_state.joint_state.position.empty())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Start state is empty");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if (not planning_scene->getRobotModel()->satisfiesPositionBounds(req.start_state.joint_state.position.data()))
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Start state violates joint limits");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ros::WallTime start_time = ros::WallTime::now();
  ChompTrajectory trajectory(planning_scene->getRobotModel(), 3.0, .03, req.group_name);

  //std::cout << trajectory.trajectory_ << " 1. complete initialized TRAJECTORY in CHOMP_PLANNER..!!!!!" << std::endl;

  jointStateToArray(planning_scene->getRobotModel(), req.start_state.joint_state, req.group_name,
                    trajectory.getTrajectoryPoint(0));

  //std::cout << trajectory.trajectory_ << " 2.  complete initialized TRAJECTORY in CHOMP_PLANNER..!!!!!" << std::endl;



  if (req.goal_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "No goal constraints specified!");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  if (req.goal_constraints[0].joint_constraints.empty())
  {
    ROS_ERROR_STREAM("Only joint-space goals are supported");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  int goal_index = trajectory.getNumPoints() - 1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  sensor_msgs::JointState js;
  for (unsigned int i = 0; i < req.goal_constraints[0].joint_constraints.size(); i++)
  {
    js.name.push_back(req.goal_constraints[0].joint_constraints[i].joint_name);
    js.position.push_back(req.goal_constraints[0].joint_constraints[i].position);
    ROS_INFO_STREAM_NAMED("chomp_planner", "Setting joint " << req.goal_constraints[0].joint_constraints[i].joint_name
                                                            << " to position "
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

  const std::vector<std::string>& active_joint_names = model_group->getActiveJointModelNames();
  const Eigen::MatrixXd goal_state = trajectory.getTrajectoryPoint(goal_index);
  moveit::core::RobotState goal_robot_state = planning_scene->getCurrentState();
  goal_robot_state.setVariablePositions(
          active_joint_names, std::vector<double>(goal_state.data(), goal_state.data() + active_joint_names.size()));

  if (not goal_robot_state.satisfiesBounds())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Goal state violates joint limits");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  //std::cout << trajectory.trajectory_ << "3. complete initialized TRAJECTORY in CHOMP_PLANNER before min jerk..!!!!!" << std::endl;

  if(params.trajectory_initialization_method_.compare("quintic-spline") == 0)
    trajectory.fillInMinJerk();
  else if(params.trajectory_initialization_method_.compare("linear") == 0)
    trajectory.fillInLinearInterpolation();
  else if(params.trajectory_initialization_method_.compare("cubic") == 0)
    trajectory.fillInCubicInterpolation();
  else
    ROS_ERROR_STREAM_NAMED("chomp_planner", "invalid interpolation method specified in the chomp_planner file");
  // fill in an initial quintic spline trajectory
  //trajectory.fillInMinJerk();
  //trajectory.fillInLinearInterpolation();
  //trajectory.fillInCubicInterpolation();
  //std::cout << " using initialization method: " << params.interpolation_method_ << std::endl;

  //std::cout << trajectory.trajectory_ << "4. complete initialized TRAJECTORY in CHOMP_PLANNER after min jerk..!!!!!" << std::endl;


  // optimize!
  moveit::core::RobotState start_state(planning_scene->getCurrentState());
  moveit::core::robotStateMsgToRobotState(req.start_state, start_state);
  start_state.update();

  ros::WallTime create_time = ros::WallTime::now();
  //std::cout << trajectory.trajectory_ << "5. complete initialized TRAJECTORY in CHOMP_PLANNER before optimizer object creation..!!!!!" << std::endl;


  ChompOptimizer optimizer(&trajectory, planning_scene, req.group_name, &params, start_state);
  if (!optimizer.isInitialized())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Could not initialize optimizer");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
  ROS_DEBUG_NAMED("chomp_planner", "Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());

  //std::cout << trajectory.trajectory_ << "6. complete initialized TRAJECTORY in CHOMP_PLANNER before OPTIMIZATION..!!!!!" << std::endl;


  optimizer.optimize();
  ROS_DEBUG_NAMED("chomp_planner", "Optimization actually took %f sec to run",
                  (ros::WallTime::now() - create_time).toSec());
  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_DEBUG_NAMED("chomp_planner", "Output trajectory has %d joints", trajectory.getNumJoints());

  //std::cout << trajectory.trajectory_ << "7. complete initialized TRAJECTORY in CHOMP_PLANNER after OPTIMIZATION..!!!!!" << std::endl;

  res.trajectory.resize(1);

  res.trajectory[0].joint_trajectory.joint_names = active_joint_names;

  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;  // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory[0].joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i = 0; i < trajectory.getNumPoints(); i++)
  {
    res.trajectory[0].joint_trajectory.points[i].positions.resize(trajectory.getNumJoints());
    for (size_t j = 0; j < res.trajectory[0].joint_trajectory.points[i].positions.size(); j++)
    {
      res.trajectory[0].joint_trajectory.points[i].positions[j] = trajectory.getTrajectoryPoint(i)(j);
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory[0].joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  ROS_DEBUG_NAMED("chomp_planner", "Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_DEBUG_NAMED("chomp_planner", "Serviced planning request in %f wall-seconds, trajectory duration is %f",
                  (ros::WallTime::now() - start_time).toSec(),
                  res.trajectory[0].joint_trajectory.points[goal_index].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  // report planning failure if path has collisions
  if (not optimizer.isCollisionFree())
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // check that final state is within goal tolerances
  kinematic_constraints::JointConstraint jc(planning_scene->getRobotModel());
  robot_state::RobotState last_state(planning_scene->getRobotModel());
  last_state.setVariablePositions(res.trajectory[0].joint_trajectory.points.back().positions.data());

  bool constraints_are_ok = true;
  for (const moveit_msgs::JointConstraint& constraint : req.goal_constraints[0].joint_constraints)
  {
    constraints_are_ok = constraints_are_ok and jc.configure(constraint);
    constraints_are_ok = constraints_are_ok and jc.decide(last_state).satisfied;
    if (not constraints_are_ok)
    {
      res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
      return false;
    }
  }

  return true;
}
}
