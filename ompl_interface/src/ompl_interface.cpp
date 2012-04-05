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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface/ompl_interface.h"
#include <planning_models/conversions.h>
#include <ompl/tools/debug/Profiler.h>

ompl_interface::OMPLInterface::OMPLInterface(const planning_models::KinematicModelConstPtr &kmodel) :
  kmodel_(kmodel), context_manager_(kmodel), constraints_library_(context_manager_), use_constraints_approximations_(true)
{
}

ompl_interface::OMPLInterface::~OMPLInterface(void)
{
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::prepareForSolve(const moveit_msgs::MotionPlanRequest &req, 
                                                                                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                                            moveit_msgs::MoveItErrorCodes *error_code,
                                                                                            unsigned int *attempts, double *timeout,
                                                                                            moveit_msgs::RobotState *prefix_state,
                                                                                            moveit_msgs::RobotTrajectory *prefix_trajectory,
                                                                                            double *prefix_plan_time) const
{
  ros::WallTime start = ros::WallTime::now();
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.start_state, start_state);

  // if the starting state does not satisfy path constraints but is otherwise valid, we attempt to compute an additional 
  // motion plan towards a state that satisfies the path constraints
  if (prefix_state && prefix_trajectory &&
      !planning_scene->isStateValid(start_state, req.path_constraints) && planning_scene->isStateValid(start_state))
  {
    ROS_INFO("Robot starting state seems valid but does not satisfy path constraints. Attempting to first plan to a state that does satisfy path constraints");
    moveit_msgs::GetMotionPlan::Request extra_req;
    moveit_msgs::GetMotionPlan::Response extra_res;
    extra_req.motion_plan_request = req;
    extra_req.motion_plan_request.goal_constraints.resize(1);
    extra_req.motion_plan_request.goal_constraints[0] = extra_req.motion_plan_request.path_constraints;
    extra_req.motion_plan_request.path_constraints = moveit_msgs::Constraints();
    if (solve(planning_scene, extra_req, extra_res))
    {
      *prefix_state = extra_res.trajectory_start;
      *prefix_trajectory = extra_res.trajectory;
      if (prefix_plan_time)
        *prefix_plan_time = extra_res.planning_time.toSec();
      ROS_INFO("Trajectory that moves towards valid path constraints was successfully computed. Continuing with motion plan request");
    }
    else
      ROS_WARN("Unable to compute trajectory that moves to a state that satisfies path constraints. Attempting planning anyway (but it may fail)");
  }

  ModelBasedPlanningContextPtr context = getPlanningContext(req);
  if (!context)
  {
    error_code->val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return context;
  }
  
  *timeout = req.allowed_planning_time.toSec();
  if (*timeout <= 0.0)
  {
    error_code->val = moveit_msgs::MoveItErrorCodes::INVALID_ALLOWED_PLANNING_TIME;
    ROS_INFO("The timeout for planning must be positive (%lf specified). Assuming one second instead.", *timeout);
    *timeout = 1.0;
  }
  
  *attempts = 1;
  if (req.num_planning_attempts > 0)
    *attempts = req.num_planning_attempts;
  else
    if (req.num_planning_attempts < 0)
      ROS_ERROR("The number of desired planning attempts should be positive. Assuming one attempt.");
  
  context->clear();
  
  // set the planning scene
  context->setPlanningScene(planning_scene);

  // if we have a path prefix, then we set the starting state to be the last path on that prefix
  if (prefix_trajectory && (prefix_trajectory->joint_trajectory.points.size() > 0 || 
                            prefix_trajectory->multi_dof_joint_trajectory.points.size() > 0))
  {
    std::size_t last_index = std::max(prefix_trajectory->joint_trajectory.points.size(),
                                      prefix_trajectory->multi_dof_joint_trajectory.points.size());
    moveit_msgs::RobotState rs;
    planning_models::robotTrajectoryPointToRobotState(*prefix_trajectory, last_index - 1, rs);
    planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), rs, start_state);
  }
  context->setStartState(start_state);
  
  context->setPlanningVolume(req.workspace_parameters);
  if (!context->setPathConstraints(req.path_constraints, error_code))
    return ModelBasedPlanningContextPtr();
  if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, error_code))
    return ModelBasedPlanningContextPtr();
  context->configure();
  ROS_DEBUG("%s: New planning context is set.", context->getName().c_str());
  error_code->val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  *timeout = std::max(*timeout - (ros::WallTime::now() - start).toSec(), 0.0);  
  
  return context;
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const
{
  ompl::tools::Profiler::ScopedStart pslv;
  ot::Profiler::ScopedBlock sblock("OMPLInterfaceSolve");
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  moveit_msgs::RobotState prefix_state;
  moveit_msgs::RobotTrajectory prefix_trajectory;
  double prefix_plan_time = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, &res.error_code, &attempts, &timeout,
                                                         &prefix_state, &prefix_trajectory, &prefix_plan_time);
  if (!context)
    return false;
  
  if (context->solve(timeout, attempts))
  {
    double ptime = prefix_plan_time + context->getLastPlanTime();
    if (ptime < timeout)
    {
      context->simplifySolution(timeout - ptime);
      ptime += context->getLastSimplifyTime();
    }
    context->interpolateSolution();
      
    // fill the response
    ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
	      context->getOMPLSimpleSetup().getSolutionPath().getStateCount());

    // add the path prefix, if needed
    if (prefix_trajectory.joint_trajectory.points.size() > 0 || 
        prefix_trajectory.multi_dof_joint_trajectory.points.size() > 0)
    {
      res.trajectory_start = prefix_state;
      context->getSolutionPath(res.trajectory);
      if (!prefix_trajectory.joint_trajectory.points.empty())
        res.trajectory.joint_trajectory.points.insert(res.trajectory.joint_trajectory.points.begin(),
                                                      ++prefix_trajectory.joint_trajectory.points.begin(),
                                                      prefix_trajectory.joint_trajectory.points.end());
      if (!prefix_trajectory.multi_dof_joint_trajectory.points.empty())
        res.trajectory.multi_dof_joint_trajectory.points.insert(res.trajectory.multi_dof_joint_trajectory.points.begin(),
                                                                ++prefix_trajectory.multi_dof_joint_trajectory.points.begin(),
                                                                prefix_trajectory.multi_dof_joint_trajectory.points.end());
    }
    else
    {
      // we have no path prefix ...
      pm::kinematicStateToRobotState(context->getCompleteInitialRobotState(), res.trajectory_start);
      context->getSolutionPath(res.trajectory);
    }
    res.planning_time = ros::Duration(ptime);
    return true;
  }
  else
  {
    ROS_INFO("Unable to solve the planning problem");
    return false;
  }
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
					  const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::MotionPlanDetailedResponse &res) const
{
  ompl::tools::Profiler::ScopedStart pslv;
  ot::Profiler::ScopedBlock sblock("OMPLInterfaceSolve");
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  moveit_msgs::MoveItErrorCodes error_code;
  moveit_msgs::RobotState prefix_state;
  moveit_msgs::RobotTrajectory prefix_trajectory;
  double prefix_plan_time = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, &error_code, &attempts, &timeout,
                                                         &prefix_state, &prefix_trajectory, &prefix_plan_time);
  if (!context)
    return false;
  
  res.trajectory.reserve(3);

  bool have_prefix = false;
  // if we have a prefix path, we add it as the first part of the detailed response
  if (prefix_trajectory.joint_trajectory.points.size() > 0 ||  prefix_trajectory.multi_dof_joint_trajectory.points.size() > 0)
  {
    have_prefix = true;
    res.trajectory_start = prefix_state;
    res.description.push_back("prefix");
    res.processing_time.push_back(ros::Duration(prefix_plan_time));
    res.trajectory.push_back(prefix_trajectory);
  }
  else
    // if there is no prefix, we just set the trajectory start
    pm::kinematicStateToRobotState(context->getCompleteInitialRobotState(), res.trajectory_start);
  
  if (context->solve(timeout, attempts))
  {
    // add info about planned solution
    double ptime = context->getLastPlanTime();
    res.processing_time.push_back(ros::Duration(ptime));
    res.description.push_back("plan");
    res.trajectory.resize(res.trajectory.size() + 1);
    context->getSolutionPath(res.trajectory.back());
    if (have_prefix)
    {
      // add the prefix
      if (!prefix_trajectory.joint_trajectory.points.empty())
        res.trajectory.back().joint_trajectory.points.insert(res.trajectory.back().joint_trajectory.points.begin(),
                                                             ++prefix_trajectory.joint_trajectory.points.begin(),
                                                             prefix_trajectory.joint_trajectory.points.end());
      if (!prefix_trajectory.multi_dof_joint_trajectory.points.empty())
        res.trajectory.back().multi_dof_joint_trajectory.points.insert(res.trajectory.back().multi_dof_joint_trajectory.points.begin(),
                                                                       ++prefix_trajectory.multi_dof_joint_trajectory.points.begin(),
                                                                       prefix_trajectory.multi_dof_joint_trajectory.points.end());
    }
    
    // simplify solution if time remains
    if (ptime < timeout)
    {
      context->simplifySolution(timeout - ptime);
      res.processing_time.push_back(ros::Duration(context->getLastSimplifyTime()));
      res.description.push_back("simplify");
      res.trajectory.resize(res.trajectory.size() + 1);
      context->getSolutionPath(res.trajectory.back());
    }

    ros::WallTime start_interpolate = ros::WallTime::now();
    context->interpolateSolution();
    res.processing_time.push_back(ros::Duration((ros::WallTime::now() - start_interpolate).toSec()));
    res.description.push_back("interpolate");
    res.trajectory.resize(res.trajectory.size() + 1);
    context->getSolutionPath(res.trajectory.back());

    // fill the response
    ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
	      context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    ROS_INFO("Unable to solve the planning problem");
    return false;
  }
}

bool ompl_interface::OMPLInterface::benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const moveit_msgs::ComputePlanningBenchmark::Request &req,
                                              moveit_msgs::ComputePlanningBenchmark::Response &res) const
{  
  unsigned int attempts = 1;
  double timeout = 0.0;
  moveit_msgs::RobotState prefix_state;
  moveit_msgs::RobotTrajectory prefix_trajectory;
  double prefix_plan_time = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, &res.error_code, &attempts, &timeout,
                                                         &prefix_state, &prefix_trajectory, &prefix_plan_time);
  // prefix is not important for benchmarking, so we ignore it
  if (!context)
    return false;
  return context->benchmark(timeout, attempts, req.filename);
}

ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints, double timeout,
                                                         const std::string &factory_type) const
{
  moveit_msgs::Constraints empty;
  return solve(planning_scene, config, start_state, goal_constraints, empty, timeout, factory_type);
}

ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints,
                                                         const moveit_msgs::Constraints &path_constraints, double timeout,
                                                         const std::string &factory_type) const
{ 
  ompl::tools::Profiler::ScopedStart pslv;
  ob::PathPtr prefix;
  ros::WallTime start_time = ros::WallTime::now();
  
  if (!planning_scene->isStateValid(start_state, path_constraints) && planning_scene->isStateValid(start_state))
  {
    ROS_INFO("Robot starting state seems valid but does not satisfy path constraints. Attempting to first plan to a state that does satisfy path constraints");
    prefix = solve(planning_scene, config, start_state, path_constraints, moveit_msgs::Constraints(), timeout, factory_type);
    if (prefix)
      ROS_INFO("Trajectory that moves towards valid path constraints was successfully computed. Continuing with motion plan request");
    else
      ROS_WARN("Unable to compute trajectory that moves to a state that satisfies path constraints. Attempting planning anyway (but it may fail)");
  }
  
  ModelBasedPlanningContextPtr context = getPlanningContext(config, factory_type);
  if (!context)
    return ob::PathPtr();
  
  std::vector<moveit_msgs::Constraints> goal_constraints_v(1, goal_constraints);  
  context->setPlanningScene(planning_scene);
  if (prefix)
  {
    planning_models::KinematicState int_state = start_state;
    context->getOMPLStateSpace()->copyToKinematicState(int_state, static_cast<og::PathGeometric*>(prefix.get())->getStates().back());
    context->setStartState(int_state);
  }
  else
    context->setStartState(start_state);
  context->setPathConstraints(path_constraints, NULL);
  context->setGoalConstraints(goal_constraints_v, path_constraints, NULL);
  context->configure();
  
  // update remaining time
  timeout = std::max(0.0, timeout - (ros::WallTime::now() - start_time).toSec());
  
  // solve the planning problem
  if (context->solve(timeout, 1))
  {
    double ptime = context->getLastPlanTime();
    if (ptime < timeout)
      context->simplifySolution(timeout - ptime);
    context->interpolateSolution();
    if (prefix)
    {
      const og::PathGeometric &suffix = context->getOMPLSimpleSetup().getSolutionPath();
      og::PathGeometric &res = static_cast<og::PathGeometric&>(*prefix);
      res.overlay(suffix, res.getStateCount() - 1);
      return prefix;
    }
    else
      return ob::PathPtr(new og::PathGeometric(context->getOMPLSimpleSetup().getSolutionPath()));
  }
  
  return ob::PathPtr();  
}

void ompl_interface::OMPLInterface::terminateSolve(void)
{
  const ModelBasedPlanningContextPtr &context = getLastPlanningContext();
  if (context)
    context->terminateSolve();
}
