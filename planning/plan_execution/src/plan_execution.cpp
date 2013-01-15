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

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/collision_detection/collision_tools.h>
#include <boost/algorithm/string/join.hpp>

#include <dynamic_reconfigure/server.h>
#include <moveit_ros_planning/PlanExecutionDynamicReconfigureConfig.h>

namespace plan_execution
{
using namespace moveit_ros_planning;

class PlanExecution::DynamicReconfigureImpl
{ 
public:
  
  DynamicReconfigureImpl(PlanExecution *owner) : owner_(owner),
                                                 dynamic_reconfigure_server_(ros::NodeHandle("~/plan_execution"))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }
  
private:
  
  void dynamicReconfigureCallback(PlanExecutionDynamicReconfigureConfig &config, uint32_t level)
  {
    owner_->setMaxReplanAttempts(config.max_replan_attempts);
    owner_->setMaxSafePathCost(config.max_safe_path_cost);
    owner_->setMaxCostSources(config.max_cost_sources);
    owner_->setMaxLookAttempts(config.max_look_attempts);
    owner_->setTrajectoryStateRecordingFrequency(config.record_trajectory_state_frequency);
    owner_->setDiscardOverlappingCostSources(config.discard_overlapping_cost_sources);
  }
  
  PlanExecution *owner_;
  dynamic_reconfigure::Server<PlanExecutionDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

}

void plan_execution::PlanExecution::initialize(bool plan_only)
{
  plan_only_ = plan_only;
  if (!planning_pipeline_)
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel()));
  if (!plan_only_)
  {
    if (!trajectory_execution_manager_)
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getKinematicModel()));
    trajectory_monitor_.reset(new planning_scene_monitor::TrajectoryMonitor(planning_scene_monitor_->getStateMonitor()));
  }
  
  default_max_look_attempts_ = 3;
  default_max_safe_path_cost_ = 0.5;
    
  default_max_replan_attempts_ = 5;
  discard_overlapping_cost_sources_ = 0.8;
  max_cost_sources_ = 100;
  preempt_requested_ = false;
  new_scene_update_ = false;
  
  
  // we want to be notified when new information is available
  planning_scene_monitor_->addUpdateCallback(boost::bind(&PlanExecution::planningSceneUpdatedCallback, this, _1));
  
  // by default we do not display path cost sources
  display_cost_sources_ = false;
  
  // load the sensor manager plugin, if needed
  if (!plan_only_ && node_handle_.hasParam("moveit_sensor_manager"))
  {
    try
    {
      sensor_manager_loader_.reset(new pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager>("moveit_core", "moveit_sensor_manager::MoveItSensorManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating sensor manager plugin loader: " << ex.what());
    }
    if (sensor_manager_loader_)
    {
      std::string manager;
      if (node_handle_.getParam("moveit_sensor_manager", manager))
        try
        {
          sensor_manager_ = sensor_manager_loader_->createInstance(manager);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR_STREAM("Exception while loading sensor manager '" << manager << "': " << ex.what());
        } 
    }
    if (sensor_manager_)
    {
      std::vector<std::string> sensors;
      sensor_manager_->getSensorsList(sensors);
      ROS_INFO_STREAM("PlanExecution is aware of the following sensors: " << boost::algorithm::join(sensors, ", "));
    }
  }
  
  // when monitoring trajectories, every added state is passed to this callback
  if (trajectory_monitor_)
    trajectory_monitor_->setOnStateAddCallback(boost::bind(&PlanExecution::newMonitoredStateCallback, this, _1, _2));

  // start the dynamic-reconfigure server
  reconfigure_impl_ = new DynamicReconfigureImpl(this);
}

plan_execution::PlanExecution::PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor, bool plan_only) :
  node_handle_("~"), planning_scene_monitor_(planning_scene_monitor)
{
  initialize(plan_only);
}

plan_execution::PlanExecution::PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor, 
                                             const planning_pipeline::PlanningPipelinePtr &planning_pipeline) : 
  node_handle_("~"), planning_scene_monitor_(planning_scene_monitor),
  planning_pipeline_(planning_pipeline)
{
  initialize(true);
}

plan_execution::PlanExecution::PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor, 
                                             const planning_pipeline::PlanningPipelinePtr &planning_pipeline,
                                             const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution) :
  node_handle_("~"), planning_scene_monitor_(planning_scene_monitor),
  planning_pipeline_(planning_pipeline),
  trajectory_execution_manager_(trajectory_execution)
{ 
  initialize(false);
}

plan_execution::PlanExecution::~PlanExecution(void)
{
  delete reconfigure_impl_;
}

void plan_execution::PlanExecution::displayCostSources(bool flag)
{
  if (flag && !display_cost_sources_)
    // publisher for cost sources
    cost_sources_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("display_cost_sources", 100, true);
  else
    if (!flag && display_cost_sources_)
      cost_sources_publisher_.shutdown();
  display_cost_sources_ = flag;
}

void plan_execution::PlanExecution::planOnly(const moveit_msgs::MotionPlanRequest &mreq)
{
  planOnly(mreq, planning_scene_monitor_->getPlanningScene());
}

void plan_execution::PlanExecution::planOnly(const moveit_msgs::MotionPlanRequest &mreq, const moveit_msgs::PlanningScene &scene_diff)
{
  planning_scene::PlanningSceneConstPtr the_scene = planning_scene_monitor_->getPlanningScene();
  if (!planning_scene::PlanningScene::isEmpty(scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
    the_scene = lscene->diff(scene_diff);
  }
  planOnly(mreq, the_scene);
}

void plan_execution::PlanExecution::planAndExecute(const moveit_msgs::MotionPlanRequest &mreq, const Options &opt)
{  
  planAndExecute(mreq, planning_scene_monitor_->getPlanningScene(), opt);
}

void plan_execution::PlanExecution::planAndExecute(const moveit_msgs::MotionPlanRequest &mreq, const moveit_msgs::PlanningScene &scene_diff, const Options &opt)
{ 
  planning_scene::PlanningSceneConstPtr the_scene = planning_scene_monitor_->getPlanningScene();
  if (!planning_scene::PlanningScene::isEmpty(scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
    the_scene = lscene->diff(scene_diff);
  }
  planAndExecute(mreq, the_scene, opt);
}

void plan_execution::PlanExecution::stop(void)
{
  preempt_requested_ = true;
}
 
void plan_execution::PlanExecution::planOnly(const moveit_msgs::MotionPlanRequest &req, const planning_scene::PlanningSceneConstPtr &the_scene)
{
  moveit_msgs::GetMotionPlan::Request mreq;
  mreq.motion_plan_request = req;

  // run the motion planner
  moveit_msgs::GetMotionPlan::Response mres;

  planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
  computePlan(the_scene, mreq, mres);
  result_ = Result();
  result_.error_code_ = mres.error_code;
  result_.trajectory_start_ = mres.trajectory_start;
  result_.planned_trajectory_ = mres.trajectory; 
}

void plan_execution::PlanExecution::planAndExecute(const moveit_msgs::MotionPlanRequest &req, const planning_scene::PlanningSceneConstPtr &the_scene, const Options &opt)
{
  // perform initial configuration steps & various checks
  preempt_requested_ = false;
  result_ = Result();
  
  if (req.group_name.empty())
  {
    ROS_WARN_STREAM("Must specify group in motion plan request");
    result_.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return;
  }

  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    // check to see if the desired constraints are already met
    for (std::size_t i = 0 ; i < req.goal_constraints.size() ; ++i)
      if (the_scene->isStateConstrained(req.start_state,
                                        kinematic_constraints::mergeConstraints(req.goal_constraints[i],
                                                                                req.path_constraints)))
      {
        ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
        result_.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return;
      }
  }
  
  // run the actual motion plan & execution
  unsigned int max_replan_attempts = opt.replan_attempts_ > 0 ? opt.replan_attempts_ : default_max_replan_attempts_;
  unsigned int replan_attempts = 0;

  double max_safe_path_cost = opt.max_safe_path_cost_ > std::numeric_limits<double>::epsilon() ? opt.max_safe_path_cost_ : default_max_safe_path_cost_;
  double previous_cost = 0.0;
  unsigned int max_look_attempts = opt.look_attempts_ > 0 ? opt.look_attempts_ : default_max_look_attempts_;
  unsigned int look_attempts = 0;

  // this flag is set to true when all conditions for looking around are met, and the command is sent.
  // the intention is for the planning looop not to terminate when having just looked around
  bool just_looked_around = false;
  
  // this flag indicates whether the last lookAt() operation failed. If this operation fails once, we assume that 
  // maybe some information was gained anyway (the sensor moved part of the way) and we try to plan one more time.
  // If we have two sensor pointing failures in a row, we fail
  bool look_around_failed = false;
  
  moveit_msgs::GetMotionPlan::Request mreq;
  mreq.motion_plan_request = req;
  
  // run a planning loop for at most the maximum replanning attempts;
  // re-planning is executed only in case of known types of failures (e.g., environment changed)
  // there can be a maximum number of looking attempts as well that lead to replanning, if the cost
  // of the path is above a maximum threshold.
  do
  {
    replan_attempts++;
    just_looked_around = false;
    ROS_DEBUG("Planning attempt %u", replan_attempts);
    
    if (opt.beforePlanCallback_)
      opt.beforePlanCallback_();
    
    // run the motion planner
    moveit_msgs::GetMotionPlan::Response mres;
    bool solved = false;
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
      solved = computePlan(the_scene, mreq, mres);
    }
    
    result_.error_code_ = mres.error_code;
    result_.trajectory_start_ = mres.trajectory_start;
    result_.planned_trajectory_ = mres.trajectory;
    
    // if planning fails in a manner that is not recoverable, we exit the loop,
    // otherwise, we attempt to continue, if replanning attempts are lefr
    if (result_.error_code_.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN ||
        result_.error_code_.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
      continue;

    // abort if no plan was found
    if (!solved)
      break;

    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
      // convert the path to a sequence of kinematic states
      trajectory_processing::convertToKinematicStates(result_.planned_trajectory_states_, mres.trajectory_start, mres.trajectory, the_scene->getCurrentState(), the_scene->getTransforms());
    }
    
    if (opt.look_around_)
    {       
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
      
      // determine the sources of cost for this path
      std::set<collision_detection::CostSource> cost_sources;
      the_scene->getCostSources(result_.planned_trajectory_states_, max_cost_sources_, mreq.motion_plan_request.group_name, cost_sources, discard_overlapping_cost_sources_);

      if (display_cost_sources_)
      {
        visualization_msgs::MarkerArray arr;
        collision_detection::getCostMarkers(arr, the_scene->getPlanningFrame(), cost_sources);
        cost_sources_publisher_.publish(arr);
      }
      
      double cost = collision_detection::getTotalCost(cost_sources);
      ROS_DEBUG("The total cost of the trajectory is %lf.", cost);
      if (previous_cost > 0.0)
        ROS_DEBUG("The change in the trajectory cost is %lf after the perception step.", cost - previous_cost);
      if (cost > max_safe_path_cost && look_attempts < max_look_attempts)
      {
        ++look_attempts;
        ROS_INFO("The cost of the trajectory is %lf, which is above the maximum safe cost of %lf. Attempt %u (of at most %u) at looking around.", cost, max_safe_path_cost, look_attempts, max_look_attempts);
        if (opt.beforeLookCallback_)
          opt.beforeLookCallback_();

        bool looked_at_result = lookAt(cost_sources);
        if (looked_at_result)
          ROS_INFO("Sensor was succesfully actuated. Attempting to recompute a motion plan.");
        else
        {
          if (look_around_failed)
            ROS_WARN("Looking around seems to keep failing. Giving up.");
          else
            ROS_WARN("Looking around seems to have failed. Attempting to recompute a motion plan anyway.");
        }
        if (looked_at_result || !look_around_failed)
        {
          previous_cost = cost;
          replan_attempts--; // this was not a replanning attempt
          just_looked_around = true;
        }
        look_around_failed = !looked_at_result;
        // if we are unable to look, let this loop continue into the next if statement
        if (just_looked_around)
          continue;
      }
      
      if (cost > max_safe_path_cost)
      {
        result_.error_code_.val = moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA;
        continue;
      }
    }
    
    if (result_.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      if (opt.beforeExecutionCallback_)
        opt.beforeExecutionCallback_();
      // execute the trajectory, and monitor its execution
      executeAndMonitor(the_scene, mreq.motion_plan_request);
    }
    
    // if we are done, then we exit the loop
    if (result_.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      break;

    // if execution failed in a manner that we do not consider recoverable, we exit the loop (with failure)
    if (result_.error_code_.val != moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE && 
        result_.error_code_.val != moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
      break;
  } while ((opt.replan_ && max_replan_attempts > replan_attempts) || just_looked_around);
  
  if (opt.doneCallback_)
    opt.doneCallback_();
  
  ROS_DEBUG("PlanExecution terminating with error code %d", result_.error_code_.val);
}

bool plan_execution::PlanExecution::computePlan(const planning_scene::PlanningSceneConstPtr &scene, const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
{
  bool solved = false;   
 
  try
  {
    solved = planning_pipeline_->generatePlan(scene, req, res);
    new_scene_update_ = false; // we just computed a plan with the latest scene (which was locked)
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
  }
  
  return solved;
}

void plan_execution::PlanExecution::executeAndMonitor(const planning_scene::PlanningSceneConstPtr &the_scene,
                                                      const moveit_msgs::MotionPlanRequest &req)
{
  // try to execute the trajectory
  execution_complete_ = false;
  
  if (!trajectory_execution_manager_)
  {
    ROS_ERROR("No trajectory execution manager");
    result_.error_code_.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    return;
  }

  if (trajectory_execution_manager_->push(result_.planned_trajectory_))
  {
    currently_executed_trajectory_index_ = 0;
    trajectory_monitor_->startTrajectoryMonitor();
    
    // start a trajectory execution thread
    trajectory_execution_manager_->execute(boost::bind(&PlanExecution::doneWithTrajectoryExecution, this, _1));
    
    // wait for path to be done, while checking that the path does not become invalid
    static const ros::WallDuration d(0.01);
    bool path_became_invalid = false;
    boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> path_constraints_set;
    {    
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while getTransforms() is called
      path_constraints_set.reset(new kinematic_constraints::KinematicConstraintSet(the_scene->getKinematicModel(), the_scene->getTransforms()));
      path_constraints_set->add(req.path_constraints);
    }
    
    while (node_handle_.ok() && !execution_complete_ && !preempt_requested_ && !path_became_invalid)
    {
      d.sleep();
      // check the path if there was an environment update in the meantime
      if (new_scene_update_)
      {
        planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while isStateValid() is called
        new_scene_update_ = false;
        for (std::size_t i = currently_executed_trajectory_index_ ; i < result_.planned_trajectory_states_.size() ; ++i)
          if (!the_scene->isStateValid(*result_.planned_trajectory_states_[i], *path_constraints_set, req.group_name, false))
          {
            the_scene->isStateValid(*result_.planned_trajectory_states_[i], *path_constraints_set, req.group_name, true);
            path_became_invalid = true;
            break;
          }
      }
    }
    
    if (preempt_requested_)
    {
      ROS_INFO("Stopping execution due to preempt request");
      trajectory_execution_manager_->stopExecution();
    }
    else
      if (path_became_invalid)
      {
        ROS_INFO("Stopping execution because the path to execute became invalid (probably the environment changed)");
        trajectory_execution_manager_->stopExecution();
      }
      else
        if (!execution_complete_)
        {    
          ROS_WARN("Stopping execution due to unknown reason. Possibly the node is about to shut down.");
          trajectory_execution_manager_->stopExecution();
        }
    
    // because of the way the trajectory monitor works, when this call completes, it is certain that no more calls
    // are made to the callback associated to the trajectory monitor
    trajectory_monitor_->stopTrajectoryMonitor();
    trajectory_monitor_->getTrajectory(result_.executed_trajectory_);
    
    if (trajectory_execution_manager_->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      result_.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
    {
      if (path_became_invalid)
        result_.error_code_.val = moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
      else
      {
        if (preempt_requested_)
        {
          result_.error_code_.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        }
        else
        {
          if (trajectory_execution_manager_->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            result_.error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
          else
            result_.error_code_.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
        }
      }
    }
  }
  else
  {
    ROS_INFO_STREAM("Apparently trajectory initialization failed");
    result_.error_code_.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
}

bool plan_execution::PlanExecution::lookAt(const std::set<collision_detection::CostSource> &cost_sources)
{  
  if (!sensor_manager_)
    return false;
  std::vector<std::string> names;
  sensor_manager_->getSensorsList(names);
  geometry_msgs::PointStamped point;
  for (std::size_t i = 0 ; i < names.size() ; ++i)
    if (collision_detection::getSensorPositioning(point.point, cost_sources))
    {      
      point.header.stamp = ros::Time::now();
      point.header.frame_id = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
      ROS_DEBUG_STREAM("Pointing sensor " << names[i] << " to:\n" << point);
      moveit_msgs::RobotTrajectory sensor_trajectory;
      if (sensor_manager_->pointSensorTo(names[i], point, sensor_trajectory))
      {
        if (!trajectory_processing::isTrajectoryEmpty(sensor_trajectory))
          return trajectory_execution_manager_->push(sensor_trajectory) && trajectory_execution_manager_->executeAndWait();
        else
          return true;
      }
    }
  return false;
}

void plan_execution::PlanExecution::planningSceneUpdatedCallback(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & (planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY | planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS))
    new_scene_update_ = true;
}

void plan_execution::PlanExecution::newMonitoredStateCallback(const kinematic_state::KinematicStateConstPtr &state, const ros::Time &stamp)
{           
  // find the index where the distance to the current state starts increasing (heuristic)
  double dist = result_.planned_trajectory_states_[currently_executed_trajectory_index_]->distance(*state);
  for (std::size_t i = currently_executed_trajectory_index_ + 1 ; i < result_.planned_trajectory_states_.size() ; ++i)
  {
    double d = result_.planned_trajectory_states_[i]->distance(*state);
    if (d >= dist)
    {
      currently_executed_trajectory_index_ = i - 1;
      break;
    }
    else
      dist = d;
  }
  std::pair<int, int> expected = trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex();
  
  ROS_DEBUG("Controller seems to be at state %lu of %lu. Trajectory execution manager expects the index to be %d.",
            currently_executed_trajectory_index_ + 1, result_.planned_trajectory_states_.size(), expected.second);
}

void plan_execution::PlanExecution::doneWithTrajectoryExecution(const moveit_controller_manager::ExecutionStatus &status)
{
  execution_complete_ = true;
}
