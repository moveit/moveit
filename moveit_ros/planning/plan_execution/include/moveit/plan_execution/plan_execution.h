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

#ifndef MOVEIT_PLAN_EXECUTION_PLAN_EXECUTION_
#define MOVEIT_PLAN_EXECUTION_PLAN_EXECUTION_

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/sensor_manager/sensor_manager.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

namespace plan_execution
{

class PlanExecution
{
public:

  struct Options
  {
    Options(void) : look_around_(false),
                    replan_(false),
                    look_attempts_(0),
                    replan_attempts_(0),
                    max_safe_path_cost_(0.0)
    {
    }
    
    bool look_around_;
    bool replan_;
    unsigned int look_attempts_;
    unsigned int replan_attempts_;
    double max_safe_path_cost_;
    
    boost::function<void(void)> beforePlanCallback_;
    boost::function<void(void)> beforeLookCallback_;
    boost::function<void(void)> beforeExecutionCallback_;
    boost::function<void(void)> doneCallback_;
  };
  
  struct Result
  {
    // The full starting state of the robot at the start of the trajectory
    moveit_msgs::RobotState trajectory_start_;

    // The trajectory that moved group produced for execution
    moveit_msgs::RobotTrajectory planned_trajectory_;
    
    // The states that make up the planned trajectory are needed for various computation steps
    std::vector<kinematic_state::KinematicStatePtr> planned_trajectory_states_;
    
    // The trace of the trajectory recorded during execution
    moveit_msgs::RobotTrajectory executed_trajectory_;

    // An error code reflecting what went wrong (if anything)
    moveit_msgs::MoveItErrorCodes error_code_;
  };

  PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor);
  ~PlanExecution(void);

  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor(void) const
  {
    return planning_scene_monitor_;
  }
  
  planning_pipeline::PlanningPipeline& getPlanningPipeline(void)
  {
    return planning_pipeline_;
  }
  
  trajectory_execution_manager::TrajectoryExecutionManager& getTrajectoryExecutionManager(void)
  {
    return trajectory_execution_manager_;
  }

  double getMaxSafePathCost(void) const
  {
    return default_max_safe_path_cost_;
  }
  
  void setMaxSafePathCost(double max_safe_path_cost)
  {
    default_max_safe_path_cost_ = max_safe_path_cost;
  }
  
  double getTrajectoryStateRecordingFrequency(void) const
  {
    return trajectory_monitor_.getSamplingFrequency();
  }
  
  void setTrajectoryStateRecordingFrequency(double freq)
  {
    trajectory_monitor_.setSamplingFrequency(freq);
  }

  void setMaxReplanAttempts(unsigned int attempts)
  {
    default_max_replan_attempts_ = attempts;
  }
  
  unsigned int getMaxReplanAttempts(void) const
  {
    return default_max_replan_attempts_;
  }

  void setMaxLookAttempts(unsigned int attempts)
  {
    default_max_look_attempts_ = attempts;
  }
  
  unsigned int getMaxLookAttempts(void) const
  {
    return default_max_look_attempts_;
  }

  unsigned int getMaxCostSources(void) const
  {
    return max_cost_sources_;
  }
  
  void setMaxCostSources(unsigned int value)
  {
    max_cost_sources_ = value;
  }
  
  double getDiscardOverlappingCostSources(void) const
  {
    return discard_overlapping_cost_sources_;
  }
  
  void setDiscardOverlappingCostSources(double value)
  {
    discard_overlapping_cost_sources_ = value;
  }
  
  void planAndExecute(const moveit_msgs::MotionPlanRequest &mreq, const Options &opt = Options());
  void planAndExecute(const moveit_msgs::MotionPlanRequest &mreq, const moveit_msgs::PlanningScene &scene_diff, const Options &opt = Options());  

  void planOnly(const moveit_msgs::MotionPlanRequest &mreq);
  void planOnly(const moveit_msgs::MotionPlanRequest &mreq, const moveit_msgs::PlanningScene &scene_diff);  

  void stop(void);

  const Result& getLastResult(void) const
  {
    return result_;
  }
  
  void displayCostSources(bool flag);

private:

  void planAndExecute(const moveit_msgs::MotionPlanRequest &mreq, const planning_scene::PlanningSceneConstPtr &the_scene, const Options &opt);
  void planOnly(const moveit_msgs::MotionPlanRequest &mreq, const planning_scene::PlanningSceneConstPtr &the_scene);
  bool computePlan(const planning_scene::PlanningSceneConstPtr &scene, const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res);
  bool lookAt(const std::set<collision_detection::CostSource> &cost_sources);
  void executeAndMonitor(const planning_scene::PlanningSceneConstPtr &the_scene, const moveit_msgs::Constraints &path_constraints);
  
  void planningSceneUpdatedCallback(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  void newMonitoredStateCallback(const kinematic_state::KinematicStateConstPtr &state, const ros::Time &stamp);
  void doneWithTrajectoryExecution(const moveit_controller_manager::ExecutionStatus &status);
  
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_pipeline::PlanningPipeline planning_pipeline_;
  trajectory_execution_manager::TrajectoryExecutionManager trajectory_execution_manager_;
  planning_scene_monitor::TrajectoryMonitor trajectory_monitor_;

  boost::scoped_ptr<pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager> > sensor_manager_loader_;
  moveit_sensor_manager::MoveItSensorManagerPtr sensor_manager_;
  unsigned int default_max_look_attempts_;
  double default_max_safe_path_cost_;

  unsigned int default_max_replan_attempts_;
  double discard_overlapping_cost_sources_;
  unsigned int max_cost_sources_;
  
  bool display_cost_sources_;
  ros::Publisher cost_sources_publisher_;
  
  
  bool preempt_requested_;
  bool new_scene_update_;
  bool execution_complete_;
  std::size_t currently_executed_trajectory_index_;
  
  Result result_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl *reconfigure_impl_;
};

}
#endif
