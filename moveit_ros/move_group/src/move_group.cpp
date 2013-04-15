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

#include <moveit/move_group/names.h>
#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/pick_place/pick_place.h>

#include <moveit/move_group/move_group_execute_service_capability.h>
#include <moveit/move_group/move_group_move_action_capability.h>
#include <moveit/move_group/move_group_pick_place_action_capability.h>
#include <moveit/move_group/move_group_plan_service_capability.h>
#include <moveit/move_group/move_group_query_planners_service_capability.h>
#include <moveit/move_group/move_group_kinematics_service_capability.h>
#include <moveit/move_group/move_group_state_validation_service_capability.h>
#include <moveit/move_group/move_group_cartesian_path_service_capability.h>

namespace move_group
{
  
class MoveGroupServer
{
public:
  
  MoveGroupServer(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug) : 
    node_handle_("~"),
    planning_scene_monitor_(psm),
    allow_trajectory_execution_(true),
    debug_(debug)
  { 
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getRobotModel()));
    
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution_, true);
    
    if (allow_trajectory_execution_)
    {  
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getRobotModel()));
      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
      plan_with_sensing_.reset(new plan_execution::PlanWithSensing(trajectory_execution_manager_));
      if (debug)
        plan_with_sensing_->displayCostSources(true);
    }
    
    pick_place_.reset(new pick_place::PickPlace(planning_pipeline_));
    
    // configure the planning pipeline
    planning_pipeline_->displayComputedMotionPlans(true);
    planning_pipeline_->checkSolutionPaths(true);
    
    pick_place_->displayComputedMotionPlans(true);
    
    if (debug_)
    {
      planning_pipeline_->publishReceivedRequests(true);
      pick_place_->displayProcessedGrasps(true);
    }

    // start the capabilities
    configureCapabilities();
  }
  
  ~MoveGroupServer()
  {
    capabilities_.clear();
    planning_scene_monitor_.reset();
  }
  
  void status()
  {
    const planning_interface::PlannerPtr &planner_interface = planning_pipeline_->getPlannerInterface();
    if (planner_interface)
    {
      ROS_INFO_STREAM("MoveGroup running using planning plugin " << planning_pipeline_->getPlannerPluginName());
      ROS_INFO_STREAM(" *** MoveGroup initialization complete !!!");
    }
    else
      ROS_WARN_STREAM("MoveGroup running was unable to load " << planning_pipeline_->getPlannerPluginName());
  }
  
private:  
  
  void configureCapabilities()
  {
    // add individual capabilities move_group supports
    capabilities_.push_back(boost::make_shared<MoveGroupMoveAction>(planning_scene_monitor_, planning_pipeline_, plan_execution_, plan_with_sensing_, allow_trajectory_execution_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupPickPlaceAction>(planning_scene_monitor_, pick_place_, plan_execution_, plan_with_sensing_, allow_trajectory_execution_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupPlanService>(planning_scene_monitor_, planning_pipeline_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupExecuteService>(planning_scene_monitor_, trajectory_execution_manager_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupQueryPlannersService>(planning_scene_monitor_, planning_pipeline_, debug_)); 
    capabilities_.push_back(boost::make_shared<MoveGroupKinematicsService>(planning_scene_monitor_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupStateValidationService>(planning_scene_monitor_, debug_));
    capabilities_.push_back(boost::make_shared<MoveGroupCartesianPathService>(planning_scene_monitor_, debug_));
  }
  
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  plan_execution::PlanExecutionPtr plan_execution_;
  plan_execution::PlanWithSensingPtr plan_with_sensing_;
  pick_place::PickPlacePtr pick_place_;
  bool allow_trajectory_execution_;
  bool debug_;
  
  std::vector<boost::shared_ptr<MoveGroupCapability> > capabilities_;
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, move_group::NODE_NAME);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
  
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(move_group::ROBOT_DESCRIPTION, tf));
  
  if (planning_scene_monitor->getPlanningScene())
  {
    bool debug = false;
    for (int i = 1 ; i < argc ; ++i)
      if (strncmp(argv[i], "--debug", 7) == 0)
      {
        debug = true;
        break;
      }
    if (debug)
      ROS_INFO("MoveGroup debug mode is ON");
    else
      ROS_INFO("MoveGroup debug mode is OFF");

    move_group::MoveGroupServer mgs(planning_scene_monitor, debug);

    planning_scene_monitor->startSceneMonitor();    
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    planning_scene_monitor->publishDebugInformation(debug);
    
    mgs.status();
    
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
