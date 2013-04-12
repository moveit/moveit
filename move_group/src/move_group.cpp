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
    node_handle_("~")
  {
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution;
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);

    context_.reset(new MoveGroupContext(psm, allow_trajectory_execution, debug));

    // start the capabilities
    configureCapabilities();
  }
  
  ~MoveGroupServer()
  {
    capabilities_.clear();
    context_.reset();
  }
  
  void status()
  {
    const planning_interface::PlannerPtr &planner_interface = context_->planning_pipeline_->getPlannerInterface();
    if (planner_interface)
    {
      ROS_INFO_STREAM("MoveGroup running using planning plugin " << context_->planning_pipeline_->getPlannerPluginName());
      ROS_INFO_STREAM(" *** MoveGroup initialization complete !!!");
    }
    else
      ROS_WARN_STREAM("MoveGroup running was unable to load " << context_->planning_pipeline_->getPlannerPluginName());
  }
  
private:  
  
  void configureCapabilities()
  {
    // add individual capabilities move_group supports
    capabilities_.push_back(boost::make_shared<MoveGroupMoveAction>());
    capabilities_.push_back(boost::make_shared<MoveGroupPickPlaceAction>());
    capabilities_.push_back(boost::make_shared<MoveGroupPlanService>());
    capabilities_.push_back(boost::make_shared<MoveGroupExecuteService>());
    capabilities_.push_back(boost::make_shared<MoveGroupQueryPlannersService>()); 
    capabilities_.push_back(boost::make_shared<MoveGroupKinematicsService>());
    capabilities_.push_back(boost::make_shared<MoveGroupStateValidationService>());
    capabilities_.push_back(boost::make_shared<MoveGroupCartesianPathService>());
    for (std::size_t i = 0 ; i < capabilities_.size() ; ++i)
    {
      std::string brief, long_desc;
      capabilities_[i]->getDescription(brief, long_desc);
      ROS_INFO_STREAM("MoveGroup using " << brief);
      capabilities_[i]->setContext(context_);
      capabilities_[i]->initialize();
    }
  }
  
  ros::NodeHandle node_handle_;
  MoveGroupContextPtr context_;
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
