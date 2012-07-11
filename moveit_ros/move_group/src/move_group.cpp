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

#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/MoveGroupAction.h>

#include <tf/transform_listener.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <planning_scene_monitor/trajectory_monitor.h>
#include <trajectory_execution_manager/trajectory_execution_manager.h>
#include <planning_pipeline/planning_pipeline.h>
#include <kinematic_constraints/utils.h>
#include <trajectory_processing/trajectory_tools.h>
#include <planning_models/conversions.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";    // name of the robot description (a param name, so it can be changed externally)
static const std::string NODE_NAME = "move_group";
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path"; // name of the advertised service (within the ~ namespace)

class MoveGroupAction
{
public:
  
  enum MoveGroupState
    {
      IDLE,
      PLANNING,
      MONITOR
    };
  
  MoveGroupAction(const planning_scene_monitor::PlanningSceneMonitorConstPtr& psm) : 
    node_handle_("~"), planning_scene_monitor_(psm), trajectory_monitor_(psm->getStateMonitor(), 10.0),
    planning_pipeline_(psm->getPlanningScene()->getKinematicModel()), state_(IDLE)
  {
    bool allow_trajectory_execution = true;
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);
    
    if (allow_trajectory_execution)
      trajectory_execution_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getPlanningScene()->getKinematicModel()));
    planning_pipeline_.displayComputedMotionPlans(true);
    planning_pipeline_.checkSolutionPaths(true);

    trajectory_monitor_.setOnStateAddCallback(boost::bind(&MoveGroupAction::newMonitoredStateCallback, this, _1, _2));
    
    // start the action server
    action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, NODE_NAME, boost::bind(&MoveGroupAction::executeCallback, this, _1), false));
    action_server_->registerPreemptCallback(boost::bind(&MoveGroupAction::preemptCallback, this));
    action_server_->start();
    
    // start the service server
    plan_service_ = root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupAction::computePlan, this);
  }
  
  void status(void)
  {
    ROS_INFO_STREAM("MoveGroup action running using planning plugin " << planning_pipeline_.getPlannerPluginName());
  }
  
private:

  void preemptCallback(void)
  {
    preempt_requested_ = true;
  }
  
  void executeCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
  {   
    preempt_requested_ = false;
    moveit_msgs::MoveGroupResult action_res;
    moveit_msgs::GetMotionPlan::Request mreq;
    mreq.motion_plan_request = goal->request;
    if (mreq.motion_plan_request.group_name.empty())
    {
      ROS_WARN_STREAM("Must specify group in motion plan request");
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      action_server_->setAborted(action_res, "Must specify group in motion plan request");
      setState(IDLE, 0.0);
      return;    
    }
    
    // check to see if the desired constraints are already met
    for (std::size_t i = 0 ; i < mreq.motion_plan_request.goal_constraints.size() ; ++i)
      if (planning_scene_monitor_->getPlanningScene()->isStateConstrained(mreq.motion_plan_request.start_state,
                                                                          kinematic_constraints::mergeConstraints(mreq.motion_plan_request.goal_constraints[i],
                                                                                                                  mreq.motion_plan_request.path_constraints)))
      {
        
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        action_server_->setSucceeded(action_res, "Requested path and goal constraints are already met.");
        setState(IDLE, 0.0);
        return;
      }
    
    setState(PLANNING, 0.1);
    moveit_msgs::GetMotionPlan::Response mres;
    const planning_scene::PlanningSceneConstPtr &the_scene = 
      planning_scene::PlanningScene::isEmpty(goal->planning_scene_diff) ? planning_scene_monitor_->getPlanningScene() : planning_scene_monitor_->getPlanningScene()->diff(goal->planning_scene_diff);
    
    bool solved = planning_pipeline_.generatePlan(the_scene, mreq, mres);
    
    if (!solved)
    {
      if (trajectory_processing::isTrajectoryEmpty(mres.trajectory))
      {
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        action_server_->setAborted(action_res, "No motion plan found. No execution attempted.");
      }
      else
      {
	action_res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
	action_server_->setAborted(action_res, "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.");
      }
      setState(IDLE, 0.0);
      return;
    }

    action_res.trajectory_start = mres.trajectory_start;
    action_res.planned_trajectory = mres.trajectory;
    if (!goal->plan_only && !trajectory_execution_)
      ROS_WARN_STREAM("Move group asked for execution and was not configured to allow execution");
    
    if (goal->plan_only || !trajectory_execution_)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      action_server_->setSucceeded(action_res, "Solution was found and returned but not executed.");
      setState(IDLE, 0.0);
      return;
    }
    
    execution_complete_ = false;

    if (trajectory_execution_->push(mres.trajectory))
    {
      trajectory_processing::convertToKinematicStates(currently_executed_trajectory_states_, mres.trajectory_start, mres.trajectory, the_scene->getCurrentState(), the_scene->getTransforms());
      currently_executed_trajectory_index_ = 0;
      setState(MONITOR, trajectory_processing::getTrajectoryDuration(mres.trajectory));
      trajectory_monitor_.startTrajectoryMonitor();
     
      trajectory_execution_->execute(boost::bind(&MoveGroupAction::doneWithTrajectoryExecution, this, _1));
      
      // wait for path to be done, while checking that the path does not become invalid
      static const ros::WallDuration d(0.01);
      bool path_became_invalid = false;
      kinematic_constraints::KinematicConstraintSet path_constraints(the_scene->getKinematicModel(), the_scene->getTransforms());
      path_constraints.add(mreq.motion_plan_request.path_constraints);
      while (node_handle_.ok() && !execution_complete_ && !preempt_requested_ && !path_became_invalid)
      {
        d.sleep(); 
        for (std::size_t i = currently_executed_trajectory_index_ ; i < currently_executed_trajectory_states_.size() ; ++i)
          if (!the_scene->isStateValid(*currently_executed_trajectory_states_[i], path_constraints, false))
          {
            the_scene->isStateValid(*currently_executed_trajectory_states_[i], path_constraints, true);
            path_became_invalid = true;
            break;
          }
      }
      
      if (preempt_requested_)
      {
        ROS_INFO("Stopping execution due to preemt request");
        trajectory_execution_->stopExecution();
      }
      else
        if (path_became_invalid)
        {
          ROS_INFO("Stopping execution because the path to execute became invalid (probably the environment changed)");
          trajectory_execution_->stopExecution();
        }

      trajectory_monitor_.stopTrajectoryMonitor();
      currently_executed_trajectory_states_.clear();
      trajectory_monitor_.getTrajectory(action_res.executed_trajectory);

      if (trajectory_execution_->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      {
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        action_server_->setSucceeded(action_res, "Solution was found and executed.");
      } 
      else
      {
        if (path_became_invalid)
        {
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
          action_server_->setAborted(action_res, "Solution found but the environment changed during execution and the path was aborted");
        }
        else
        {
          if (trajectory_execution_->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
          else
            action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
          action_server_->setAborted(action_res, "Solution found but controller failed during execution");
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("Apparently trajectory initialization failed");
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
      action_server_->setAborted(action_res, "Solution found but could not initiate trajectory execution");
    }
    setState(IDLE, 0.0);
  }
  
  void doneWithTrajectoryExecution(const moveit_controller_manager::ExecutionStatus &status)
  {
    execution_complete_ = true;
  }
  
  void newMonitoredStateCallback(const planning_models::KinematicStateConstPtr &state, const ros::Time &stamp)
  {           
    // find the index where the distance to the current state starts increasing
    double dist = currently_executed_trajectory_states_[currently_executed_trajectory_index_]->distance(*state);
    for (std::size_t i = currently_executed_trajectory_index_ + 1 ; i < currently_executed_trajectory_states_.size() ; ++i)
    {
      double d = currently_executed_trajectory_states_[i]->distance(*state);
      if (d >= dist)
      {
        currently_executed_trajectory_index_ = i - 1;
        break;
      }
      else
        dist = d;
    }
    std::pair<int, int> expected = trajectory_execution_->getCurrentExpectedTrajectoryIndex();

    ROS_DEBUG("Controller seems to be at state %lu of %lu. Trajectory execution manager expects the index to be %d.", currently_executed_trajectory_index_ + 1, currently_executed_trajectory_states_.size(), expected.second);
  }
  
  void setState(MoveGroupState state, double duration)
  {
    state_ = state;
    switch (state_)
    {
    case IDLE:
      feedback_.state = "IDLE";
      feedback_.time_to_completion = ros::Duration(duration);
      break;
    case PLANNING:
      feedback_.state = "PLANNING";
      feedback_.time_to_completion = ros::Duration(duration);
      break;
    case MONITOR:
      feedback_.state = "MONITOR";
      feedback_.time_to_completion = ros::Duration(duration);
      break;
    }
    action_server_->publishFeedback(feedback_);
  }

  bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning service request...");
    return planning_pipeline_.generatePlan(planning_scene_monitor_->getPlanningScene(), req, res);
  }
   
  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorConstPtr planning_scene_monitor_;
  planning_scene_monitor::TrajectoryMonitor trajectory_monitor_;
  std::vector<planning_models::KinematicStatePtr> currently_executed_trajectory_states_;
  std::size_t currently_executed_trajectory_index_;
  
  planning_pipeline::PlanningPipeline planning_pipeline_;
  
  boost::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > action_server_;
  moveit_msgs::MoveGroupFeedback feedback_;
  
  ros::ServiceServer plan_service_;
  
  boost::scoped_ptr<trajectory_execution_manager::TrajectoryExecutionManager> trajectory_execution_;  
  bool preempt_requested_;
  bool execution_complete_;
  MoveGroupState state_;
};


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
    
    MoveGroupAction mga(planning_scene_monitor);
    mga.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
