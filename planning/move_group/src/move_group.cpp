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
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>

#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";    // name of the robot description (a param name, so it can be changed externally)
static const std::string NODE_NAME = "move_group";                   // name of node
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path";    // name of the advertised service (within the ~ namespace)
static const std::string EXECUTE_SERVICE_NAME="execute_kinematic_path"; // name of the advertised service (within the ~ namespace)
static const std::string QUERY_SERVICE_NAME="query_planner_interface"; // name of the advertised query service

class MoveGroupAction
{
public:
  
  enum MoveGroupState
    {
      IDLE,
      PLANNING,
      MONITOR,
      LOOK
    };
  
  MoveGroupAction(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug) : 
    node_handle_("~"), planning_scene_monitor_(psm),
    allow_trajectory_execution_(true), state_(IDLE)
  {
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution_, true);
    
    plan_execution_.reset(new plan_execution::PlanExecution(psm, !allow_trajectory_execution_));
    
    // configure the planning pipeline
    plan_execution_->getPlanningPipeline().displayComputedMotionPlans(true);
    plan_execution_->getPlanningPipeline().checkSolutionPaths(true);

    if (debug)
    {
      plan_execution_->displayCostSources(true);
      plan_execution_->getPlanningPipeline().publishReceivedRequests(true);
    }
    
    // start the action server
    action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, NODE_NAME, boost::bind(&MoveGroupAction::executeCallback, this, _1), false));
    action_server_->registerPreemptCallback(boost::bind(&MoveGroupAction::preemptCallback, this));
    action_server_->start();
    
    // start the service servers
    plan_service_ = root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupAction::computePlanService, this);
    execute_service_ = root_node_handle_.advertiseService(EXECUTE_SERVICE_NAME, &MoveGroupAction::executeTrajectoryService, this);
    query_service_ = root_node_handle_.advertiseService(QUERY_SERVICE_NAME, &MoveGroupAction::queryInterface, this);
  }
  
  ~MoveGroupAction(void)
  {
    action_server_.reset();
    execute_service_.shutdown();
    plan_service_.shutdown();
    query_service_.shutdown();
    planning_scene_monitor_.reset();
  }
  
  void status(void)
  {
    const planning_interface::PlannerPtr &planner_interface = plan_execution_->getPlanningPipeline().getPlannerInterface();
    if (planner_interface)
      ROS_INFO_STREAM("MoveGroup action running using planning plugin " << plan_execution_->getPlanningPipeline().getPlannerPluginName());
    else
      ROS_WARN_STREAM("MoveGroup action running was unable to load " << plan_execution_->getPlanningPipeline().getPlannerPluginName());
  }
  
private:
  
  void preemptCallback(void)
  {
    plan_execution_->stop();
  }
  
  void startPlanningCallback(void)
  { 
    setState(PLANNING, 0.5);
  }

  void startExecutionCallback(void)
  {
    setState(MONITOR, 1.0);
  }

  void startLookCallback(void)
  {
    setState(LOOK, 1.0);
  }
  
  void executeCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
  {
    setState(PLANNING, 0.1);
    planning_scene_monitor_->updateFrameTransforms();

    if (goal->plan_only || !allow_trajectory_execution_)
    {
      if (!goal->plan_only)
        ROS_WARN("This instance of MoveGroup is now allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
      plan_execution_->planOnly(goal->request, goal->planning_scene_diff);
    }
    else
    {
      plan_execution::PlanExecution::Options opt;
      opt.look_around_ = goal->look_around;
      opt.replan_ = goal->replan;
      opt.replan_attempts_ = goal->replan_attempts;
      opt.look_attempts_ = goal->look_around_attempts;
      opt.beforePlanCallback_ = boost::bind(&MoveGroupAction::startPlanningCallback, this);
      opt.beforeExecutionCallback_ = boost::bind(&MoveGroupAction::startExecutionCallback, this);
      opt.beforeLookCallback_ = boost::bind(&MoveGroupAction::startLookCallback, this);
      plan_execution_->planAndExecute(goal->request, goal->planning_scene_diff, opt);  
    }
    
    const plan_execution::PlanExecution::Result &res = plan_execution_->getLastResult();
    moveit_msgs::MoveGroupResult action_res;
    action_res.trajectory_start = res.trajectory_start_;
    action_res.planned_trajectory = res.planned_trajectory_;
    action_res.executed_trajectory = res.executed_trajectory_;
    action_res.error_code = res.error_code_;
      
    
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      if (trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory))
        action_server_->setSucceeded(action_res, "Requested path and goal constraints are already met.");
      else
      {
        if (goal->plan_only)
          action_server_->setSucceeded(action_res, "Motion plan was computed succesfully.");
        else
          action_server_->setSucceeded(action_res, "Solution was found and executed.");
      }
    }
    else
      if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
        action_server_->setAborted(action_res, "Must specify group in motion plan request");
    else
      if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
      {
        if (trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory))
          action_server_->setAborted(action_res, "No motion plan found. No execution attempted.");
        else
          action_server_->setAborted(action_res, "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.");
      }
      else
        if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
          action_server_->setAborted(action_res, "Motion plan was found but it seems to be too costly and looking around did not help.");
        else
          if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
            action_server_->setAborted(action_res, "Solution found but the environment changed during execution and the path was aborted");
          else
            if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED)
              action_server_->setAborted(action_res, "Solution found but controller failed during execution");
            else
              if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
                action_server_->setAborted(action_res, "Timeout reached");
              else
                if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
                  action_server_->setPreempted(action_res, "Preempted");
                else
                  action_server_->setAborted(action_res, "Unknown event");
    setState(IDLE, 0.0);
  }
  
  void setState(MoveGroupState state, double duration)
  {
    state_ = state;
    switch (state_)
    {
    case IDLE:
      feedback_.state = "IDLE";
      break;
    case PLANNING:
      feedback_.state = "PLANNING";
      break;
    case MONITOR:
      feedback_.state = "MONITOR";
      break;
    case LOOK:
      feedback_.state = "LOOK";
      break;
    }      
    feedback_.time_to_completion = ros::Duration(duration);
    action_server_->publishFeedback(feedback_);
  }

  bool computePlanService(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning service request...");
    planning_scene_monitor_->updateFrameTransforms();
    
    bool solved = false;   
    planning_scene_monitor_->lockScene();
    
    try
    {
      solved = plan_execution_->getPlanningPipeline().generatePlan(planning_scene_monitor_->getPlanningScene(), req, res);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
    }
    planning_scene_monitor_->unlockScene();

    return solved;
  }

  bool executeTrajectoryService(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res)
  {
    ROS_INFO("Received new trajectory execution service request...");
    if (!plan_execution_->getTrajectoryExecutionManager())
    {
      ROS_ERROR("Cannot execute trajectory since ~allow_trajectory_execution was set to false");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
      return true;
    }
    
    plan_execution_->getTrajectoryExecutionManager()->clear();
    if (plan_execution_->getTrajectoryExecutionManager()->push(req.trajectory))
    {
      plan_execution_->getTrajectoryExecutionManager()->execute();
      if (req.wait_for_execution)
      {
        moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
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
        ROS_INFO_STREAM("Execution completed: " << es.asString());
      }
      else
      {
        ROS_INFO("Trajectory was successfully forwarded to the controller");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }
    else
    {    
      res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    }
    return true;
  }
  
  bool queryInterface(moveit_msgs::QueryPlannerInterfaces::Request &req, moveit_msgs::QueryPlannerInterfaces::Response &res)
  {    
    const planning_interface::PlannerPtr &planner_interface = plan_execution_->getPlanningPipeline().getPlannerInterface();
    if (planner_interface)
    {
      std::vector<std::string> algs;
      planner_interface->getPlanningAlgorithms(algs);
      moveit_msgs::PlannerInterfaceDescription pi_desc;
      pi_desc.name = planner_interface->getDescription();
      planner_interface->getPlanningAlgorithms(pi_desc.planner_ids);
      res.planner_interfaces.push_back(pi_desc);
    }
    return true;
  }

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  plan_execution::PlanExecutionPtr plan_execution_;
  bool allow_trajectory_execution_;
  
  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > action_server_;
  moveit_msgs::MoveGroupFeedback feedback_;
  
  ros::ServiceServer plan_service_;
  ros::ServiceServer execute_service_;
  ros::ServiceServer query_service_;
  
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
    
    bool debug = false;
    for (int i = 1 ; i < argc ; ++i)
      if (strncmp(argv[i], "--debug", 7) == 0)
      {
        debug = true;
        break;
      }
    MoveGroupAction mga(planning_scene_monitor, debug);
    mga.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
