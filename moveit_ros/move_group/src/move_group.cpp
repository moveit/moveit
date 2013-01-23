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
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/pick_place/pick_place.h>

namespace move_group
{

class MoveGroupServer
{
public:
  
  enum MoveGroupState
    {
      IDLE,
      PLANNING,
      MONITOR,
      LOOK
    };
  
  MoveGroupServer(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug) : 
    node_handle_("~"),
    planning_scene_monitor_(psm),
    allow_trajectory_execution_(true),
    move_state_(IDLE),
    pickup_state_(IDLE)
  { 
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel()));
    
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution_, true);
    
    if (allow_trajectory_execution_)
    {  
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getKinematicModel()));
      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
      plan_with_sensing_.reset(new plan_execution::PlanWithSensing(trajectory_execution_manager_));
      plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupServer::startLookCallback, this));
      if (debug)
        plan_with_sensing_->displayCostSources(true);
    }
    
    pick_place_.reset(new pick_place::PickPlace(planning_pipeline_));
    
    // configure the planning pipeline
    planning_pipeline_->displayComputedMotionPlans(true);
    planning_pipeline_->checkSolutionPaths(true);

    if (debug)
      planning_pipeline_->publishReceivedRequests(true);
    
    // start the service servers
    plan_service_ = root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupServer::computePlanService, this);
    execute_service_ = root_node_handle_.advertiseService(EXECUTE_SERVICE_NAME, &MoveGroupServer::executeTrajectoryService, this);
    query_service_ = root_node_handle_.advertiseService(QUERY_SERVICE_NAME, &MoveGroupServer::queryInterface, this);

    // start the move action server
    move_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, MOVE_ACTION,
                                                                                              boost::bind(&MoveGroupServer::executeMoveCallback, this, _1), false));
    move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupServer::preemptMoveCallback, this));
    move_action_server_->start();

    // start the pickup action server
    pickup_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(root_node_handle_, PICKUP_ACTION,
                                                                                             boost::bind(&MoveGroupServer::executePickupCallback, this, _1), false));
    pickup_action_server_->registerPreemptCallback(boost::bind(&MoveGroupServer::preemptPickupCallback, this));
    pickup_action_server_->start();
  }
  
  ~MoveGroupServer(void)
  {
    move_action_server_.reset();
    pickup_action_server_.reset();
    execute_service_.shutdown();
    plan_service_.shutdown();
    query_service_.shutdown();
    planning_scene_monitor_.reset();
  }
  
  void status(void)
  {
    const planning_interface::PlannerPtr &planner_interface = planning_pipeline_->getPlannerInterface();
    if (planner_interface)
      ROS_INFO_STREAM("MoveGroup running using planning plugin " << planning_pipeline_->getPlannerPluginName());
    else
      ROS_WARN_STREAM("MoveGroup running was unable to load " << planning_pipeline_->getPlannerPluginName());
  }
  
private:
  
  bool planUsingPlanningPipeline(const moveit_msgs::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan)
  {    
    setMoveState(PLANNING);

    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
    bool solved = false;
    moveit_msgs::MotionPlanResponse res;
    try
    {
      solved = planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    plan.trajectory_start_ = res.trajectory_start;
    plan.planned_trajectory_.resize(1);
    plan.planned_trajectory_[0] = res.trajectory;
    plan.planned_trajectory_descriptions_.resize(1);
    plan.planned_trajectory_descriptions_[0] = "plan";
    plan.planned_trajectory_states_.resize(1);
    plan.error_code_ = res.error_code;
    plan.planning_group_ = res.group_name;
    trajectory_processing::convertToKinematicStates(plan.planned_trajectory_states_[0], plan.trajectory_start_, plan.planned_trajectory_[0],
                                                    plan.planning_scene_->getCurrentState(), plan.planning_scene_->getTransforms());
    return solved;
  }
  
  void startExecutionCallback(void)
  {
    setMoveState(MONITOR);
  }

  void startLookCallback(void)
  {
    setMoveState(LOOK);
  }

  void executeMoveCallback_PlanOnly(const moveit_msgs::MoveGroupGoalConstPtr& goal, moveit_msgs::MoveGroupResult &action_res)
  {
    ROS_INFO("Planning request received for MoveGroup action. Forwarding to planning pipeline.");
    
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
    const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
      static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);
    moveit_msgs::MotionPlanResponse res;
    try
    {
      planning_pipeline_->generatePlan(the_scene, goal->request, res);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what()); 
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception"); 
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    action_res.trajectory_start = res.trajectory_start;
    action_res.planned_trajectory = res.trajectory;
    action_res.error_code = res.error_code;
  }

  moveit_msgs::MotionPlanRequest clearRequestStartState(const moveit_msgs::MotionPlanRequest &request) const
  {
    moveit_msgs::MotionPlanRequest r = request;
    r.start_state = moveit_msgs::RobotState();
    ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request");
    return r;
  }
  
  moveit_msgs::PlanningScene clearSceneRobotState(const moveit_msgs::PlanningScene &scene) const
  {
    moveit_msgs::PlanningScene r = scene;
    r.robot_state = moveit_msgs::RobotState();
    ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff");
    return r;
  }
  
  void executeMoveCallback_PlanAndExecute(const moveit_msgs::MoveGroupGoalConstPtr& goal, moveit_msgs::MoveGroupResult &action_res)
  {  
    ROS_INFO("Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.");

    if (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff))
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
      const kinematic_state::KinematicState &current_state = lscene->getCurrentState();
      
      // check to see if the desired constraints are already met
      for (std::size_t i = 0 ; i < goal->request.goal_constraints.size() ; ++i)
        if (lscene->isStateConstrained(current_state, kinematic_constraints::mergeConstraints(goal->request.goal_constraints[i],
                                                                                              goal->request.path_constraints)))
        {
          ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          return;
        }
    }
    
    plan_execution::PlanExecution::Options opt;

    const moveit_msgs::MotionPlanRequest &motion_plan_request = planning_scene::PlanningScene::isEmpty(goal->request.start_state) ?
      goal->request : clearRequestStartState(goal->request);
    const moveit_msgs::PlanningScene &planning_scene_diff = planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
      goal->planning_options.planning_scene_diff : clearSceneRobotState(goal->planning_options.planning_scene_diff);
    
    opt.replan_ = goal->planning_options.replan;
    opt.replan_attempts_ = goal->planning_options.replan_attempts;
    opt.before_execution_callback_ = boost::bind(&MoveGroupServer::startExecutionCallback, this);
    
    opt.plan_callback_ = boost::bind(&MoveGroupServer::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);
    if (goal->planning_options.look_around && plan_with_sensing_)
    {
      ROS_INFO("Using sensing in the planning step");
      // we pass 0.0 as the max cost so that defaults are used (as controlled with dynamic reconfigure)
      opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, plan_with_sensing_.get(), _1, opt.plan_callback_,
                                       goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
    }
    
    plan_execution::ExecutableMotionPlan plan;
    plan_execution_->planAndExecute(plan, planning_scene_diff, opt);  
    
    action_res.trajectory_start = plan.trajectory_start_;
    if (plan.planned_trajectory_.empty())
      action_res.planned_trajectory = moveit_msgs::RobotTrajectory();
    else
      action_res.planned_trajectory = plan.planned_trajectory_[0];
    action_res.executed_trajectory = plan.executed_trajectory_;
    action_res.error_code = plan.error_code_;
  }
  
  void executeMoveCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
  {
    setMoveState(PLANNING);
    planning_scene_monitor_->updateFrameTransforms();

    moveit_msgs::MoveGroupResult action_res;
    if (goal->planning_options.plan_only || !allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
      executeMoveCallback_PlanOnly(goal, action_res);
    }
    else
      executeMoveCallback_PlanAndExecute(goal, action_res);

    bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
    std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      move_action_server_->setSucceeded(action_res, response);
    else
    {
      if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
        move_action_server_->setPreempted(action_res, response);
      else 
        move_action_server_->setAborted(action_res, response);
    }
    
    setMoveState(IDLE);
  }

  void preemptMoveCallback(void)
  {
    plan_execution_->stop();
  }

  std::string getActionResultString(const moveit_msgs::MoveItErrorCodes &error_code, bool planned_trajectory_empty, bool plan_only)
  {
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      if (planned_trajectory_empty)
        return "Requested path and goal constraints are already met.";
      else
      {
        if (plan_only)
          return "Motion plan was computed succesfully.";
        else
          return "Solution was found and executed.";
      }
    }
    else
      if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
        return "Must specify group in motion plan request";
      else
        if (error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
        {
          if (planned_trajectory_empty)
            return "No motion plan found. No execution attempted.";
          else
            return "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.";
        }
        else
          if (error_code.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
            return "Motion plan was found but it seems to be too costly and looking around did not help.";
          else
            if (error_code.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
              return "Solution found but the environment changed during execution and the path was aborted";
            else
              if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED)
                return "Solution found but controller failed during execution";
              else
                if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
                  return "Timeout reached";
                else
                  if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
                    return "Preempted";
                  else
                    if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS)
                      return "Invalid goal constraints";
                    else
                      if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
                        return "Invalid group name";
                      else
                        if (error_code.val == moveit_msgs::MoveItErrorCodes::FAILURE)
                          return "Catastrophic failure";
    return "Unknown event";
  }
  
  std::string stateToStr(MoveGroupState state) const
  {
    switch (state)
    {
    case IDLE:
      return "IDLE";
    case PLANNING:
      return "PLANNING";
    case MONITOR:
      return "MONITOR";
    case LOOK:
      return "LOOK";
    default:
      return "UNKNOWN";
    }
  }
  
  void setMoveState(MoveGroupState state)
  {
    move_state_ = state;
    move_feedback_.state = stateToStr(state);
    move_action_server_->publishFeedback(move_feedback_);
  }
  
  void executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
  { 
    pick_place::PickPlanPtr plan; 
    try
    {
      planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
      plan = pick_place_->planPick(ps, *goal);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Pick&place threw an exception: %s", ex.what()); 
    }
    catch(...)
    {
      ROS_ERROR("Pick&place threw an exception");
    }

    if (plan)
    {
      const std::vector<pick_place::ManipulationPlanPtr> &success = plan->getSuccessfulManipulationPlans();
      if (success.empty())
      {  
        action_res.error_code = plan->getErrorCode();
      }
      else
      {
        const pick_place::ManipulationPlanPtr &result = success.back();
        action_res.trajectory_start = result->trajectory_start_;
        action_res.trajectory_stages = result->trajectories_;
        action_res.trajectory_descriptions = result->trajectory_descriptions_; 
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        pick_place_->displayPlan(result);
      }
    }
    else
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
  }
  
  bool planUsingPickPlace(const moveit_msgs::PickupGoal& goal, plan_execution::ExecutableMotionPlan &plan)
  {
    setPickupState(PLANNING);
    
    planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);
    
    pick_place::PickPlanPtr pick_plan;
    try
    {
      pick_plan = pick_place_->planPick(plan.planning_scene_, goal);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Pick&place threw an exception: %s", ex.what()); 
    }
    catch(...)
    {
      ROS_ERROR("Pick&place threw an exception");
    }
    
    if (pick_plan)
    {
      const std::vector<pick_place::ManipulationPlanPtr> &success = pick_plan->getSuccessfulManipulationPlans();
      if (success.empty())
      {
        plan.error_code_ = pick_plan->getErrorCode();
      }
      else
      {
        const pick_place::ManipulationPlanPtr &result = success.back();
        plan.trajectory_start_ = result->trajectory_start_;
        plan.planned_trajectory_ = result->trajectories_;
        plan.planned_trajectory_descriptions_ = result->trajectory_descriptions_; 
        plan.planning_group_ = result->planning_group_;
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS; 
        pick_place_->displayPlan(result);
      }
    }
    else
    {
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    
    plan.planned_trajectory_states_.resize(plan.planned_trajectory_.size());
    for (std::size_t i = 0 ; i < plan.planned_trajectory_.size() ; ++i) // trajectory_start_ state is only correct for the first path component
      trajectory_processing::convertToKinematicStates(plan.planned_trajectory_states_[i], plan.trajectory_start_, plan.planned_trajectory_[i],
                                                      plan.planning_scene_->getCurrentState(), plan.planning_scene_->getTransforms());
    
    return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  
  void executePickupCallback_PlanAndExecute(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
  {
    plan_execution::PlanExecution::Options opt;
    
    opt.replan_ = goal->planning_options.replan;
    opt.replan_attempts_ = goal->planning_options.replan_attempts;
    opt.before_execution_callback_ = boost::bind(&MoveGroupServer::startExecutionCallback, this);
    
    opt.plan_callback_ = boost::bind(&MoveGroupServer::planUsingPickPlace, this, boost::cref(*goal), _1);
    if (goal->planning_options.look_around && plan_with_sensing_)
      // we pass 0.0 as the max cost so that defaults are used (as controlled with dynamic reconfigure)
      opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, plan_with_sensing_.get(), _1, opt.plan_callback_,
                                       goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);

    plan_execution::ExecutableMotionPlan plan;
    plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);  

    action_res.trajectory_start = plan.trajectory_start_;
    action_res.trajectory_stages = plan.planned_trajectory_;
    action_res.trajectory_descriptions = plan.planned_trajectory_descriptions_;
    action_res.error_code = plan.error_code_; 
  }

  void executePickupCallback(const moveit_msgs::PickupGoalConstPtr& goal)
  {
    setPickupState(PLANNING);
    
    planning_scene_monitor_->updateFrameTransforms();

    moveit_msgs::PickupResult action_res;

    if (goal->planning_options.plan_only || !allow_trajectory_execution_)
    {
      if (!goal->planning_options.plan_only)
        ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
      executePickupCallback_PlanOnly(goal, action_res);
    }
    else
      executePickupCallback_PlanAndExecute(goal, action_res);
    
    bool planned_trajectory_empty = action_res.trajectory_stages.empty();
    std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      pickup_action_server_->setSucceeded(action_res, response);
    else
    {
      if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
        pickup_action_server_->setPreempted(action_res, response);
      else 
        pickup_action_server_->setAborted(action_res, response);
    }
    
    setPickupState(IDLE);
  }  

  void preemptPickupCallback(void)
  {
  }

  void setPickupState(MoveGroupState state)
  {  
    pickup_state_ = state;
    pickup_feedback_.state = stateToStr(state);
    pickup_action_server_->publishFeedback(pickup_feedback_);
  }
  
  bool computePlanService(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning service request...");
    planning_scene_monitor_->updateFrameTransforms();
    
    bool solved = false;   
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);

    try
    {
      solved = planning_pipeline_->generatePlan(ps, req.motion_plan_request, res.motion_plan_response);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Planning pipeline threw an exception: %s", ex.what()); 
      res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
      ROS_ERROR("Planning pipeline threw an exception"); 
      res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    return solved;
  }

  bool executeTrajectoryService(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res)
  {
    ROS_INFO("Received new trajectory execution service request...");
    if (!trajectory_execution_manager_)
    {
      ROS_ERROR("Cannot execute trajectory since ~allow_trajectory_execution was set to false");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
      return true;
    }
    
    trajectory_execution_manager_->clear();
    if (trajectory_execution_manager_->push(req.trajectory))
    {
      trajectory_execution_manager_->execute();
      if (req.wait_for_execution)
      {
        moveit_controller_manager::ExecutionStatus es = trajectory_execution_manager_->waitForExecution();
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
    const planning_interface::PlannerPtr &planner_interface = planning_pipeline_->getPlannerInterface();
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
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  plan_execution::PlanExecutionPtr plan_execution_;
  plan_execution::PlanWithSensingPtr plan_with_sensing_;
  pick_place::PickPlacePtr pick_place_;
  
  bool allow_trajectory_execution_;
  
  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > move_action_server_;
  moveit_msgs::MoveGroupFeedback move_feedback_;

  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction> > pickup_action_server_;
  moveit_msgs::PickupFeedback pickup_feedback_;

  ros::ServiceServer plan_service_;
  ros::ServiceServer execute_service_;
  ros::ServiceServer query_service_;
  
  MoveGroupState move_state_;
  MoveGroupState pickup_state_;
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, move_group::NODE_NAME);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(move_group::ROBOT_DESCRIPTION, tf));
  
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
    move_group::MoveGroupServer mgs(planning_scene_monitor, debug);
    mgs.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
