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
#include <planning_interface/planning_interface.h>
#include <planning_request_adapter/planning_request_adapter.h>

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <trajectory_execution_ros/trajectory_execution_monitor_ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/tokenizer.hpp>

#include <trajectory_processing/iterative_smoother.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";      // name of the robot description (a param name, so it can be changed externally)
static const std::string DISPLAY_PATH_PUB_TOPIC = "display_trajectory";

class MoveGroupAction
{
public:
  
  enum MoveGroupState
    {
      IDLE,
      PLANNING,
      MONITOR
    };
  
  MoveGroupAction(planning_scene_monitor::PlanningSceneMonitor &psm) : 
    nh_("~"), psm_(psm), state_(IDLE)
  {
    // load the group name
    if (nh_.getParam("group", group_name_))
      ROS_INFO("Starting move_group for group '%s'", group_name_.c_str());
    else
      ROS_FATAL("Group name not specified. Cannot start move_group");
    bool manage_controllers= false;
    nh_.param("manage_controllers", manage_controllers, true);
    
    trajectory_execution_.reset(new trajectory_execution_ros::TrajectoryExecutionMonitorRos(psm_.getPlanningScene()->getKinematicModel(),
                                                                                            manage_controllers));

    // load the planning plugin
    try
    {
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface", "planning_interface::Planner"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    
    nh_.getParam("planning_plugin", planning_plugin_name_);
    const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
    if (planning_plugin_name_.empty() && classes.size() == 1)
    {
      planning_plugin_name_ = classes[0];
      ROS_INFO("No 'planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.", planning_plugin_name_.c_str());
    }
    if (planning_plugin_name_.empty() && classes.size() > 1)
    {      
      planning_plugin_name_ = classes[0];   
      ROS_INFO("Multiple planning plugins available. You shuold specify the 'planning_plugin' parameter. Using '%s' for now.", planning_plugin_name_.c_str());
    }
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planning_plugin_name_));
      planner_instance_->init(psm_.getPlanningScene()->getKinematicModel());      
      ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      std::stringstream ss;
      for (std::size_t i = 0 ; i < classes.size() ; ++i)
        ss << classes[i] << " ";
      ROS_FATAL_STREAM("Exception while loading planner '" << planning_plugin_name_ << "': " << ex.what() << std::endl
                       << "Available plugins: " << ss.str());
    }

    // load the planner request adapters
    std::string adapters;
    if (nh_.getParam("request_adapters", adapters))
    { 
      try
      {
        adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>("planning_request_adapter", "planning_request_adapter::PlanningRequestAdapter"));
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
      }
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
      std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
      for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
      {
        planning_request_adapter::PlanningRequestAdapterConstPtr ad;
        try
        {
          ad.reset(adapter_plugin_loader_->createUnmanagedInstance(*beg));
        }
        catch (pluginlib::PluginlibException& ex)
        {
          ROS_ERROR_STREAM("Exception while planning adapter plugin '" << *beg << "': " << ex.what());
        }
        if (ad)
          ads.push_back(ad);
      }
      if (!ads.empty())
      {
        adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
        for (std::size_t i = 0 ; i < ads.size() ; ++i)
        {
          ROS_INFO_STREAM("Using planning request adapter '" << ads[i]->getDescription() << "'");
          adapter_chain_->addAdapter(ads[i]);
        }
      }
    }
    
    
    display_path_publisher_ = root_nh_.advertise<moveit_msgs::DisplayTrajectory>("move_" + group_name_ + "/" + DISPLAY_PATH_PUB_TOPIC, 1, true);

    // start the action server
    action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_nh_, "move_" + group_name_, false));
    action_server_->registerGoalCallback(boost::bind(&MoveGroupAction::goalCallback, this));
    action_server_->registerPreemptCallback(boost::bind(&MoveGroupAction::preemptCallback, this));
    action_server_->start();
  }
  
  void goalCallback(void)
  {
    if (service_goal_thread_)
    {
      terminate_service_thread_ = true;
      service_goal_thread_->join();
      service_goal_thread_.reset();
    }
    goal_ = action_server_->acceptNewGoal();
    if (!goal_)
    {
      ROS_ERROR("Something unexpected happened. No goal found in callback for goal...");
      return;
    }
    
    if (!goal_->request.group_name.empty() && goal_->request.group_name != group_name_)
    {
      moveit_msgs::MoveGroupResult res;
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      action_server_->setAborted(res, "Cannot accept requests for group '" + 
                                 goal_->request.group_name  + "' when the move_group action is loaded for group '" +
                                 group_name_ +  "'");
    }
    else {
      terminate_service_thread_ = false;
      service_goal_thread_.reset(new boost::thread(boost::bind(&MoveGroupAction::serviceGoalRequest, this)));
    }
  }
  
  void preemptCallback(void)
  {
    action_server_->setPreempted();
    terminate_service_thread_ = true;
  }

  void serviceGoalRequest(void)
  {
    setState(PLANNING);
    
    bool solved = false;
    moveit_msgs::GetMotionPlan::Request req;
    req.motion_plan_request = goal_->request;
    if (req.motion_plan_request.group_name.empty())
      req.motion_plan_request.group_name = group_name_;
    moveit_msgs::GetMotionPlan::Response res;

    const planning_scene::PlanningScenePtr &the_scene = 
      planning_scene::PlanningScene::isEmpty(goal_->planning_scene_diff) ? psm_.getPlanningScene() : planning_scene::PlanningScene::diff(psm_.getPlanningScene(), goal_->planning_scene_diff);

    try
    {
      if (adapter_chain_)
        solved = adapter_chain_->adaptAndPlan(planner_instance_, the_scene, req, res);
      else
        solved = planner_instance_->solve(the_scene, req, res);
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Exception caught: '%s'", ex.what());
    }
    catch(...)
    {
      ROS_ERROR("Unknown exception thrown by planner");
    }
    
    if (solved)
    {
      trajectory_msgs::JointTrajectory trajectory_out;
      smoother_.smooth(res.trajectory.joint_trajectory, trajectory_out, psm_.getGroupJointLimitsMap().at(group_name_));
      res.trajectory.joint_trajectory = trajectory_out;
      
      setState(MONITOR);
      execution_complete_ = false;
      
      // display the trajectory
      moveit_msgs::DisplayTrajectory disp;
      disp.model_id = psm_.getPlanningScene()->getKinematicModel()->getName();
      disp.trajectory_start = res.trajectory_start;
      disp.trajectory = res.trajectory;
      display_path_publisher_.publish(disp);      

      trajectory_execution::TrajectoryExecutionRequest ter;
      ter.group_name_ = group_name_;      
      ter.trajectory_ = res.trajectory.joint_trajectory; // \TODO This should take in a RobotTrajectory
      if (trajectory_execution_->executeTrajectory(ter, boost::bind(&MoveGroupAction::doneWithTrajectoryExecution, this, _1)))
      {
        ros::WallDuration d(0.01);
        while (nh_.ok() && !execution_complete_ && !terminate_service_thread_)
        {
          /// \TODO Check if the remainder of the path is still valid; If not, replan.
          /// We need a callback in the trajectory monitor for this
          d.sleep();
        }     
        moveit_msgs::MoveGroupResult res;
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        action_server_->setSucceeded(res, "Solution was found and executed.");
      }
      else
      {
        moveit_msgs::MoveGroupResult res;
        //        res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
        action_server_->setAborted(res, "Solution was found but the controller failed to execute it.");
      }
    }
    else
    {
      moveit_msgs::MoveGroupResult res;
      res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      action_server_->setAborted(res, "No motion plan found. No execution attempted.");
    }
    
    setState(IDLE);
  }

  bool doneWithTrajectoryExecution(trajectory_execution::TrajectoryExecutionDataVector data)
  {
    execution_complete_ = true;
    return true;
  }
  
  void setState(MoveGroupState state)
  {
    state_ = state;
    switch (state_)
    {
    case IDLE:
      feedback_.state = "IDLE";
      feedback_.time_to_completion = ros::Duration(0.0);
      break;
    case PLANNING:
      feedback_.state = "PLANNING";
      feedback_.time_to_completion = ros::Duration(0.0);
      break;
    case MONITOR:
      feedback_.state = "MONITOR";
      feedback_.time_to_completion = ros::Duration(0.0);
      break;
    }
    action_server_->publishFeedback(feedback_);
  }
  
  void status(void)
  {
    ROS_INFO("MoveGroup action for group '%s' running using planning plugin '%s'", group_name_.c_str(), planning_plugin_name_.c_str());
  }
  
private:
  
  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitor &psm_;

  std::string planning_plugin_name_;
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader_;
  planning_interface::PlannerPtr planner_instance_;
  std::string group_name_;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;
  boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;
  
  boost::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > action_server_;
  moveit_msgs::MoveGroupGoalConstPtr goal_;
  moveit_msgs::MoveGroupFeedback feedback_;

  boost::scoped_ptr<boost::thread> service_goal_thread_;
  
  boost::shared_ptr<trajectory_execution_ros::TrajectoryExecutionMonitorRos> trajectory_execution_;  
  bool terminate_service_thread_;
  bool execution_complete_;
  MoveGroupState state_;
  trajectory_processing::IterativeParabolicSmoother smoother_;
  
  ros::Publisher display_path_publisher_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group", ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  if (psm.getPlanningScene()->isConfigured())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();
    
    MoveGroupAction mga(psm);
    mga.status();
    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");
  
  return 0;
}
