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
#include <moveit_sensor_manager/moveit_sensor_manager.h>
#include <kinematic_constraints/utils.h>
#include <trajectory_processing/trajectory_tools.h>
#include <planning_models/conversions.h>
#include <pluginlib/class_loader.h>
#include <boost/algorithm/string/join.hpp>
#include <collision_detection/collision_tools.h>

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
  
  MoveGroupAction(const planning_scene_monitor::PlanningSceneMonitorPtr& psm) : 
    node_handle_("~"), planning_scene_monitor_(psm), trajectory_monitor_(psm->getStateMonitor(), 10.0),
    new_scene_update_(false), planning_pipeline_(psm->getPlanningScene()->getKinematicModel()), state_(IDLE)
  {
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution = true;
    node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);
    
    if (allow_trajectory_execution)
      trajectory_execution_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getPlanningScene()->getKinematicModel()));
    
    // we want to be notified when new information is available
    planning_scene_monitor_->setUpdateCallback(boost::bind(&MoveGroupAction::planningSceneUpdatedCallback, this, _1));
    

    // load the sensor manager plugin, if needed
    if (node_handle_.hasParam("moveit_sensor_manager"))
    {
      try
      {
        sensor_manager_loader_.reset(new pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager>("moveit_sensor_manager", "moveit_sensor_manager::MoveItSensorManager"));
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
        ROS_INFO_STREAM("MoveGroup action is aware of the following sensors: " << boost::algorithm::join(sensors, ", "));
      }
    }
    
    // configure the planning pipeline
    planning_pipeline_.displayComputedMotionPlans(true);
    planning_pipeline_.checkSolutionPaths(true);

    // when monitoring trajectories, every added state is passed to this callback
    trajectory_monitor_.setOnStateAddCallback(boost::bind(&MoveGroupAction::newMonitoredStateCallback, this, _1, _2));
    
    // start the action server
    action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, NODE_NAME, boost::bind(&MoveGroupAction::executeCallback, this, _1), false));
    action_server_->registerPreemptCallback(boost::bind(&MoveGroupAction::preemptCallback, this));
    action_server_->start();
    
    // publisher for cost sources
    cost_sources_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("display_cost_sources", 100, true);

    // start the service server
    plan_service_ = root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupAction::computePlanService, this);
  }
  
  void status(void)
  {
    ROS_INFO_STREAM("MoveGroup action running using planning plugin " << planning_pipeline_.getPlannerPluginName());
  }
  
private:
  
  struct LockScene
  {
    LockScene(const planning_scene_monitor::PlanningSceneMonitorPtr &monitor) : monitor_(monitor.get())
    {
      monitor_->lockScene();
    }
    ~LockScene(void)
    {   
      monitor_->unlockScene();
    }
    planning_scene_monitor::PlanningSceneMonitor *monitor_;
  };
  
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
    {
      LockScene lock(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while isStateConstrained() is called
      for (std::size_t i = 0 ; i < mreq.motion_plan_request.goal_constraints.size() ; ++i)
        if (planning_scene_monitor_->getPlanningScene()->isStateConstrained(mreq.motion_plan_request.start_state,
                                                                            kinematic_constraints::mergeConstraints(mreq.motion_plan_request.goal_constraints[i],
                                                                                                                    mreq.motion_plan_request.path_constraints)))
        {
          ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
	  action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          action_server_->setSucceeded(action_res, "Requested path and goal constraints are already met.");
          setState(IDLE, 0.0);
          return;
        }
    }
    
    // get the planning scene to be used
    setState(PLANNING, 0.2);
    new_scene_update_ = false;
    planning_scene::PlanningSceneConstPtr the_scene = planning_scene_monitor_->getPlanningScene();
    if (!planning_scene::PlanningScene::isEmpty(goal->planning_scene_diff))
    {
      LockScene lock(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
      the_scene = planning_scene_monitor_->getPlanningScene()->diff(goal->planning_scene_diff); 
      new_scene_update_ = false; // set this to false again, just in case an update came in in the meantime
    }

    // run the motion planner
    moveit_msgs::GetMotionPlan::Response mres;
    bool solved = computePlan(the_scene, mreq, mres);
    
    // abort if no plan was found
    if (!solved)
    {
      action_res.error_code = mres.error_code;
      if (trajectory_processing::isTrajectoryEmpty(mres.trajectory))
        action_server_->setAborted(action_res, "No motion plan found. No execution attempted.");
      else
        action_server_->setAborted(action_res, "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.");
      setState(IDLE, 0.0);
      return;
    }

    // fill in results
    action_res.trajectory_start = mres.trajectory_start;
    action_res.planned_trajectory = mres.trajectory;
    if (!goal->plan_only && !trajectory_execution_)
      ROS_WARN_STREAM("Move group asked for execution and was not configured to allow execution");
    
    // stop if no trajectory execution is needed or can't be done
    if (goal->plan_only || !trajectory_execution_)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      action_server_->setSucceeded(action_res, "Solution was found and returned but not executed.");
      setState(IDLE, 0.0);
      return;
    }
    
    // if we are allowed to look around, see if we have costs that are too high
    if (goal->look_around)
    {       
      LockScene lock(planning_scene_monitor_);
      // determine the sources of cost for this path
      trajectory_processing::convertToKinematicStates(currently_executed_trajectory_states_, mres.trajectory_start, mres.trajectory, the_scene->getCurrentState(), the_scene->getTransforms());
      collision_detection::CollisionRequest creq;
      creq.max_cost_sources = 100;
      creq.cost = true;   
      std::set<collision_detection::CostSource> cost_sources;
      for (std::size_t i = 0 ; i < currently_executed_trajectory_states_.size() ; ++i)
      {
        collision_detection::CollisionResult cres;
        the_scene->checkCollision(creq, cres, *currently_executed_trajectory_states_[i]);
        cost_sources.insert(cres.cost_sources.begin(), cres.cost_sources.end());
      }
      visualization_msgs::MarkerArray arr;
      collision_detection::getCostMarkers(arr, the_scene->getPlanningFrame(), cost_sources);
      cost_sources_publisher_.publish(arr);
      double cost = 0.0;
      for (std::set<collision_detection::CostSource>::const_iterator it = cost_sources.begin() ; it != cost_sources.end() ; ++it)
        cost += it->getVolume() * it->cost;
      ROS_INFO("The total cost of the trajectory is %lf", cost);
    }
    
    // try to execute the trajectory
    execution_complete_ = false;
    if (trajectory_execution_->push(mres.trajectory))
    {
      if (currently_executed_trajectory_states_.empty())
      {    
        LockScene lock(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while isStateValid() is called
        trajectory_processing::convertToKinematicStates(currently_executed_trajectory_states_, mres.trajectory_start, mres.trajectory, the_scene->getCurrentState(), the_scene->getTransforms());
      }
      currently_executed_trajectory_index_ = 0;
      setState(MONITOR, trajectory_processing::getTrajectoryDuration(mres.trajectory));
      trajectory_monitor_.startTrajectoryMonitor();

      // start a trajectory execution thread
      trajectory_execution_->execute(boost::bind(&MoveGroupAction::doneWithTrajectoryExecution, this, _1));
      
      // wait for path to be done, while checking that the path does not become invalid
      static const ros::WallDuration d(0.01);
      bool path_became_invalid = false;
      boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> path_constraints;
      {    
        LockScene lock(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while getTransforms() is called
        path_constraints.reset(new kinematic_constraints::KinematicConstraintSet(the_scene->getKinematicModel(), the_scene->getTransforms()));
        path_constraints->add(mreq.motion_plan_request.path_constraints);
      }
      
      while (node_handle_.ok() && !execution_complete_ && !preempt_requested_ && !path_became_invalid)
      {
        d.sleep();
        // check the path if there was an environment update in the meantime
        if (new_scene_update_)
        {
          LockScene lock(planning_scene_monitor_); // lock the scene so that it does not modify the world representation while isStateValid() is called
          new_scene_update_ = false;
          for (std::size_t i = currently_executed_trajectory_index_ ; i < currently_executed_trajectory_states_.size() ; ++i)
            if (!the_scene->isStateValid(*currently_executed_trajectory_states_[i], *path_constraints, false))
            {
              the_scene->isStateValid(*currently_executed_trajectory_states_[i], *path_constraints, true);
              path_became_invalid = true;
              break;
            }
        }
      }
      
      if (preempt_requested_)
      {
        ROS_INFO("Stopping execution due to preempt request");
        trajectory_execution_->stopExecution();
      }
      else
        if (path_became_invalid)
        {
          ROS_INFO("Stopping execution because the path to execute became invalid (probably the environment changed)");
          trajectory_execution_->stopExecution();
        }
        else
          if (!execution_complete_)
          {    
            ROS_WARN("Stopping execution due to unknown reason. Possibly the node is about to shut down.");
            trajectory_execution_->stopExecution();
          }
      
      // because of the way the trajectory monitor works, when this call completes, it is certain that no more calls
      // are made to the callback associated to the trajectory monitor
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
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
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
  
  void planningSceneUpdatedCallback(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
  {
    if (update_type & (planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY | planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS))
      new_scene_update_ = true;
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
  
  bool computePlan(const planning_scene::PlanningSceneConstPtr &scene, moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    bool solved = false;   
    LockScene lock(planning_scene_monitor_);
    
    try
    {
      solved = planning_pipeline_.generatePlan(scene, req, res);
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
  
  bool computePlanService(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
  {
    ROS_INFO("Received new planning service request...");
    return computePlan(planning_scene_monitor_->getPlanningScene(), req, res);
  }
  
  /// Given a set of locations to point sensors at, this function loops through the known sensors and attempts
  /// to point one at each point, in a best effort approach. If at least one sensor is succesfully pointed at a target,
  /// this function returns true
  bool lookAt(const std::vector<Eigen::Vector3d> &points)
  {
    bool result = true;
    if (!points.empty())
    {
      if (sensor_manager_)
      {
        std::vector<std::string> names;
        sensor_manager_->getSensorsList(names);
        std::vector<bool> used(false, names.size());
        
        std::size_t good = 0;
        for (std::size_t i = 0 ; i < points.size() && good < names.size() ; ++i)
        {
          geometry_msgs::PointStamped t;
          t.header.stamp = planning_scene_monitor_->getLastUpdateTime();
          t.header.frame_id = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
          t.point.x = points[i].x();
          t.point.y = points[i].y();
          t.point.z = points[i].z();
          for (std::size_t k = 0 ; k < names.size() ; ++k)
            if (!used[k])
            {
              moveit_msgs::RobotTrajectory traj;
              if (sensor_manager_->pointSensorTo(names[k], t, traj))
              {
                if (!trajectory_processing::isTrajectoryEmpty(traj) && trajectory_execution_)
                {
                  if (trajectory_execution_->push(traj) && trajectory_execution_->executeAndWait())
                  {
                    used[k] = true;
                    good++;
                  }
                }
                else
                {
                  used[k] = true;
                  good++;
                }
              }
            }
        }
        result = good > 0;
      }
      else
        result = false;
    }
    return result;
  }
  
  /// Attempt to point a sensor at this given point
  bool lookAt(const Eigen::Vector3d &point)
  {
    return lookAt(std::vector<Eigen::Vector3d>(1, point));
  }
  
  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene_monitor::TrajectoryMonitor trajectory_monitor_;
  std::vector<planning_models::KinematicStatePtr> currently_executed_trajectory_states_;
  std::size_t currently_executed_trajectory_index_;
  bool new_scene_update_;
  ros::Publisher cost_sources_publisher_;
  
  planning_pipeline::PlanningPipeline planning_pipeline_;
  
  boost::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > action_server_;
  moveit_msgs::MoveGroupFeedback feedback_;
  
  ros::ServiceServer plan_service_;

  boost::scoped_ptr<pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager> > sensor_manager_loader_;
  moveit_sensor_manager::MoveItSensorManagerPtr sensor_manager_;

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
