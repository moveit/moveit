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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_ros_planning/TrajectoryExecutionDynamicReconfigureConfig.h>
#include <geometric_shapes/check_isometry.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/robot_state/conversions.h>

// Name of this class for logging
static const std::string LOGNAME = "trajectory_execution_manager";

namespace trajectory_execution_manager
{
const std::string TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC = "trajectory_execution_event";

static const ros::Duration DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE(1.0);
static const double DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN = 0.5;  // allow 0.5s more than the expected execution time
                                                                    // before triggering a trajectory cancel (applied
                                                                    // after scaling)
static const double DEFAULT_CONTROLLER_GOAL_DURATION_SCALING =
    1.1;  // allow the execution of a trajectory to take more time than expected (scaled by a value > 1)

using namespace moveit_ros_planning;

class TrajectoryExecutionManager::DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl(TrajectoryExecutionManager* owner)
    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle("~/trajectory_execution"))
  {
    dynamic_reconfigure_server_.setCallback(
        [this](const auto& config, uint32_t level) { dynamicReconfigureCallback(config, level); });
  }

private:
  void dynamicReconfigureCallback(const TrajectoryExecutionDynamicReconfigureConfig& config, uint32_t /*level*/)
  {
    owner_->enableExecutionDurationMonitoring(config.execution_duration_monitoring);
    owner_->setAllowedExecutionDurationScaling(config.allowed_execution_duration_scaling);
    owner_->setAllowedGoalDurationMargin(config.allowed_goal_duration_margin);
    owner_->setExecutionVelocityScaling(config.execution_velocity_scaling);
    owner_->setAllowedStartTolerance(config.allowed_start_tolerance);
    owner_->setWaitForTrajectoryCompletion(config.wait_for_trajectory_completion);
    owner_->setContinuousExecutionThreadRate(config.continuous_execution_thread_rate);
  }

  TrajectoryExecutionManager* owner_;
  dynamic_reconfigure::Server<TrajectoryExecutionDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

TrajectoryExecutionManager::TrajectoryExecutionManager(
    const moveit::core::RobotModelConstPtr& robot_model,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : robot_model_(robot_model)
  , csm_(planning_scene_monitor_->getStateMonitor())
  , planning_scene_monitor_(planning_scene_monitor)
  , node_handle_("~")
{
  if (!node_handle_.getParam("moveit_manage_controllers", manage_controllers_))
    manage_controllers_ = false;

  initialize();
}

TrajectoryExecutionManager::TrajectoryExecutionManager(
    const moveit::core::RobotModelConstPtr& robot_model,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, bool manage_controllers)
  : robot_model_(robot_model)
  , csm_(planning_scene_monitor_->getStateMonitor())
  , planning_scene_monitor_(planning_scene_monitor)
  , node_handle_("~")
  , manage_controllers_(manage_controllers)
{
  initialize();
}

TrajectoryExecutionManager::~TrajectoryExecutionManager()
{
  stopExecution(true);
  delete reconfigure_impl_;
}

void TrajectoryExecutionManager::initialize()
{
  reconfigure_impl_ = nullptr;
  verbose_ = false;
  execution_complete_ = true;
  stop_continuous_execution_ = false;
  current_context_ = -1;
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  run_continuous_execution_thread_ = true;
  execution_duration_monitoring_ = true;
  execution_velocity_scaling_ = 1.0;
  allowed_start_tolerance_ = 0.01;

  allowed_execution_duration_scaling_ = DEFAULT_CONTROLLER_GOAL_DURATION_SCALING;
  allowed_goal_duration_margin_ = DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN;

  // load controller-specific values for allowed_execution_duration_scaling and allowed_goal_duration_margin
  loadControllerParams();

  // load the controller manager plugin
  try
  {
    controller_manager_loader_ =
        std::make_unique<pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>>(
            "moveit_core", "moveit_controller_manager::MoveItControllerManager");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM_NAMED(LOGNAME, "Exception while creating controller manager plugin loader: " << ex.what());
    return;
  }

  if (controller_manager_loader_)
  {
    std::string controller;
    if (!node_handle_.getParam("moveit_controller_manager", controller))
    {
      const std::vector<std::string>& classes = controller_manager_loader_->getDeclaredClasses();
      if (classes.size() == 1)
      {
        controller = classes[0];
        ROS_WARN_NAMED(LOGNAME,
                       "Parameter '~moveit_controller_manager' is not specified but only one "
                       "matching plugin was found: '%s'. Using that one.",
                       controller.c_str());
      }
      else
        ROS_FATAL_NAMED(LOGNAME, "Parameter '~moveit_controller_manager' not specified. This is needed to "
                                 "identify the plugin to use for interacting with controllers. No paths can "
                                 "be executed.");
    }

    if (!controller.empty())
      try
      {
        controller_manager_ = controller_manager_loader_->createUniqueInstance(controller);
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_FATAL_STREAM_NAMED(LOGNAME,
                               "Exception while loading controller manager '" << controller << "': " << ex.what());
      }
  }

  // other configuration steps
  reloadControllerInformation();

  event_topic_subscriber_ =
      root_node_handle_.subscribe(EXECUTION_EVENT_TOPIC, 100, &TrajectoryExecutionManager::receiveEvent, this);

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  if (manage_controllers_)
    ROS_INFO_NAMED(LOGNAME, "Trajectory execution is managing controllers");
  else
    ROS_INFO_NAMED(LOGNAME, "Trajectory execution is not managing controllers");
}

void TrajectoryExecutionManager::enableExecutionDurationMonitoring(bool flag)
{
  execution_duration_monitoring_ = flag;
}

void TrajectoryExecutionManager::setAllowedExecutionDurationScaling(double scaling)
{
  allowed_execution_duration_scaling_ = scaling;
}

void TrajectoryExecutionManager::setAllowedGoalDurationMargin(double margin)
{
  allowed_goal_duration_margin_ = margin;
}

void TrajectoryExecutionManager::setExecutionVelocityScaling(double scaling)
{
  execution_velocity_scaling_ = scaling;
}

void TrajectoryExecutionManager::setAllowedStartTolerance(double tolerance)
{
  allowed_start_tolerance_ = tolerance;
}

void TrajectoryExecutionManager::setWaitForTrajectoryCompletion(bool flag)
{
  wait_for_trajectory_completion_ = flag;
}

void TrajectoryExecutionManager::setContinuousExecutionThreadRate(int rate)
{
  continuous_execution_thread_rate_ = rate;
}

bool TrajectoryExecutionManager::isManagingControllers() const
{
  return manage_controllers_;
}

const moveit_controller_manager::MoveItControllerManagerPtr& TrajectoryExecutionManager::getControllerManager() const
{
  return controller_manager_;
}

void TrajectoryExecutionManager::processEvent(const std::string& event)
{
  if (event == "stop")
    stopExecution(true);
  else
    ROS_WARN_STREAM_NAMED(LOGNAME, "Unknown event type: '" << event << "'");
}

void TrajectoryExecutionManager::receiveEvent(const std_msgs::StringConstPtr& event)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Received event '" << event->data << "'");
  processEvent(event->data);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::RobotTrajectory& trajectory, const std::string& controller)
{
  if (controller.empty())
    return push(trajectory, std::vector<std::string>());
  else
    return push(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::JointTrajectory& trajectory, const std::string& controller)
{
  if (controller.empty())
    return push(trajectory, std::vector<std::string>());
  else
    return push(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::JointTrajectory& trajectory,
                                      const std::vector<std::string>& controllers)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory = trajectory;
  return push(traj, controllers);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::RobotTrajectory& trajectory,
                                      const std::vector<std::string>& controllers)
{
  if (!execution_complete_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot push a new trajectory in blocking mode while another is being executed (use "
                             "pushAndExecuteSimultaneously instead)");
    return false;
  }

  std::shared_ptr<TrajectoryExecutionContext> context = std::make_shared<TrajectoryExecutionContext>();
  if (configure(*context, trajectory, controllers))
  {
    if (verbose_)
    {
      std::stringstream ss;
      ss << "Pushed trajectory for execution using controllers [ ";
      for (const std::string& controller : context->controllers_)
        ss << controller << " ";
      ss << "]:" << std::endl;
      for (const moveit_msgs::RobotTrajectory& trajectory_part : context->trajectory_parts_)
        ss << trajectory_part << std::endl;
      ROS_INFO_NAMED(LOGNAME, "%s", ss.str().c_str());
    }
    trajectories_.push_back(std::move(context));
    return true;
  }
  else
  {
    context.reset();
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  }

  return false;
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const moveit_msgs::RobotTrajectory& trajectory,
                                                              const std::string& controller, const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  if (controller.empty())
    return pushAndExecuteSimultaneously(trajectory, std::vector<std::string>(), expiration_time, callback);
  else
    return pushAndExecuteSimultaneously(trajectory, std::vector<std::string>(1, controller), expiration_time, callback);
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const trajectory_msgs::JointTrajectory& trajectory,
                                                              const std::string& controller, const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  if (controller.empty())
    return pushAndExecuteSimultaneously(trajectory, std::vector<std::string>(), expiration_time, callback);
  else
    return pushAndExecuteSimultaneously(trajectory, std::vector<std::string>(1, controller), expiration_time, callback);
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const sensor_msgs::JointState& state,
                                                              const std::string& controller, const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  if (controller.empty())
    return pushAndExecuteSimultaneously(state, std::vector<std::string>(), expiration_time, callback);
  else
    return pushAndExecuteSimultaneously(state, std::vector<std::string>(1, controller), expiration_time, callback);
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const trajectory_msgs::JointTrajectory& trajectory,
                                                              const std::vector<std::string>& controllers,
                                                              const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory = trajectory;
  return pushAndExecuteSimultaneously(traj, controllers, expiration_time, callback);
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const sensor_msgs::JointState& state,
                                                              const std::vector<std::string>& controllers,
                                                              const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory.header = state.header;
  traj.joint_trajectory.joint_names = state.name;
  traj.joint_trajectory.points.resize(1);
  traj.joint_trajectory.points[0].positions = state.position;
  traj.joint_trajectory.points[0].velocities = state.velocity;
  traj.joint_trajectory.points[0].effort = state.effort;
  traj.joint_trajectory.points[0].time_from_start = ros::Duration(0, 0);
  return pushAndExecuteSimultaneously(traj, controllers, expiration_time, callback);
}

bool TrajectoryExecutionManager::pushAndExecuteSimultaneously(const moveit_msgs::RobotTrajectory& trajectory,
                                                              const std::vector<std::string>& controllers,
                                                              const int& expiration_time,
                                                              const ExecutionCompleteCallback& callback)
{
  std::shared_ptr<TrajectoryExecutionContext> context = std::make_shared<TrajectoryExecutionContext>();
  if (configure(*context, trajectory, controllers, expiration_time))
  {
    context->execution_complete_callback = callback;
    {
      boost::mutex::scoped_lock slock(continuous_execution_thread_mutex_);
      continuous_execution_queue_.push_back(std::move(context));
      stop_continuous_execution_ = false;
      if (!continuous_execution_thread_)
        continuous_execution_thread_ = std::make_unique<boost::thread>([this] { continuousExecutionThread(); });
    }
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    continuous_execution_condition_.notify_all();
    return true;
  }
  else
  {
    context.reset();
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    return false;
  }
}

void TrajectoryExecutionManager::continuousExecutionThread()
{
  /*
  Implemention of simple scheduling for simultaneous execution of multiple trajectories
  main loop:
    1. Check if we have entries in the *backlog*
      a. If so,
        I. check that the handles in the current item are not necessary in previous items of the backlog
          (avoid altering the sequential order in which requests arrived per handle).
         If there are not, go to step II., else check the next item in the backlog
        II. check the first item is executable, step 3. to 7., if so remove backlog entry, else go to the next item in
  the backlog c. after checking the entire *backlog*, go to step 2.
    2. Pop new request
    --- validateAndExecuteContext start here ---
    3. Check its handles (controllers) and see if they are available
    4. Check that the necessary handles are not busy, otherwise push request into *backlog*
    5. Check that the new trajectories are not in collision with the active collisions, otherwise push request into
  *backlog*
    6. Check that the new trajectories start from the current pose of the robot, otherwise abort request
    7. If everything is okay, execute trajectory, store request as used_handles, active_contexts
    --- validateAndExecuteContext stop here ---
  */
  /*
  TODO (cambel):
  - Adapt the validation of the duration of a trajectory as done in ExecuteThread(), thus aborting trajectories with
  "Controller is taking longer than expected"
  */
  std::set<moveit_controller_manager::MoveItControllerHandlePtr> used_handles;
  // The list of trajectories currently being executed
  std::vector<std::shared_ptr<TrajectoryExecutionContext>> active_contexts;
  std::deque<std::pair<std::shared_ptr<TrajectoryExecutionContext>, ros::Time>> backlog;

  ros::Rate r(continuous_execution_thread_rate_);
  while (run_continuous_execution_thread_)
  {
    ROS_DEBUG_NAMED(LOGNAME, "===========Loop top-most entry================");
    // This waits for the lock to be released
    if (!stop_continuous_execution_)
    {
      if (continuous_execution_queue_.empty() &&
          !active_contexts.empty())  // While trajectories are still being executed, check their response.
                                     // Instead of doing this, we could add a callback in the controller_manager,
                                     // but that seems like a bigger change.
      {
        ROS_DEBUG_NAMED(LOGNAME, "Updating list in top-most loop");
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "active_contexts size: " << active_contexts.size());
        updateActiveHandlesAndContexts(used_handles, active_contexts);
        // Waiting like this instead of waitForExecution so the queue keeps being checked for new entries
        r.sleep();
      }
      boost::unique_lock<boost::mutex> ulock(continuous_execution_thread_mutex_);
      while (continuous_execution_queue_.empty() && active_contexts.empty() && backlog.empty() &&
             run_continuous_execution_thread_ && !stop_continuous_execution_)
        continuous_execution_condition_.wait(ulock);
    }

    // If stop-flag is set, break out
    if (stop_continuous_execution_ || !run_continuous_execution_thread_)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Stop!. stop_continuous_execution: " << stop_continuous_execution_
                                                                           << " run_continuous_execution_thread_: "
                                                                           << run_continuous_execution_thread_);
      // Cancel on going executions
      for (const moveit_controller_manager::MoveItControllerHandlePtr& used_handle : used_handles)
        if (used_handle->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::RUNNING)
          used_handle->cancelExecution();
      // Clear map and used handles set
      active_contexts.clear();
      used_handles.clear();
      backlog.clear();
      while (!continuous_execution_queue_.empty())
      {
        std::shared_ptr<TrajectoryExecutionContext> context = continuous_execution_queue_.front();
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Calling completed callback to abort");
        if (!context->execution_complete_callback.empty())
          context->execution_complete_callback(moveit_controller_manager::ExecutionStatus::ABORTED);
        continuous_execution_queue_.pop_front();
        context.reset();
      }
      stop_continuous_execution_ = false;
      continue;
    }

    while (!continuous_execution_queue_.empty() || !backlog.empty())
    {
      ROS_DEBUG_NAMED(LOGNAME, "===========Loop2 entry================");

      ROS_DEBUG_NAMED(LOGNAME, "Start checking backlog");
      // Check all backlog entries for trajectories that can now be executed
      for (auto it = backlog.begin(); it != backlog.end();)
      {
        std::shared_ptr<TrajectoryExecutionContext> current_context = it->first;
        ros::Time& created_at = it->second;

        const auto& time_from_start =
            current_context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start;

        // Remove backlog items that have expired (to avoid deadlocks)
        if (created_at + ros::Duration(current_context->expiration_time) < ros::Time::now())
        {
          ROS_WARN_STREAM_NAMED(
              LOGNAME, "Backlog item with duration "
                           << time_from_start
                           << " has expired (older than 1 minute). Assuming malfunction, removing from backlog.");
          if (!current_context->execution_complete_callback.empty())
            current_context->execution_complete_callback(moveit_controller_manager::ExecutionStatus::ABORTED);
          it = backlog.erase(it);
          continue;
        }

        // Validate that the handles used in this context are not already in earlier (= higher priority) backlogged trajectories
        bool controllers_not_used_earlier_in_backlog = true;
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Backlog evaluation of item: " << time_from_start);
        for (auto it2 = backlog.begin(); it2 != it; ++it2)
        {
          std::shared_ptr<TrajectoryExecutionContext> priority_context =
              it2->first;  // Previous context in the backlog (earlier ones have priority)
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "Backlog comparing item with duration: " << time_from_start);
          ROS_DEBUG_STREAM_NAMED(
              LOGNAME, "vs item with duration: "
                           << priority_context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start);
          if (hasCommonHandles(*current_context, *priority_context))
          {
            controllers_not_used_earlier_in_backlog = false;
            ROS_DEBUG_NAMED(LOGNAME, "Backlog item has handles blocked by previous items");
            break;
          }
        }
        if (controllers_not_used_earlier_in_backlog)
        {
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "Backlog item with duration "
                                              << time_from_start << " will be checked and pushed to controller.");
          if (validateAndExecuteContext(*current_context, used_handles, active_contexts))
          {
            ROS_DEBUG_STREAM_NAMED(LOGNAME,
                                   "Backlog item with duration " << time_from_start << " has been executed correctly.");
            it = backlog.erase(it);
          }
          else if (it == backlog.begin() && active_contexts.empty())
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Trajectory is in a deadlock, aborting");
            // Since there is not active trajectory being executed but this Top priority backlog-trajectory is not
            // executable, abort it.
            if (!current_context->execution_complete_callback.empty())
              current_context->execution_complete_callback(moveit_controller_manager::ExecutionStatus::ABORTED);
            it = backlog.erase(it);
          }
          else
          {
            ROS_DEBUG_STREAM_NAMED(LOGNAME,
                                   "Backlog item with duration " << time_from_start << " is still not executable");
            it++;
          }
        }
        else
          it++;
      }
      r.sleep();  // Small delay to not process a pending trajectory over and over when it is temporarily blocked
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Done checking backlog, size: " << backlog.size());

      // Get next trajectory context from queue
      std::shared_ptr<TrajectoryExecutionContext> context = nullptr;
      {
        boost::mutex::scoped_lock slock(continuous_execution_thread_mutex_);
        if (continuous_execution_queue_.empty())
          break;
        context = std::move(continuous_execution_queue_.front());
        continuous_execution_queue_.pop_front();
        if (continuous_execution_queue_.empty())
          continuous_execution_condition_.notify_all();
      }

      ROS_DEBUG_NAMED(LOGNAME, "==========");
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Popped element with duration "
                                          << context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start
                                          << " from queue. Remaining length: " << continuous_execution_queue_.size());

      // First make sure desired controllers are active
      if (!areControllersActive(context->controllers_))
      {
        ROS_ERROR_NAMED(LOGNAME, "Not all needed controllers are active. Cannot push and execute. You can try "
                                 "calling ensureActiveControllers() before pushAndExecuteSimultaneously()");
        ROS_ERROR_NAMED(LOGNAME, "Not all needed controllers are active. Cannot push and execute. You can try "
                                 "calling ensureActiveControllers() before pushAndExecuteSimultaneously()");
        last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
        ROS_INFO_NAMED(LOGNAME, "Calling completed callback");
        if (!context->execution_complete_callback.empty())
          context->execution_complete_callback(moveit_controller_manager::ExecutionStatus::ABORTED);
        context.reset();
        continue;
      }

      // Check that this context's controller handles are not used in the backlog. Otherwise, add to backlog (because
      // trajectories need to be executed in order)
      bool controllers_not_used_in_backlog = true;
      for (auto backlog_context : backlog)
        if (hasCommonHandles(*backlog_context.first, *context))
        {
          ROS_DEBUG_STREAM_NAMED(LOGNAME,
                                 "Request with duration "
                                     << context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start);
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "has handles blocked by backlog items. push_back to backlog");
          backlog.push_back(
              std::pair<std::shared_ptr<TrajectoryExecutionContext>, ros::Time>(context, ros::Time::now()));
          controllers_not_used_in_backlog = false;
          break;
        }

      if (controllers_not_used_in_backlog && !validateAndExecuteContext(*context, used_handles, active_contexts))
      {
        ROS_DEBUG_STREAM_NAMED(
            LOGNAME, "Request: " << context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start
                                 << " not executable, pushing it into backlog");
        backlog.push_back(std::pair<std::shared_ptr<TrajectoryExecutionContext>, ros::Time>(context, ros::Time::now()));
      }
    }
  }
}

void TrajectoryExecutionManager::reloadControllerInformation()
{
  known_controllers_.clear();
  if (controller_manager_)
  {
    std::vector<std::string> names;
    controller_manager_->getControllersList(names);
    for (const std::string& name : names)
    {
      std::vector<std::string> joints;
      controller_manager_->getControllerJoints(name, joints);
      ControllerInformation ci;
      ci.name_ = name;
      ci.joints_.insert(joints.begin(), joints.end());
      known_controllers_[ci.name_] = ci;
    }

    for (std::map<std::string, ControllerInformation>::iterator it = known_controllers_.begin();
         it != known_controllers_.end(); ++it)
      for (std::map<std::string, ControllerInformation>::iterator jt = known_controllers_.begin();
           jt != known_controllers_.end(); ++jt)
        if (it != jt)
        {
          std::vector<std::string> intersect;
          std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), jt->second.joints_.begin(),
                                jt->second.joints_.end(), std::back_inserter(intersect));
          if (!intersect.empty())
          {
            it->second.overlapping_controllers_.insert(jt->first);
            jt->second.overlapping_controllers_.insert(it->first);
          }
        }
  }
}

void TrajectoryExecutionManager::updateControllerState(const std::string& controller, const ros::Duration& age)
{
  std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controller);
  if (it != known_controllers_.end())
    updateControllerState(it->second, age);
  else
    ROS_ERROR_NAMED(LOGNAME, "Controller '%s' is not known.", controller.c_str());
}

void TrajectoryExecutionManager::updateControllerState(ControllerInformation& ci, const ros::Duration& age)
{
  if (ros::Time::now() - ci.last_update_ >= age)
  {
    if (controller_manager_)
    {
      if (verbose_)
        ROS_INFO_NAMED(LOGNAME, "Updating information for controller '%s'.", ci.name_.c_str());
      ci.state_ = controller_manager_->getControllerState(ci.name_);
      ci.last_update_ = ros::Time::now();
    }
  }
  else if (verbose_)
    ROS_INFO_NAMED(LOGNAME, "Information for controller '%s' is assumed to be up to date.", ci.name_.c_str());
}

void TrajectoryExecutionManager::updateControllersState(const ros::Duration& age)
{
  for (std::pair<const std::string, ControllerInformation>& known_controller : known_controllers_)
    updateControllerState(known_controller.second, age);
}

bool TrajectoryExecutionManager::checkControllerCombination(std::vector<std::string>& selected,
                                                            const std::set<std::string>& actuated_joints)
{
  std::set<std::string> combined_joints;
  for (const std::string& controller : selected)
  {
    const ControllerInformation& ci = known_controllers_[controller];
    combined_joints.insert(ci.joints_.begin(), ci.joints_.end());
  }

  if (verbose_)
  {
    std::stringstream ss, saj, sac;
    for (const std::string& controller : selected)
      ss << controller << " ";
    for (const std::string& actuated_joint : actuated_joints)
      saj << actuated_joint << " ";
    for (const std::string& combined_joint : combined_joints)
      sac << combined_joint << " ";
    ROS_INFO_NAMED(LOGNAME, "Checking if controllers [ %s] operating on joints [ %s] cover joints [ %s]",
                   ss.str().c_str(), sac.str().c_str(), saj.str().c_str());
  }

  return std::includes(combined_joints.begin(), combined_joints.end(), actuated_joints.begin(), actuated_joints.end());
}

void TrajectoryExecutionManager::generateControllerCombination(std::size_t start_index, std::size_t controller_count,
                                                               const std::vector<std::string>& available_controllers,
                                                               std::vector<std::string>& selected_controllers,
                                                               std::vector<std::vector<std::string>>& selected_options,
                                                               const std::set<std::string>& actuated_joints)
{
  if (selected_controllers.size() == controller_count)
  {
    if (checkControllerCombination(selected_controllers, actuated_joints))
      selected_options.push_back(selected_controllers);
    return;
  }

  for (std::size_t i = start_index; i < available_controllers.size(); ++i)
  {
    bool overlap = false;
    const ControllerInformation& ci = known_controllers_[available_controllers[i]];
    for (std::size_t j = 0; j < selected_controllers.size() && !overlap; ++j)
    {
      if (ci.overlapping_controllers_.find(selected_controllers[j]) != ci.overlapping_controllers_.end())
        overlap = true;
    }
    if (overlap)
      continue;
    selected_controllers.push_back(available_controllers[i]);
    generateControllerCombination(i + 1, controller_count, available_controllers, selected_controllers,
                                  selected_options, actuated_joints);
    selected_controllers.pop_back();
  }
}

namespace
{
struct OrderPotentialControllerCombination
{
  bool operator()(const std::size_t a, const std::size_t b) const
  {
    // preference is given to controllers marked as default
    if (nrdefault[a] > nrdefault[b])
      return true;
    if (nrdefault[a] < nrdefault[b])
      return false;

    // and then to ones that operate on fewer joints
    if (nrjoints[a] < nrjoints[b])
      return true;
    if (nrjoints[a] > nrjoints[b])
      return false;

    // and then to active ones
    if (nractive[a] < nractive[b])
      return true;
    if (nractive[a] > nractive[b])
      return false;

    return false;
  }

  std::vector<std::vector<std::string>> selected_options;
  std::vector<std::size_t> nrdefault;
  std::vector<std::size_t> nrjoints;
  std::vector<std::size_t> nractive;
};
}  // namespace

bool TrajectoryExecutionManager::findControllers(const std::set<std::string>& actuated_joints,
                                                 std::size_t controller_count,
                                                 const std::vector<std::string>& available_controllers,
                                                 std::vector<std::string>& selected_controllers)
{
  // generate all combinations of controller_count controllers that operate on disjoint sets of joints
  std::vector<std::string> work_area;
  OrderPotentialControllerCombination order;
  std::vector<std::vector<std::string>>& selected_options = order.selected_options;
  generateControllerCombination(0, controller_count, available_controllers, work_area, selected_options,
                                actuated_joints);

  if (verbose_)
  {
    std::stringstream saj;
    std::stringstream sac;
    for (const std::string& available_controller : available_controllers)
      sac << available_controller << " ";
    for (const std::string& actuated_joint : actuated_joints)
      saj << actuated_joint << " ";
    ROS_INFO_NAMED(LOGNAME, "Looking for %zu controllers among [ %s] that cover joints [ %s]. Found %zd options.",
                   controller_count, sac.str().c_str(), saj.str().c_str(), selected_options.size());
  }

  // if none was found, this is a problem
  if (selected_options.empty())
    return false;

  // if only one was found, return it
  if (selected_options.size() == 1)
  {
    selected_controllers.swap(selected_options[0]);
    return true;
  }

  // if more options were found, evaluate them all and return the best one

  // count how many default controllers are used in each reported option, and how many joints are actuated in total by
  // the selected controllers,
  // to use that in the ranking of the options
  order.nrdefault.resize(selected_options.size(), 0);
  order.nrjoints.resize(selected_options.size(), 0);
  order.nractive.resize(selected_options.size(), 0);
  for (std::size_t i = 0; i < selected_options.size(); ++i)
  {
    for (std::size_t k = 0; k < selected_options[i].size(); ++k)
    {
      updateControllerState(selected_options[i][k], DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
      const ControllerInformation& ci = known_controllers_[selected_options[i][k]];

      if (ci.state_.default_)
        order.nrdefault[i]++;
      if (ci.state_.active_)
        order.nractive[i]++;
      order.nrjoints[i] += ci.joints_.size();
    }
  }

  // define a bijection to compute the raking of the found options
  std::vector<std::size_t> bijection(selected_options.size(), 0);
  for (std::size_t i = 0; i < selected_options.size(); ++i)
    bijection[i] = i;

  // sort the options
  std::sort(bijection.begin(), bijection.end(), order);

  // depending on whether we are allowed to load & unload controllers,
  // we have different preference on deciding between options
  if (!manage_controllers_)
  {
    // if we can't load different options at will, just choose one that is already loaded
    for (std::size_t i = 0; i < selected_options.size(); ++i)
      if (areControllersActive(selected_options[bijection[i]]))
      {
        selected_controllers.swap(selected_options[bijection[i]]);
        return true;
      }
  }

  // otherwise, just use the first valid option
  selected_controllers.swap(selected_options[bijection[0]]);
  return true;
}

bool TrajectoryExecutionManager::isControllerActive(const std::string& controller)
{
  return areControllersActive(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::areControllersActive(const std::vector<std::string>& controllers)
{
  for (const std::string& controller : controllers)
  {
    updateControllerState(controller, DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controller);
    if (it == known_controllers_.end() || !it->second.state_.active_)
      return false;
  }
  return true;
}

bool TrajectoryExecutionManager::selectControllers(const std::set<std::string>& actuated_joints,
                                                   const std::vector<std::string>& available_controllers,
                                                   std::vector<std::string>& selected_controllers)
{
  for (std::size_t i = 1; i <= available_controllers.size(); ++i)
    if (findControllers(actuated_joints, i, available_controllers, selected_controllers))
    {
      // if we are not managing controllers, prefer to use active controllers even if there are more of them
      if (!manage_controllers_ && !areControllersActive(selected_controllers))
      {
        std::vector<std::string> other_option;
        for (std::size_t j = i + 1; j <= available_controllers.size(); ++j)
          if (findControllers(actuated_joints, j, available_controllers, other_option))
          {
            if (areControllersActive(other_option))
            {
              selected_controllers = other_option;
              break;
            }
          }
      }
      return true;
    }
  return false;
}

bool TrajectoryExecutionManager::distributeTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                                      const std::vector<std::string>& controllers,
                                                      std::vector<moveit_msgs::RobotTrajectory>& parts)
{
  parts.clear();
  parts.resize(controllers.size());

  std::set<std::string> actuated_joints_mdof;
  actuated_joints_mdof.insert(trajectory.multi_dof_joint_trajectory.joint_names.begin(),
                              trajectory.multi_dof_joint_trajectory.joint_names.end());
  std::set<std::string> actuated_joints_single;
  for (const std::string& joint_name : trajectory.joint_trajectory.joint_names)
  {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_name);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != nullptr || jm->getType() == moveit::core::JointModel::FIXED)
        continue;
      actuated_joints_single.insert(jm->getName());
    }
  }

  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controllers[i]);
    if (it == known_controllers_.end())
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Controller " << controllers[i] << " not found.");
      return false;
    }
    std::vector<std::string> intersect_mdof;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), actuated_joints_mdof.begin(),
                          actuated_joints_mdof.end(), std::back_inserter(intersect_mdof));
    std::vector<std::string> intersect_single;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(), actuated_joints_single.begin(),
                          actuated_joints_single.end(), std::back_inserter(intersect_single));
    if (intersect_mdof.empty() && intersect_single.empty())
      ROS_WARN_STREAM_NAMED(LOGNAME, "No joints to be distributed for controller " << controllers[i]);
    {
      if (!intersect_mdof.empty())
      {
        std::vector<std::string>& jnames = parts[i].multi_dof_joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_mdof.begin(), intersect_mdof.end());
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0; j < trajectory.multi_dof_joint_trajectory.joint_names.size(); ++j)
          index[trajectory.multi_dof_joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0; j < jnames.size(); ++j)
          bijection[j] = index[jnames[j]];

        parts[i].multi_dof_joint_trajectory.points.resize(trajectory.multi_dof_joint_trajectory.points.size());
        for (std::size_t j = 0; j < trajectory.multi_dof_joint_trajectory.points.size(); ++j)
        {
          parts[i].multi_dof_joint_trajectory.points[j].time_from_start =
              trajectory.multi_dof_joint_trajectory.points[j].time_from_start;
          parts[i].multi_dof_joint_trajectory.points[j].transforms.resize(bijection.size());
          for (std::size_t k = 0; k < bijection.size(); ++k)
          {
            parts[i].multi_dof_joint_trajectory.points[j].transforms[k] =
                trajectory.multi_dof_joint_trajectory.points[j].transforms[bijection[k]];

            if (!trajectory.multi_dof_joint_trajectory.points[j].velocities.empty())
            {
              parts[i].multi_dof_joint_trajectory.points[j].velocities.resize(bijection.size());

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.x =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.x * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.y =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.y * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].linear.z =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].linear.z * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.x =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.x * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.y =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.y * execution_velocity_scaling_;

              parts[i].multi_dof_joint_trajectory.points[j].velocities[0].angular.z =
                  trajectory.multi_dof_joint_trajectory.points[j].velocities[0].angular.z * execution_velocity_scaling_;
            }
          }
        }
      }
      if (!intersect_single.empty())
      {
        std::vector<std::string>& jnames = parts[i].joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_single.begin(), intersect_single.end());
        parts[i].joint_trajectory.header = trajectory.joint_trajectory.header;
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j)
          index[trajectory.joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0; j < jnames.size(); ++j)
          bijection[j] = index[jnames[j]];
        parts[i].joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        for (std::size_t j = 0; j < trajectory.joint_trajectory.points.size(); ++j)
        {
          parts[i].joint_trajectory.points[j].time_from_start = trajectory.joint_trajectory.points[j].time_from_start;
          if (!trajectory.joint_trajectory.points[j].positions.empty())
          {
            parts[i].joint_trajectory.points[j].positions.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
              parts[i].joint_trajectory.points[j].positions[k] =
                  trajectory.joint_trajectory.points[j].positions[bijection[k]];
          }
          if (!trajectory.joint_trajectory.points[j].velocities.empty())
          {
            parts[i].joint_trajectory.points[j].velocities.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
              parts[i].joint_trajectory.points[j].velocities[k] =
                  trajectory.joint_trajectory.points[j].velocities[bijection[k]] * execution_velocity_scaling_;
          }
          if (!trajectory.joint_trajectory.points[j].accelerations.empty())
          {
            parts[i].joint_trajectory.points[j].accelerations.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
              parts[i].joint_trajectory.points[j].accelerations[k] =
                  trajectory.joint_trajectory.points[j].accelerations[bijection[k]];
          }
          if (!trajectory.joint_trajectory.points[j].effort.empty())
          {
            parts[i].joint_trajectory.points[j].effort.resize(bijection.size());
            for (std::size_t k = 0; k < bijection.size(); ++k)
              parts[i].joint_trajectory.points[j].effort[k] =
                  trajectory.joint_trajectory.points[j].effort[bijection[k]];
          }
        }
      }
    }
  }
  return true;
}

bool TrajectoryExecutionManager::validate(const TrajectoryExecutionContext& context) const
{
  if (allowed_start_tolerance_ == 0)  // skip validation on this magic number
    return true;

  ROS_DEBUG_NAMED(LOGNAME, "Validating trajectory with allowed_start_tolerance %g", allowed_start_tolerance_);

  moveit::core::RobotStatePtr current_state;
  if (!csm_->waitForCurrentState(ros::Time::now()) || !(current_state = csm_->getCurrentState()))
  {
    ROS_WARN_NAMED(LOGNAME, "Failed to validate trajectory: couldn't receive full current joint state within 1s");
    return false;
  }

  for (const auto& trajectory : context.trajectory_parts_)
  {
    if (!trajectory.joint_trajectory.points.empty())
    {
      // Check single-dof trajectory
      const std::vector<double>& positions = trajectory.joint_trajectory.points.front().positions;
      const std::vector<std::string>& joint_names = trajectory.joint_trajectory.joint_names;
      if (positions.size() != joint_names.size())
      {
        ROS_ERROR_NAMED(LOGNAME, "Wrong trajectory: #joints: %zu != #positions: %zu", joint_names.size(),
                        positions.size());
        return false;
      }

      for (std::size_t i = 0, end = joint_names.size(); i < end; ++i)
      {
        const moveit::core::JointModel* jm = current_state->getJointModel(joint_names[i]);
        if (!jm)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown joint in trajectory: " << joint_names[i]);
          return false;
        }

        double cur_position = current_state->getJointPositions(jm)[0];
        double traj_position = positions[i];
        // normalize positions and compare
        jm->enforcePositionBounds(&cur_position);
        jm->enforcePositionBounds(&traj_position);
        if (jm->distance(&cur_position, &traj_position) > allowed_start_tolerance_)
        {
          ROS_ERROR_NAMED(LOGNAME,
                          "\nInvalid Trajectory: start point deviates from current robot state more than %g"
                          "\njoint '%s': expected: %g, current: %g",
                          allowed_start_tolerance_, joint_names[i].c_str(), traj_position, cur_position);
          return false;
        }
      }
    }
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      // Check multi-dof trajectory
      const std::vector<geometry_msgs::Transform>& transforms =
          trajectory.multi_dof_joint_trajectory.points.front().transforms;
      const std::vector<std::string>& joint_names = trajectory.multi_dof_joint_trajectory.joint_names;
      if (transforms.size() != joint_names.size())
      {
        ROS_ERROR_NAMED(LOGNAME, "Wrong trajectory: #joints: %zu != #transforms: %zu", joint_names.size(),
                        transforms.size());
        return false;
      }

      for (std::size_t i = 0, end = joint_names.size(); i < end; ++i)
      {
        const moveit::core::JointModel* jm = current_state->getJointModel(joint_names[i]);
        if (!jm)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown joint in trajectory: " << joint_names[i]);
          return false;
        }

        // compute difference (offset vector and rotation angle) between current transform
        // and start transform in trajectory
        Eigen::Isometry3d cur_transform, start_transform;
        // computeTransform() computes a valid isometry by contract
        jm->computeTransform(current_state->getJointPositions(jm), cur_transform);
        start_transform = tf2::transformToEigen(transforms[i]);
        ASSERT_ISOMETRY(start_transform)  // unsanitized input, could contain a non-isometry
        Eigen::Vector3d offset = cur_transform.translation() - start_transform.translation();
        Eigen::AngleAxisd rotation;
        rotation.fromRotationMatrix(cur_transform.linear().transpose() * start_transform.linear());
        if ((offset.array() > allowed_start_tolerance_).any() || rotation.angle() > allowed_start_tolerance_)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME,
                                 "\nInvalid Trajectory: start point deviates from current robot state more than "
                                     << allowed_start_tolerance_ << "\nmulti-dof joint '" << joint_names[i]
                                     << "': pos delta: " << offset.transpose() << " rot delta: " << rotation.angle());
          return false;
        }
      }
    }
  }
  return true;
}

bool TrajectoryExecutionManager::configure(TrajectoryExecutionContext& context,
                                           const moveit_msgs::RobotTrajectory& trajectory,
                                           const std::vector<std::string>& controllers, const int& expiration_time)
{
  context.trajectory_ = trajectory;
  context.expiration_time = expiration_time;
  if (trajectory.multi_dof_joint_trajectory.points.empty() && trajectory.joint_trajectory.points.empty())
  {
    // empty trajectories don't need to configure anything
    return true;
  }
  std::set<std::string> actuated_joints;

  auto is_actuated = [this](const std::string& joint_name) -> bool {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_name);
    return (jm && !jm->isPassive() && !jm->getMimic() && jm->getType() != moveit::core::JointModel::FIXED);
  };
  for (const std::string& joint_name : trajectory.multi_dof_joint_trajectory.joint_names)
    if (is_actuated(joint_name))
      actuated_joints.insert(joint_name);
  for (const std::string& joint_name : trajectory.joint_trajectory.joint_names)
    if (is_actuated(joint_name))
      actuated_joints.insert(joint_name);

  if (actuated_joints.empty())
  {
    ROS_WARN_NAMED(LOGNAME, "The trajectory to execute specifies no joints");
    return false;
  }

  if (controllers.empty())
  {
    bool retry = true;
    bool reloaded = false;
    while (retry)
    {
      retry = false;
      std::vector<std::string> all_controller_names;
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
           it != known_controllers_.end(); ++it)
        all_controller_names.push_back(it->first);
      if (selectControllers(actuated_joints, all_controller_names, context.controllers_))
      {
        if (distributeTrajectory(trajectory, context.controllers_, context.trajectory_parts_))
          return true;
      }
      else
      {
        // maybe we failed because we did not have a complete list of controllers
        if (!reloaded)
        {
          reloadControllerInformation();
          reloaded = true;
          retry = true;
        }
      }
    }
  }
  else
  {
    // check if the specified controllers are valid names;
    // if they appear not to be, try to reload the controller information, just in case they are new in the system
    bool reloaded = false;
    for (const std::string& controller : controllers)
      if (known_controllers_.find(controller) == known_controllers_.end())
      {
        reloadControllerInformation();
        reloaded = true;
        break;
      }
    if (reloaded)
      for (const std::string& controller : controllers)
        if (known_controllers_.find(controller) == known_controllers_.end())
        {
          ROS_ERROR_NAMED(LOGNAME, "Controller '%s' is not known", controller.c_str());
          return false;
        }
    if (selectControllers(actuated_joints, controllers, context.controllers_))
    {
      if (distributeTrajectory(trajectory, context.controllers_, context.trajectory_parts_))
        return true;
    }
  }
  std::stringstream ss;
  for (const std::string& actuated_joint : actuated_joints)
    ss << actuated_joint << " ";
  ROS_ERROR_NAMED(LOGNAME, "Unable to identify any set of controllers that can actuate the specified joints: [ %s]",
                  ss.str().c_str());

  std::stringstream ss2;
  std::map<std::string, ControllerInformation>::const_iterator mi;
  for (mi = known_controllers_.begin(); mi != known_controllers_.end(); mi++)
  {
    ss2 << "controller '" << mi->second.name_ << "' controls joints:\n";

    std::set<std::string>::const_iterator ji;
    for (ji = mi->second.joints_.begin(); ji != mi->second.joints_.end(); ji++)
    {
      ss2 << "  " << *ji << std::endl;
    }
  }
  ROS_ERROR_NAMED(LOGNAME, "Known controllers and their joints:\n%s", ss2.str().c_str());
  return false;
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::executeAndWait(bool auto_clear)
{
  execute(ExecutionCompleteCallback(), auto_clear);
  return waitForExecution();
}

void TrajectoryExecutionManager::stopExecutionInternal()
{
  // execution_state_mutex_ needs to have been locked by the caller
  for (moveit_controller_manager::MoveItControllerHandlePtr& active_handle : active_handles_)
    try
    {
      active_handle->cancelExecution();
    }
    catch (std::exception& ex)
    {
      ROS_ERROR_NAMED(LOGNAME, "Caught %s when canceling execution.", ex.what());
    }
}

void TrajectoryExecutionManager::stopExecution(bool auto_clear)
{
  // wait for the execution thread to finish
  stop_continuous_execution_ = true;
  run_continuous_execution_thread_ = false;
  continuous_execution_condition_.notify_all();

  if (continuous_execution_thread_)
  {
    continuous_execution_thread_->join();
  }

  if (!execution_complete_)
  {
    execution_state_mutex_.lock();
    if (!execution_complete_)
    {
      // we call cancel for all active handles; we know these are not being modified as we loop through them because of
      // the lock
      // we mark execution_complete_ as true ahead of time. Using this flag, executePart() will know that an external
      // trigger to stop has been received
      execution_complete_ = true;
      stopExecutionInternal();

      // we set the status here; executePart() will not set status when execution_complete_ is true ahead of time
      last_execution_status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      execution_state_mutex_.unlock();
      ROS_INFO_NAMED(LOGNAME, "Stopped trajectory execution.");

      // wait for the execution thread to finish
      boost::mutex::scoped_lock lock(blocking_execution_thread_mutex_);
      if (blocking_execution_thread_)
      {
        blocking_execution_thread_->join();
        blocking_execution_thread_.reset();
      }

      if (auto_clear)
        clear();
    }
    else
      execution_state_mutex_.unlock();
  }
  else if (blocking_execution_thread_)  // just in case we have some thread waiting to be joined from some point in the
                                        // past, we join it now
  {
    boost::mutex::scoped_lock lock(blocking_execution_thread_mutex_);
    if (blocking_execution_thread_)
    {
      blocking_execution_thread_->join();
      blocking_execution_thread_.reset();
    }
  }
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback& callback, bool auto_clear)
{
  execute(callback, PathSegmentCompleteCallback(), auto_clear);
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback& callback,
                                         const PathSegmentCompleteCallback& part_callback, bool auto_clear)
{
  stopExecution(false);

  // check whether first trajectory starts at current robot state
  if (!trajectories_.empty() && !validate(*trajectories_.front()))
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (auto_clear)
      clear();
    if (callback)
      callback(last_execution_status_);
    return;
  }

  // start the execution thread
  execution_complete_ = false;
  blocking_execution_thread_.reset(
      new boost::thread(&TrajectoryExecutionManager::executeThread, this, callback, part_callback, auto_clear));
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::waitForExecution()
{
  {
    boost::unique_lock<boost::mutex> ulock(execution_state_mutex_);
    while (!execution_complete_)
      execution_complete_condition_.wait(ulock);
  }
  {
    boost::unique_lock<boost::mutex> ulock(continuous_execution_thread_mutex_);
    while (!continuous_execution_queue_.empty())
      continuous_execution_condition_.wait(ulock);
  }

  // this will join the thread for executing sequences of trajectories
  // stopExecution(false);
  // TODO (cambel): Is it ok to remove this line? Probably not.

  return last_execution_status_;
}

void TrajectoryExecutionManager::clear()
{
  if (execution_complete_)
  {
    for (std::shared_ptr<TrajectoryExecutionContext> trajectory : trajectories_)
      trajectory.reset();
    trajectories_.clear();
    {
      boost::mutex::scoped_lock slock(continuous_execution_thread_mutex_);
      while (!continuous_execution_queue_.empty())
      {
        continuous_execution_queue_.front().reset();
        continuous_execution_queue_.pop_front();
      }
    }
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Cannot push a new trajectory while another is being executed");
}

void TrajectoryExecutionManager::executeThread(const ExecutionCompleteCallback& callback,
                                               const PathSegmentCompleteCallback& part_callback, bool auto_clear)
{
  // if we already got a stop request before we even started anything, we abort
  if (execution_complete_)
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (callback)
      callback(last_execution_status_);
    return;
  }

  ROS_DEBUG_NAMED(LOGNAME, "Starting trajectory execution ...");
  // assume everything will be OK
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;

  // execute each trajectory, one after the other (executePart() is blocking) or until one fails.
  // on failure, the status is set by executePart(). Otherwise, it will remain as set above (success)
  std::size_t i = 0;
  for (; i < trajectories_.size(); ++i)
  {
    bool epart = executePart(i);
    if (epart && part_callback)
      part_callback(i);
    if (!epart || execution_complete_)
    {
      ++i;
      break;
    }
  }

  // only report that execution finished successfully when the robot actually stopped moving
  if (last_execution_status_ == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    waitForRobotToStop(*trajectories_[i - 1]);

  ROS_INFO_NAMED(LOGNAME, "Completed trajectory execution with status %s ...",
                 last_execution_status_.asString().c_str());

  // notify whoever is waiting for the event of trajectory completion
  execution_state_mutex_.lock();
  execution_complete_ = true;
  execution_state_mutex_.unlock();
  execution_complete_condition_.notify_all();

  // clear the paths just executed, if needed
  if (auto_clear)
    clear();

  // call user-specified callback
  if (callback)
    callback(last_execution_status_);
}

bool TrajectoryExecutionManager::executePart(std::size_t part_index)
{
  TrajectoryExecutionContext& context = *trajectories_[part_index];

  // first make sure desired controllers are active
  if (ensureActiveControllers(context.controllers_))
  {
    // stop if we are already asked to do so
    if (execution_complete_)
      return false;

    std::vector<moveit_controller_manager::MoveItControllerHandlePtr> handles;
    {
      boost::mutex::scoped_lock slock(execution_state_mutex_);
      if (!execution_complete_)
      {
        // time indexing uses this member too, so we lock this mutex as well
        time_index_mutex_.lock();
        current_context_ = part_index;
        time_index_mutex_.unlock();
        active_handles_.resize(context.controllers_.size());
        for (std::size_t i = 0; i < context.controllers_.size(); ++i)
        {
          moveit_controller_manager::MoveItControllerHandlePtr h;
          try
          {
            h = controller_manager_->getControllerHandle(context.controllers_[i]);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR_NAMED(LOGNAME, "Caught %s when retrieving controller handle", ex.what());
          }
          if (!h)
          {
            active_handles_.clear();
            current_context_ = -1;
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            ROS_ERROR_NAMED(LOGNAME, "No controller handle for controller '%s'. Aborting.",
                            context.controllers_[i].c_str());
            return false;
          }
          active_handles_[i] = h;
        }
        handles = active_handles_;  // keep a copy for later, to avoid thread safety issues
        for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
        {
          bool ok = false;
          try
          {
            ok = active_handles_[i]->sendTrajectory(context.trajectory_parts_[i]);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR_NAMED(LOGNAME, "Caught %s when sending trajectory to controller", ex.what());
          }
          if (!ok)
          {
            for (std::size_t j = 0; j < i; ++j)
              try
              {
                active_handles_[j]->cancelExecution();
              }
              catch (std::exception& ex)
              {
                ROS_ERROR_NAMED(LOGNAME, "Caught %s when canceling execution", ex.what());
              }
            ROS_ERROR_NAMED(LOGNAME, "Failed to send trajectory part %zu of %zu to controller %s", i + 1,
                            context.trajectory_parts_.size(), active_handles_[i]->getName().c_str());
            if (i > 0)
              ROS_ERROR_NAMED(LOGNAME, "Cancelling previously sent trajectory parts");
            active_handles_.clear();
            current_context_ = -1;
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            return false;
          }
        }
      }
    }

    // compute the expected duration of the trajectory and find the part of the trajectory that takes longest to execute
    ros::Time current_time = ros::Time::now();
    ros::Duration expected_trajectory_duration(0.0);
    int longest_part = -1;
    for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
    {
      ros::Duration d(0.0);
      if (!(context.trajectory_parts_[i].joint_trajectory.points.empty() &&
            context.trajectory_parts_[i].multi_dof_joint_trajectory.points.empty()))
      {
        if (context.trajectory_parts_[i].joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[i].joint_trajectory.header.stamp - current_time;
        if (context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp > current_time)
          d = std::max(d, context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp - current_time);
        d += std::max(context.trajectory_parts_[i].joint_trajectory.points.empty() ?
                          ros::Duration(0.0) :
                          context.trajectory_parts_[i].joint_trajectory.points.back().time_from_start,
                      context.trajectory_parts_[i].multi_dof_joint_trajectory.points.empty() ?
                          ros::Duration(0.0) :
                          context.trajectory_parts_[i].multi_dof_joint_trajectory.points.back().time_from_start);

        if (longest_part < 0 ||
            std::max(context.trajectory_parts_[i].joint_trajectory.points.size(),
                     context.trajectory_parts_[i].multi_dof_joint_trajectory.points.size()) >
                std::max(context.trajectory_parts_[longest_part].joint_trajectory.points.size(),
                         context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size()))
          longest_part = i;
      }

      // prefer controller-specific values over global ones if defined
      // TODO: the controller-specific parameters are static, but override
      //       the global ones are configurable via dynamic reconfigure
      std::map<std::string, double>::const_iterator scaling_it =
          controller_allowed_execution_duration_scaling_.find(context.controllers_[i]);
      const double current_scaling = scaling_it != controller_allowed_execution_duration_scaling_.end() ?
                                         scaling_it->second :
                                         allowed_execution_duration_scaling_;

      std::map<std::string, double>::const_iterator margin_it =
          controller_allowed_goal_duration_margin_.find(context.controllers_[i]);
      const double current_margin = margin_it != controller_allowed_goal_duration_margin_.end() ?
                                        margin_it->second :
                                        allowed_goal_duration_margin_;

      // expected duration is the duration of the longest part
      expected_trajectory_duration =
          std::max(d * current_scaling + ros::Duration(current_margin), expected_trajectory_duration);
    }

    // construct a map from expected time to state index, for easy access to expected state location
    if (longest_part >= 0)
    {
      boost::mutex::scoped_lock slock(time_index_mutex_);

      if (context.trajectory_parts_[longest_part].joint_trajectory.points.size() >=
          context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size())
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].joint_trajectory.header.stamp - current_time;
        for (trajectory_msgs::JointTrajectoryPoint& point :
             context.trajectory_parts_[longest_part].joint_trajectory.points)
          time_index_.push_back(current_time + d + point.time_from_start);
      }
      else
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp - current_time;
        for (trajectory_msgs::MultiDOFJointTrajectoryPoint& point :
             context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points)
          time_index_.push_back(current_time + d + point.time_from_start);
      }
    }

    bool result = true;
    for (moveit_controller_manager::MoveItControllerHandlePtr& handle : handles)
    {
      if (execution_duration_monitoring_)
      {
        if (!handle->waitForExecution(expected_trajectory_duration))
          if (!execution_complete_ && ros::Time::now() - current_time > expected_trajectory_duration)
          {
            ROS_ERROR_NAMED(LOGNAME,
                            "Controller is taking too long to execute trajectory (the expected upper "
                            "bound for the trajectory execution was %lf seconds). Stopping trajectory.",
                            expected_trajectory_duration.toSec());
            {
              boost::mutex::scoped_lock slock(execution_state_mutex_);
              stopExecutionInternal();  // this is really tricky. we can't call stopExecution() here, so we call the
                                        // internal function only
            }
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::TIMED_OUT;
            result = false;
            break;
          }
      }
      else
        handle->waitForExecution();

      // if something made the trajectory stop, we stop this thread too
      if (execution_complete_)
      {
        result = false;
        break;
      }
      else if (handle->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      {
        ROS_WARN_STREAM_NAMED(LOGNAME, "Controller handle " << handle->getName() << " reports status "
                                                            << handle->getLastExecutionStatus().asString());
        last_execution_status_ = handle->getLastExecutionStatus();
        result = false;
      }
    }

    // clear the active handles
    execution_state_mutex_.lock();
    active_handles_.clear();

    // clear the time index
    time_index_mutex_.lock();
    time_index_.clear();
    current_context_ = -1;
    time_index_mutex_.unlock();

    execution_state_mutex_.unlock();
    return result;
  }
  else
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    return false;
  }
}

bool TrajectoryExecutionManager::waitForRobotToStop(const TrajectoryExecutionContext& context, double wait_time)
{
  // skip waiting for convergence?
  if (allowed_start_tolerance_ == 0 || !wait_for_trajectory_completion_)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Not waiting for trajectory completion");
    return true;
  }

  ros::WallTime start = ros::WallTime::now();
  double time_remaining = wait_time;

  moveit::core::RobotStatePtr prev_state, cur_state;
  prev_state = csm_->getCurrentState();
  prev_state->enforceBounds();

  // assume robot stopped when 3 consecutive checks yield the same robot state
  unsigned int no_motion_count = 0;  // count iterations with no motion
  while (time_remaining > 0. && no_motion_count < 3)
  {
    if (!csm_->waitForCurrentState(ros::Time::now(), time_remaining) || !(cur_state = csm_->getCurrentState()))
    {
      ROS_WARN_NAMED(LOGNAME, "Failed to receive current joint state");
      return false;
    }
    cur_state->enforceBounds();
    time_remaining = wait_time - (ros::WallTime::now() - start).toSec();  // remaining wait_time

    // check for motion in effected joints of execution context
    bool moved = false;
    for (const auto& trajectory : context.trajectory_parts_)
    {
      const std::vector<std::string>& joint_names = trajectory.joint_trajectory.joint_names;
      const std::size_t n = joint_names.size();

      for (std::size_t i = 0; i < n && !moved; ++i)
      {
        const moveit::core::JointModel* jm = cur_state->getJointModel(joint_names[i]);
        if (!jm)
          continue;  // joint vanished from robot state (shouldn't happen), but we don't care

        if (fabs(cur_state->getJointPositions(jm)[0] - prev_state->getJointPositions(jm)[0]) > allowed_start_tolerance_)
        {
          moved = true;
          no_motion_count = 0;
          break;
        }
      }
    }

    if (!moved)
      ++no_motion_count;

    std::swap(prev_state, cur_state);
  }

  return time_remaining > 0;
}

std::pair<int, int> TrajectoryExecutionManager::getCurrentExpectedTrajectoryIndex() const
{
  boost::mutex::scoped_lock slock(time_index_mutex_);
  if (current_context_ < 0)
    return std::make_pair(-1, -1);
  if (time_index_.empty())
    return std::make_pair((int)current_context_, -1);
  std::vector<ros::Time>::const_iterator time_index_it =
      std::lower_bound(time_index_.begin(), time_index_.end(), ros::Time::now());
  int pos = time_index_it - time_index_.begin();
  return std::make_pair((int)current_context_, pos);
}

const std::vector<std::shared_ptr<TrajectoryExecutionManager::TrajectoryExecutionContext>>&
TrajectoryExecutionManager::getTrajectories() const
{
  return trajectories_;
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::getLastExecutionStatus() const
{
  return last_execution_status_;
}

bool TrajectoryExecutionManager::ensureActiveControllersForGroup(const std::string& group)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group);
  if (joint_model_group)
    return ensureActiveControllersForJoints(joint_model_group->getJointModelNames());
  else
    return false;
}

bool TrajectoryExecutionManager::ensureActiveControllersForJoints(const std::vector<std::string>& joints)
{
  std::vector<std::string> all_controller_names;
  for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
       it != known_controllers_.end(); ++it)
    all_controller_names.push_back(it->first);
  std::vector<std::string> selected_controllers;
  std::set<std::string> jset;
  for (const std::string& joint : joints)
  {
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != nullptr || jm->getType() == moveit::core::JointModel::FIXED)
        continue;
      jset.insert(joint);
    }
  }

  if (selectControllers(jset, all_controller_names, selected_controllers))
    return ensureActiveControllers(selected_controllers);
  else
    return false;
}

bool TrajectoryExecutionManager::ensureActiveController(const std::string& controller)
{
  return ensureActiveControllers(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::ensureActiveControllers(const std::vector<std::string>& controllers)
{
  updateControllersState(DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);

  if (manage_controllers_)
  {
    std::vector<std::string> controllers_to_activate;
    std::vector<std::string> controllers_to_deactivate;
    std::set<std::string> joints_to_be_activated;
    std::set<std::string> joints_to_be_deactivated;
    for (const std::string& controller : controllers)
    {
      std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.find(controller);
      if (it == known_controllers_.end())
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Controller " << controller << " is not known");
        return false;
      }
      if (!it->second.state_.active_)
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Need to activate " << controller);
        controllers_to_activate.push_back(controller);
        joints_to_be_activated.insert(it->second.joints_.begin(), it->second.joints_.end());
        for (const std::string& overlapping_controller : it->second.overlapping_controllers_)
        {
          const ControllerInformation& ci = known_controllers_[overlapping_controller];
          if (ci.state_.active_)
          {
            controllers_to_deactivate.push_back(overlapping_controller);
            joints_to_be_deactivated.insert(ci.joints_.begin(), ci.joints_.end());
          }
        }
      }
      else
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Controller " << controller << " is already active");
    }
    std::set<std::string> diff;
    std::set_difference(joints_to_be_deactivated.begin(), joints_to_be_deactivated.end(),
                        joints_to_be_activated.begin(), joints_to_be_activated.end(), std::inserter(diff, diff.end()));
    if (!diff.empty())
    {
      // find the set of controllers that do not overlap with the ones we want to activate so far
      std::vector<std::string> possible_additional_controllers;
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
           it != known_controllers_.end(); ++it)
      {
        bool ok = true;
        for (const std::string& controller_to_activate : controllers_to_activate)
          if (it->second.overlapping_controllers_.find(controller_to_activate) !=
              it->second.overlapping_controllers_.end())
          {
            ok = false;
            break;
          }
        if (ok)
          possible_additional_controllers.push_back(it->first);
      }

      // out of the allowable controllers, try to find a subset of controllers that covers the joints to be actuated
      std::vector<std::string> additional_controllers;
      if (selectControllers(diff, possible_additional_controllers, additional_controllers))
        controllers_to_activate.insert(controllers_to_activate.end(), additional_controllers.begin(),
                                       additional_controllers.end());
      else
        return false;
    }
    if (!controllers_to_activate.empty() || !controllers_to_deactivate.empty())
    {
      if (controller_manager_)
      {
        // load controllers to be activated, if needed, and reset the state update cache
        for (const std::string& controller_to_activate : controllers_to_activate)
        {
          ControllerInformation& ci = known_controllers_[controller_to_activate];
          ci.last_update_ = ros::Time();
        }
        // reset the state update cache
        for (const std::string& controller_to_activate : controllers_to_deactivate)
          known_controllers_[controller_to_activate].last_update_ = ros::Time();
        return controller_manager_->switchControllers(controllers_to_activate, controllers_to_deactivate);
      }
      else
        return false;
    }
    else
      return true;
  }
  else
  {
    std::set<std::string> originally_active;
    for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin();
         it != known_controllers_.end(); ++it)
      if (it->second.state_.active_)
        originally_active.insert(it->first);
    return std::includes(originally_active.begin(), originally_active.end(), controllers.begin(), controllers.end());
  }
}

void TrajectoryExecutionManager::loadControllerParams()
{
  XmlRpc::XmlRpcValue controller_list;
  if (node_handle_.getParam("controller_list", controller_list) &&
      controller_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < controller_list.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      XmlRpc::XmlRpcValue& controller = controller_list[i];
      if (controller.hasMember("name"))
      {
        if (controller.hasMember("allowed_execution_duration_scaling"))
          controller_allowed_execution_duration_scaling_[std::string(controller["name"])] =
              controller["allowed_execution_duration_scaling"];
        if (controller.hasMember("allowed_goal_duration_margin"))
          controller_allowed_goal_duration_margin_[std::string(controller["name"])] =
              controller["allowed_goal_duration_margin"];
      }
    }
  }
}

void TrajectoryExecutionManager::updateActiveHandlesAndContexts(
    std::set<moveit_controller_manager::MoveItControllerHandlePtr>& used_handles,
    std::vector<std::shared_ptr<TrajectoryExecutionContext>>& active_contexts)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Entered updateActiveHandlesAndContexts");
  // Go through list of current trajectories, check the statuses of all handles
  // for (auto& context_controllers_pair : active_contexts)  // first: context. second: handles_ for that trajectory
  for (auto it = active_contexts.begin(); it != active_contexts.end();)
  {
    auto& context = *it;

    std::set<moveit_controller_manager::MoveItControllerHandlePtr> context_handles;
    getContextHandles(*context, context_handles);

    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Update context");

    // TODO(felixvd): This doesn't cover all the cases, like TIMED_OUT, CONTROL_FAILED. Ugh.
    bool some_aborted = false;
    bool all_aborted = true;
    bool some_running = false;
    bool all_running = true;
    bool some_succeeded = false;
    bool all_succeeded = true;
    for (const auto& handle : context_handles)
    {
      auto last_status = handle->getLastExecutionStatus();
      if (last_status == moveit_controller_manager::ExecutionStatus::ABORTED ||
          last_status == moveit_controller_manager::ExecutionStatus::TIMED_OUT ||
          last_status == moveit_controller_manager::ExecutionStatus::FAILED ||
          last_status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
      // TODO: This converts everything to ABORTED.
      {
        some_aborted = true;
        all_succeeded = false;
        all_running = false;
      }
      else if (last_status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      {
        all_aborted = false;
        some_succeeded = true;
        all_running = false;
      }
      else if (last_status == moveit_controller_manager::ExecutionStatus::RUNNING)
      {
        all_aborted = false;
        all_succeeded = false;
        some_running = true;
      }
    }

    moveit_controller_manager::ExecutionStatus combined_status;
    if (some_aborted || all_aborted)
      combined_status = moveit_controller_manager::ExecutionStatus::ABORTED;
    else if (some_running || all_running)
      combined_status = moveit_controller_manager::ExecutionStatus::RUNNING;
    else if (all_succeeded)
      combined_status = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else
      continue;

    if (combined_status == moveit_controller_manager::ExecutionStatus::SUCCEEDED ||
        combined_status == moveit_controller_manager::ExecutionStatus::ABORTED)
    {
      if (!context->execution_complete_callback.empty())
        context->execution_complete_callback(combined_status);
      it = active_contexts.erase(it);

    }
    else
    {
      it++;
    }
  }

  // Remove controller handles from list if they are not executing a trajectory
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Cleaning used_handles");
  for (std::set<moveit_controller_manager::MoveItControllerHandlePtr>::iterator uit = used_handles.begin();
       uit != used_handles.end();)
    if ((*uit)->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::RUNNING)
    {
      std::map<moveit_controller_manager::MoveItControllerHandlePtr, moveit_msgs::RobotTrajectory>::iterator it;
      uit = used_handles.erase(uit);
    }
    else
      ++uit;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Done updateActiveHandlesAndContexts");
}

// simultaneous execution
bool TrajectoryExecutionManager::checkCollisionBetweenTrajectories(const moveit_msgs::RobotTrajectory& new_trajectory,
                                                                   const moveit_msgs::RobotTrajectory& active_trajectory)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Start checkCollision between trajectories using PlanningScene.isPathValid()");
  // before we start checking collisions, ensure that we have the latest robot state received...
  planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  planning_scene_monitor_->updateFrameTransforms();

  planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
  moveit::core::RobotState start_state = ps->getCurrentState();

  moveit_msgs::RobotState start_state_msg;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Compute collision check");
  for (std::size_t i = 0; i < active_trajectory.joint_trajectory.points.size(); ++i)
    if (jointTrajPointToRobotState(active_trajectory.joint_trajectory, i, start_state))
    {
      robotStateToRobotStateMsg(start_state, start_state_msg);
      if (!ps->isPathValid(start_state_msg, new_trajectory, new_trajectory.group_name, true))
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Done checkCollision between trajectories: Collision found!");
        return false;  // Return as soon as any point is invalid
      }
    }
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Done checkCollision between trajectories: No collisions found");
  return true;
}

bool TrajectoryExecutionManager::checkContextForCollisions(
    const TrajectoryExecutionContext& context,
    const std::vector<std::shared_ptr<TrajectoryExecutionContext>>& active_contexts)
{
  // 2. Check that new trajectory does not collide with other active trajectories

  /* Approach: checking point by point :
        a. Get planning scene
        b. Update the planning scene with the first point of both trajectories
        c. Use robot's group name to check collisions of relevant links only
        d. Check if the two states collide, if so, return, no need to check anything else
      - Can be optimized by estimating the position of the robot in the active trajectory, then triming the active
     trajectory to check only the remaining states
      - May be optimized by active trajectory backward, it is more likely that any collision would happen at the end of
     the trajectory than at the beginning (given that the planning return a valid trajectory).
  */

  for (const auto& currently_running_context : active_contexts)
  {
    moveit_msgs::RobotTrajectory& currently_running_trajectory = currently_running_context->trajectory_;

    if (!checkCollisionBetweenTrajectories(context.trajectory_, currently_running_trajectory))
    {
      // Push to backlog
      ROS_DEBUG_STREAM_NAMED(
          LOGNAME,
          "Collision found between trajectory with duration: "
              << context.trajectory_parts_[0].joint_trajectory.points.back().time_from_start
              << " and trajectory with duration: "
              << currently_running_context->trajectory_parts_[0].joint_trajectory.points.back().time_from_start);
      return false;
    }
  }
  return true;
}

bool TrajectoryExecutionManager::checkCollisionsWithCurrentState(const moveit_msgs::RobotTrajectory& trajectory)
{
  moveit::core::RobotStatePtr current_state;
  if (!csm_->waitForCurrentState(ros::Time::now()) || !(current_state = csm_->getCurrentState()))
  {
    ROS_DEBUG_NAMED(LOGNAME, "Failed to validate trajectory: couldn't receive full current joint state within 1s");
    return false;
  }
  planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
  if (jointTrajPointToRobotState(trajectory.joint_trajectory, trajectory.joint_trajectory.points.size() - 1,
                                 *current_state))
  {
    moveit_msgs::RobotState robot_state_msg;
    robotStateToRobotStateMsg(*current_state, robot_state_msg);
    if (!ps->isPathValid(robot_state_msg, trajectory, trajectory.group_name))
    {
      ROS_DEBUG_NAMED(LOGNAME, "New trajectory collides with the current robot state. Abort!");
      last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      return false;
    }
  }

  return true;
}

void TrajectoryExecutionManager::getContextHandles(
    const TrajectoryExecutionContext& context, std::set<moveit_controller_manager::MoveItControllerHandlePtr>& handles)
{
  for (std::size_t i = 0; i < context.controllers_.size(); ++i)
  {
    moveit_controller_manager::MoveItControllerHandlePtr h;
    try
    {
      h = controller_manager_->getControllerHandle(context.controllers_[i]);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR_NAMED(LOGNAME, "%s caught when retrieving controller handle", ex.what());
    }
    if (!h)
    {
      last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      ROS_ERROR_NAMED(LOGNAME, "No controller handle for controller '%s'. Aborting.", context.controllers_[i].c_str());
      handles.clear();
      break;
    }
    handles.insert(h);
  }
}

bool TrajectoryExecutionManager::validateAndExecuteContext(
    const TrajectoryExecutionContext& context,
    std::set<moveit_controller_manager::MoveItControllerHandlePtr>& used_handles,
    std::vector<std::shared_ptr<TrajectoryExecutionContext>>& active_contexts)
{
  ROS_DEBUG_NAMED(LOGNAME, "Start validateAndExecuteContext");
  updateActiveHandlesAndContexts(used_handles, active_contexts);

  // Get the controller handles needed to execute the new trajectory
  std::set<moveit_controller_manager::MoveItControllerHandlePtr> handles;
  getContextHandles(context, handles);
  if (handles.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Trajectory context had no controller handles??");
    return false;
  }

  // Break out if flags set
  if (stop_continuous_execution_ || !run_continuous_execution_thread_)
  {
    return false;
  }

  ROS_DEBUG_NAMED(LOGNAME, "DEBUG: Printing necessary handles for new traj");
  for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "handle: " << (i + 1) << " of " << context.trajectory_parts_.size() << " : "
                                               << (*std::next(handles.begin(), i))->getName());
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Next-up trajectory has duration "
                                        << context.trajectory_parts_[i].joint_trajectory.points.back().time_from_start);
  }

  // 1. Skip trajectory if it collides with other active trajectories
  if (!checkContextForCollisions(context, active_contexts))
  {
    return false;
  }

  // 2. Check that controllers are not busy, wait for execution to finish if they are.
  for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
  {
    std::set<moveit_controller_manager::MoveItControllerHandlePtr>::iterator uit = used_handles.begin();

    // Check if required handle is already in use
    while (uit != used_handles.end())
      if ((*std::next(handles.begin(), i))->getName() ==
          (*uit)->getName())  // If controller is busy, return false so trajectory is pushed to backlog
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Handle " << (*std::next(handles.begin(), i))->getName() << " already in use: "
                                                  << (*uit)->getLastExecutionStatus().asString());
        return false;
      }
      else
        ++uit;
  }

  // 3. Skip trajectory if it collides with current state
  if (!checkCollisionsWithCurrentState(context.trajectory_))
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Trajectory collides with current state. Cannot execute yet.");
    return false;
  }

  // Check whether this trajectory starts at current robot state
  if (!validate(context))
  {
    ROS_ERROR_NAMED(LOGNAME, "Trajectory became invalid before execution, abort.");
    if (!context.execution_complete_callback.empty())
      context.execution_complete_callback(moveit_controller_manager::ExecutionStatus::ABORTED);
    return true;
  }

  // Push trajectory to all controllers simultaneously (each part goes to one controller)
  for (std::size_t i = 0; i < context.trajectory_parts_.size(); ++i)
  {
    // Send trajectory (part) to controller
    bool ok = false;
    try
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME,
                             "Sending trajectory to controller: " << (*std::next(handles.begin(), i))->getName());
      ROS_DEBUG_STREAM_NAMED(
          LOGNAME, "duration: " << context.trajectory_parts_[i].joint_trajectory.points.back().time_from_start);
      ok = (*std::next(handles.begin(), i))->sendTrajectory(context.trajectory_parts_[i]);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR_NAMED(LOGNAME, "Caught %s when sending trajectory to controller", ex.what());
    }
    if (!ok)
    {
      for (std::size_t j = 0; j < i; ++j)
        try
        {
          (*std::next(handles.begin(), j))->cancelExecution();
        }
        catch (std::exception& ex)
        {
          ROS_ERROR_NAMED(LOGNAME, "Caught %s when canceling execution", ex.what());
        }
      ROS_ERROR_NAMED(LOGNAME, "Failed to send trajectory part %zu of %zu to controller %s", i + 1,
                      context.trajectory_parts_.size(), (*std::next(handles.begin(), i))->getName().c_str());
      if (i > 0)
        ROS_ERROR_NAMED(LOGNAME, "Cancelling previously sent trajectory parts");
      return false;
    }
  }

  // Remember which handles are now in use and which trajectories they execute
  if (!handles.empty())
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Populating the lists with " << handles.size() << " handles.");

    used_handles.insert(handles.begin(), handles.end());

    active_contexts.push_back(std::make_shared<TrajectoryExecutionContext>(context));
  }
  return true;
}

bool TrajectoryExecutionManager::hasCommonHandles(const TrajectoryExecutionContext& context1,
                                                  const TrajectoryExecutionContext& context2)
{
  std::set<moveit_controller_manager::MoveItControllerHandlePtr> ctx1_handles;
  std::set<moveit_controller_manager::MoveItControllerHandlePtr> ctx2_handles;
  getContextHandles(context1, ctx1_handles);
  getContextHandles(context2, ctx2_handles);
  return ctx1_handles == ctx2_handles;
}

}  // namespace trajectory_execution_manager
