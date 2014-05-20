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
#include <moveit_ros_planning/TrajectoryExecutionDynamicReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

namespace trajectory_execution_manager
{

const std::string TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC = "trajectory_execution_event";

static const ros::Duration DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE(1.0);
static const double DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN = 0.5; // allow 0.5s more than the expected execution time before triggering a trajectory cancel (applied after scaling)
static const double DEFAULT_CONTROLLER_GOAL_DURATION_SCALING = 1.1; // allow the execution of a trajectory to take more time than expected (scaled by a value > 1)

using namespace moveit_ros_planning;

class TrajectoryExecutionManager::DynamicReconfigureImpl
{
public:

  DynamicReconfigureImpl(TrajectoryExecutionManager *owner) : owner_(owner),
                                                              dynamic_reconfigure_server_(ros::NodeHandle("~/trajectory_execution"))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

private:

  void dynamicReconfigureCallback(TrajectoryExecutionDynamicReconfigureConfig &config, uint32_t level)
  {
    owner_->enableExecutionDurationMonitoring(config.execution_duration_monitoring);
    owner_->setAllowedExecutionDurationScaling(config.allowed_execution_duration_scaling);
  }

  TrajectoryExecutionManager *owner_;
  dynamic_reconfigure::Server<TrajectoryExecutionDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

TrajectoryExecutionManager::TrajectoryExecutionManager(const robot_model::RobotModelConstPtr &kmodel) :
  robot_model_(kmodel), node_handle_("~")
{
  if (!node_handle_.getParam("moveit_manage_controllers", manage_controllers_))
    manage_controllers_ = false;

  if (!node_handle_.getParam("allowed_execution_duration_scaling", allowed_execution_duration_scaling_))
    allowed_execution_duration_scaling_ = DEFAULT_CONTROLLER_GOAL_DURATION_SCALING;

  if (!node_handle_.getParam("allowed_goal_duration_margin", allowed_goal_duration_margin_))
    allowed_goal_duration_margin_ = DEFAULT_CONTROLLER_GOAL_DURATION_MARGIN;

  initialize();
}

TrajectoryExecutionManager::TrajectoryExecutionManager(const robot_model::RobotModelConstPtr &kmodel, bool manage_controllers) :
  robot_model_(kmodel), node_handle_("~"), manage_controllers_(manage_controllers)
{
  initialize();
}

TrajectoryExecutionManager::~TrajectoryExecutionManager()
{
  run_continuous_execution_thread_ = false;
  stopExecution(true);
  delete reconfigure_impl_;
}

void TrajectoryExecutionManager::initialize()
{
  reconfigure_impl_ = NULL;
  verbose_ = false;
  execution_complete_ = true;
  stop_continuous_execution_ = false;
  current_context_ = -1;
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  run_continuous_execution_thread_ = true;
  execution_duration_monitoring_ = true;
  execution_velocity_scaling_ = 1.0;

  // load the controller manager plugin
  try
  {
    controller_manager_loader_.reset(new pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>("moveit_core", "moveit_controller_manager::MoveItControllerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating controller manager plugin loader: " << ex.what());
    return;
  }

  if (controller_manager_loader_)
  {
    std::string controller;
    if (!node_handle_.getParam("moveit_controller_manager", controller))
    {
      const std::vector<std::string> &classes = controller_manager_loader_->getDeclaredClasses();
      if (classes.size() == 1)
      {
        controller = classes[0];
        ROS_WARN("Parameter '~moveit_controller_manager' is not specified but only one matching plugin was found: '%s'. Using that one.", controller.c_str());
      }
      else
        ROS_FATAL("Parameter '~moveit_controller_manager' not specified. This is needed to identify the plugin to use for interacting with controllers. No paths can be executed.");
    }

    if (!controller.empty())
      try
      {
        controller_manager_.reset(controller_manager_loader_->createUnmanagedInstance(controller));
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_FATAL_STREAM("Exception while loading controller manager '" << controller << "': " << ex.what());
      }
  }

  // other configuration steps
  reloadControllerInformation();

  event_topic_subscriber_ = root_node_handle_.subscribe(EXECUTION_EVENT_TOPIC, 100, &TrajectoryExecutionManager::receiveEvent, this);

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  if (manage_controllers_)
    ROS_INFO("Trajectory execution is managing controllers");
  else
    ROS_INFO("Trajectory execution is not managing controllers");
}

void TrajectoryExecutionManager::enableExecutionDurationMonitoring(bool flag)
{
  execution_duration_monitoring_ = flag;
}

void TrajectoryExecutionManager::setAllowedExecutionDurationScaling(double scaling)
{
  allowed_execution_duration_scaling_ = scaling;
}

void TrajectoryExecutionManager::setExecutionVelocityScaling(double scaling)
{
  execution_velocity_scaling_ = scaling;
}

bool TrajectoryExecutionManager::isManagingControllers() const
{
  return manage_controllers_;
}

const moveit_controller_manager::MoveItControllerManagerPtr& TrajectoryExecutionManager::getControllerManager() const
{
  return controller_manager_;
}

void TrajectoryExecutionManager::processEvent(const std::string &event)
{
  if (event == "stop")
    stopExecution(true);
  else
    ROS_WARN_STREAM("Unknown event type: '" << event << "'");
}

void TrajectoryExecutionManager::receiveEvent(const std_msgs::StringConstPtr &event)
{
  ROS_INFO_STREAM("Received event '" << event->data << "'");
  processEvent(event->data);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::RobotTrajectory &trajectory, const std::string &controller)
{
  if (controller.empty())
    return push(trajectory, std::vector<std::string>());
  else
    return push(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::JointTrajectory &trajectory, const std::string &controller)
{
  if (controller.empty())
    return push(trajectory, std::vector<std::string>());
  else
    return push(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::push(const trajectory_msgs::JointTrajectory &trajectory, const std::vector<std::string> &controllers)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory = trajectory;
  return push(traj, controllers);
}

bool TrajectoryExecutionManager::push(const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers)
{
  if (!execution_complete_)
  {
    ROS_ERROR("Cannot push a new trajectory while another is being executed");
    return false;
  }

  TrajectoryExecutionContext *context = new TrajectoryExecutionContext();
  if (configure(*context, trajectory, controllers))
  {
    if (verbose_)
    {
      std::stringstream ss;
      ss << "Pushed trajectory for execution using controllers [ ";
      for (std::size_t i = 0 ; i < context->controllers_.size() ; ++i)
        ss << context->controllers_[i] << " ";
      ss << "]:" << std::endl;
      for (std::size_t i = 0 ; i < context->trajectory_parts_.size() ; ++i)
        ss << context->trajectory_parts_[i] << std::endl;
      ROS_INFO("%s", ss.str().c_str());
    }
    trajectories_.push_back(context);
    return true;
  }
  else
  {
    delete context;
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  }

  return false;
}

bool TrajectoryExecutionManager::pushAndExecute(const moveit_msgs::RobotTrajectory &trajectory, const std::string &controller)
{
  if (controller.empty())
    return pushAndExecute(trajectory, std::vector<std::string>());
  else
    return pushAndExecute(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::pushAndExecute(const trajectory_msgs::JointTrajectory &trajectory, const std::string &controller)
{
  if (controller.empty())
    return pushAndExecute(trajectory, std::vector<std::string>());
  else
    return pushAndExecute(trajectory, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::pushAndExecute(const sensor_msgs::JointState &state, const std::string &controller)
{
  if (controller.empty())
    return pushAndExecute(state, std::vector<std::string>());
  else
    return pushAndExecute(state, std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::pushAndExecute(const trajectory_msgs::JointTrajectory &trajectory, const std::vector<std::string> &controllers)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory = trajectory;
  return pushAndExecute(traj, controllers);
}

bool TrajectoryExecutionManager::pushAndExecute(const sensor_msgs::JointState &state, const std::vector<std::string> &controllers)
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory.header = state.header;
  traj.joint_trajectory.joint_names = state.name;
  traj.joint_trajectory.points.resize(1);
  traj.joint_trajectory.points[0].positions = state.position;
  traj.joint_trajectory.points[0].velocities = state.velocity;
  traj.joint_trajectory.points[0].effort = state.effort;
  traj.joint_trajectory.points[0].time_from_start = ros::Duration(0, 0);
  return pushAndExecute(traj, controllers);
}

bool TrajectoryExecutionManager::pushAndExecute(const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers)
{
  if (!execution_complete_)
  {
    ROS_ERROR("Cannot push & execute a new trajectory while another is being executed");
    return false;
  }

  TrajectoryExecutionContext *context = new TrajectoryExecutionContext();
  if (configure(*context, trajectory, controllers))
  {
    {
      boost::mutex::scoped_lock slock(continuous_execution_mutex_);
      continuous_execution_queue_.push_back(context);
      if (!continuous_execution_thread_)
        continuous_execution_thread_.reset(new boost::thread(boost::bind(&TrajectoryExecutionManager::continuousExecutionThread, this)));
    }
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    continuous_execution_condition_.notify_all();
    return true;
  }
  else
  {
    delete context;
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    return false;
  }
}

void TrajectoryExecutionManager::continuousExecutionThread()
{
  std::set<moveit_controller_manager::MoveItControllerHandlePtr> used_handles;
  while (run_continuous_execution_thread_)
  {
    if (!stop_continuous_execution_)
    {
      boost::unique_lock<boost::mutex> ulock(continuous_execution_mutex_);
      while (continuous_execution_queue_.empty() && run_continuous_execution_thread_ && !stop_continuous_execution_)
        continuous_execution_condition_.wait(ulock);
    }

    if (stop_continuous_execution_ || !run_continuous_execution_thread_)
    {
      for (std::set<moveit_controller_manager::MoveItControllerHandlePtr>::iterator uit = used_handles.begin() ; uit != used_handles.end() ; ++uit)
        if ((*uit)->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::RUNNING)
          (*uit)->cancelExecution();
      used_handles.clear();
      while (!continuous_execution_queue_.empty())
      {
        TrajectoryExecutionContext *context = continuous_execution_queue_.front();
        continuous_execution_queue_.pop_front();
        delete context;
      }
      stop_continuous_execution_ = false;
      continue;
    }

    while (!continuous_execution_queue_.empty())
    {
      TrajectoryExecutionContext *context = NULL;
      {
        boost::mutex::scoped_lock slock(continuous_execution_mutex_);
        if (continuous_execution_queue_.empty())
          break;
        context = continuous_execution_queue_.front();
        continuous_execution_queue_.pop_front();
        if (continuous_execution_queue_.empty())
          continuous_execution_condition_.notify_all();
      }

      // remove handles we no longer need
      std::set<moveit_controller_manager::MoveItControllerHandlePtr>::iterator uit = used_handles.begin();
      while (uit != used_handles.end())
        if ((*uit)->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::RUNNING)
        {
          std::set<moveit_controller_manager::MoveItControllerHandlePtr>::iterator toErase = uit;
          ++uit;
          used_handles.erase(toErase);
        }
        else
          ++uit;

      // now send stuff to controllers

      // first make sure desired controllers are active
      if (areControllersActive(context->controllers_))
      {
        // get the controller handles needed to execute the new trajectory
        std::vector<moveit_controller_manager::MoveItControllerHandlePtr> handles(context->controllers_.size());
        for (std::size_t i = 0 ; i < context->controllers_.size() ; ++i)
        {
          moveit_controller_manager::MoveItControllerHandlePtr h;
          try
          {
            h = controller_manager_->getControllerHandle(context->controllers_[i]);
          }
          catch(...)
          {
            ROS_ERROR("Exception caught when retrieving controller handle");
          }
          if (!h)
          {
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            ROS_ERROR("No controller handle for controller '%s'. Aborting.", context->controllers_[i].c_str());
            handles.clear();
            break;
          }
          handles[i] = h;
        }

        if (stop_continuous_execution_ || !run_continuous_execution_thread_)
        {
          delete context;
          break;
        }

        // push all trajectories to all controllers simultaneously
        if (!handles.empty())
          for (std::size_t i = 0 ; i < context->trajectory_parts_.size() ; ++i)
          {
            bool ok = false;
            try
            {
              ok = handles[i]->sendTrajectory(context->trajectory_parts_[i]);
            }
            catch(...)
            {
              ROS_ERROR("Exception caught when sending trajectory to controller");
            }
            if (!ok)
            {
              for (std::size_t j = 0 ; j < i ; ++j)
                try
                {
                  handles[j]->cancelExecution();
                }
                catch(...)
                {
                  ROS_ERROR("Exception caught when canceling execution");
                }
              ROS_ERROR("Failed to send trajectory part %zu of %zu to controller %s", i + 1, context->trajectory_parts_.size(), handles[i]->getName().c_str());
              if (i > 0)
                ROS_ERROR("Cancelling previously sent trajectory parts");
              last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
              handles.clear();
              break;
            }
          }
        delete context;

        // remember which handles we used
        for (std::size_t i = 0 ; i < handles.size() ; ++i)
          used_handles.insert(handles[i]);
      }
      else
      {
        ROS_ERROR("Not all needed controllers are active. Cannot push and execute. You can try calling ensureActiveControllers() before pushAndExecute()");
        last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
        delete context;
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
    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
      std::vector<std::string> joints;
      controller_manager_->getControllerJoints(names[i], joints);
      ControllerInformation ci;
      ci.name_ = names[i];
      ci.joints_.insert(joints.begin(), joints.end());
      known_controllers_[ci.name_] = ci;
    }

    for (std::map<std::string, ControllerInformation>::iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
      for (std::map<std::string, ControllerInformation>::iterator jt = known_controllers_.begin() ; jt != known_controllers_.end() ; ++jt)
        if (it != jt)
        {
          std::vector<std::string> intersect;
          std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(),
                                jt->second.joints_.begin(), jt->second.joints_.end(),
                                std::back_inserter(intersect));
          if (!intersect.empty())
          {
            it->second.overlapping_controllers_.insert(jt->first);
            jt->second.overlapping_controllers_.insert(it->first);
          }
        }
  }
}

void TrajectoryExecutionManager::updateControllerState(const std::string &controller, const ros::Duration &age)
{
  std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controller);
  if (it != known_controllers_.end())
    updateControllerState(it->second, age);
  else
    ROS_ERROR("Controller '%s' is not known.", controller.c_str());
}

void TrajectoryExecutionManager::updateControllerState(ControllerInformation &ci, const ros::Duration &age)
{
  if (ros::Time::now() - ci.last_update_ >= age)
  {
    if (controller_manager_)
    {
      if (verbose_)
        ROS_INFO("Updating information for controller '%s'.", ci.name_.c_str());
      ci.state_ = controller_manager_->getControllerState(ci.name_);
      ci.last_update_ = ros::Time::now();
    }
  }
  else
    if (verbose_)
      ROS_INFO("Information for controller '%s' is assumed to be up to date.", ci.name_.c_str());
}

void TrajectoryExecutionManager::updateControllersState(const ros::Duration &age)
{
  for (std::map<std::string, ControllerInformation>::iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
    updateControllerState(it->second, age);
}

bool TrajectoryExecutionManager::checkControllerCombination(std::vector<std::string> &selected, const std::set<std::string> &actuated_joints)
{
  std::set<std::string> combined_joints;
  for (std::size_t i = 0 ; i < selected.size() ; ++i)
  {
    const ControllerInformation &ci = known_controllers_[selected[i]];
    combined_joints.insert(ci.joints_.begin(), ci.joints_.end());
  }

  if (verbose_)
  {
    std::stringstream ss, saj, sac;
    for (std::size_t i = 0 ; i < selected.size() ; ++i)
      ss << selected[i] << " ";
    for (std::set<std::string>::const_iterator it = actuated_joints.begin() ; it != actuated_joints.end() ; ++it)
      saj << *it << " ";
    for (std::set<std::string>::const_iterator it = combined_joints.begin() ; it != combined_joints.end() ; ++it)
      sac << *it << " ";
    ROS_INFO("Checking if controllers [ %s] operating on joints [ %s] cover joints [ %s]", ss.str().c_str(), sac.str().c_str(), saj.str().c_str());
  }

  return std::includes(combined_joints.begin(), combined_joints.end(),
                       actuated_joints.begin(), actuated_joints.end());
}

void TrajectoryExecutionManager::generateControllerCombination(std::size_t start_index, std::size_t controller_count,
                                                               const std::vector<std::string> &available_controllers,
                                                               std::vector<std::string> &selected_controllers,
                                                               std::vector< std::vector<std::string> > &selected_options,
                                                               const std::set<std::string> &actuated_joints)
{
  if (selected_controllers.size() == controller_count)
  {
    if (checkControllerCombination(selected_controllers, actuated_joints))
      selected_options.push_back(selected_controllers);
    return;
  }

  for (std::size_t i = start_index ; i < available_controllers.size() ; ++i)
  {
    bool overlap = false;
    const ControllerInformation &ci = known_controllers_[available_controllers[i]];
    for (std::size_t j = 0 ; j < selected_controllers.size() && !overlap ; ++j)
    {
      if (ci.overlapping_controllers_.find(selected_controllers[j]) != ci.overlapping_controllers_.end())
        overlap = true;
    }
    if (overlap)
      continue;
    selected_controllers.push_back(available_controllers[i]);
    generateControllerCombination(i + 1, controller_count, available_controllers, selected_controllers, selected_options, actuated_joints);
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

  std::vector< std::vector<std::string> > selected_options;
  std::vector<std::size_t> nrdefault;
  std::vector<std::size_t> nrjoints;
  std::vector<std::size_t> nractive;
};
}

bool TrajectoryExecutionManager::findControllers(const std::set<std::string> &actuated_joints, std::size_t controller_count, const std::vector<std::string> &available_controllers, std::vector<std::string> &selected_controllers)
{
  // generate all combinations of controller_count controllers that operate on disjoint sets of joints
  std::vector<std::string> work_area;
  OrderPotentialControllerCombination order;
  std::vector< std::vector<std::string> > &selected_options = order.selected_options;
  generateControllerCombination(0, controller_count, available_controllers, work_area, selected_options, actuated_joints);

  if (verbose_)
  {
    std::stringstream saj;
    std::stringstream sac;
    for (std::size_t i = 0 ; i < available_controllers.size() ; ++i)
      sac << available_controllers[i] << " ";
    for (std::set<std::string>::const_iterator it = actuated_joints.begin() ; it != actuated_joints.end() ; ++it)
      saj << *it << " ";
    ROS_INFO("Looking for %zu controllers among [ %s] that cover joints [ %s]. Found %zd options.", controller_count, sac.str().c_str(), saj.str().c_str(), selected_options.size());
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

  // count how many default controllers are used in each reported option, and how many joints are actuated in total by the selected controllers,
  // to use that in the ranking of the options
  order.nrdefault.resize(selected_options.size(), 0);
  order.nrjoints.resize(selected_options.size(), 0);
  order.nractive.resize(selected_options.size(), 0);
  for (std::size_t i = 0 ; i < selected_options.size() ; ++i)
  {
    for (std::size_t k = 0 ; k < selected_options[i].size() ; ++k)
    {
      updateControllerState(selected_options[i][k], DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
      const ControllerInformation &ci = known_controllers_[selected_options[i][k]];

      if (ci.state_.default_)
        order.nrdefault[i]++;
      if (ci.state_.active_)
        order.nractive[i]++;
      order.nrjoints[i] += ci.joints_.size();
    }
  }

  // define a bijection to compute the raking of the found options
  std::vector<std::size_t> bijection(selected_options.size(), 0);
  for (std::size_t i = 0 ; i < selected_options.size() ; ++i)
    bijection[i] = i;

  // sort the options
  std::sort(bijection.begin(), bijection.end(), order);

  // depending on whether we are allowed to load & unload controllers,
  // we have different preference on deciding between options
  if (!manage_controllers_)
  {
    // if we can't load different options at will, just choose one that is already loaded
    for (std::size_t i = 0 ; i < selected_options.size() ; ++i)
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

bool TrajectoryExecutionManager::isControllerActive(const std::string &controller)
{
  return areControllersActive(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::areControllersActive(const std::vector<std::string> &controllers)
{
  for (std::size_t i = 0 ; i < controllers.size() ; ++i)
  {
    updateControllerState(controllers[i], DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controllers[i]);
    if (it == known_controllers_.end() || !it->second.state_.active_)
      return false;
  }
  return true;
}

bool TrajectoryExecutionManager::selectControllers(const std::set<std::string> &actuated_joints, const std::vector<std::string> &available_controllers, std::vector<std::string> &selected_controllers)
{
  for (std::size_t i = 1 ; i <= available_controllers.size() ; ++i)
    if (findControllers(actuated_joints, i, available_controllers, selected_controllers))
    {
      // if we are not managing controllers, prefer to use active controllers even if there are more of them
      if (!manage_controllers_ && !areControllersActive(selected_controllers))
      {
        std::vector<std::string> other_option;
        for (std::size_t j = i + 1 ; j <= available_controllers.size() ; ++j)
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

bool TrajectoryExecutionManager::distributeTrajectory(const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers, std::vector<moveit_msgs::RobotTrajectory> &parts)
{
  parts.clear();
  parts.resize(controllers.size());

  std::set<std::string> actuated_joints_mdof;
  actuated_joints_mdof.insert(trajectory.multi_dof_joint_trajectory.joint_names.begin(),
                              trajectory.multi_dof_joint_trajectory.joint_names.end());
  std::set<std::string> actuated_joints_single;
  for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
  {
    const robot_model::JointModel *jm = robot_model_->getJointModel(trajectory.joint_trajectory.joint_names[i]);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != NULL || jm->getType() == robot_model::JointModel::FIXED)
        continue;
      actuated_joints_single.insert(jm->getName());
    }
  }

  for (std::size_t i = 0 ; i < controllers.size() ; ++i)
  {
    std::map<std::string, ControllerInformation>::iterator it = known_controllers_.find(controllers[i]);
    if (it == known_controllers_.end())
    {
      ROS_ERROR_STREAM("Controller " << controllers[i] << " not found.");
      return false;
    }
    std::vector<std::string> intersect_mdof;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(),
                          actuated_joints_mdof.begin(), actuated_joints_mdof.end(),
                          std::back_inserter(intersect_mdof));
    std::vector<std::string> intersect_single;
    std::set_intersection(it->second.joints_.begin(), it->second.joints_.end(),
                          actuated_joints_single.begin(), actuated_joints_single.end(),
                          std::back_inserter(intersect_single));
    if (intersect_mdof.empty() && intersect_single.empty())
      ROS_WARN_STREAM("No joints to be distributed for controller " << controllers[i]);
    {
      if (!intersect_mdof.empty())
      {
        std::vector<std::string> &jnames = parts[i].multi_dof_joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_mdof.begin(), intersect_mdof.end());
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0 ; j < trajectory.multi_dof_joint_trajectory.joint_names.size() ; ++j)
          index[trajectory.multi_dof_joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0 ; j < jnames.size() ; ++j)
          bijection[j] = index[jnames[j]];

        parts[i].multi_dof_joint_trajectory.points.resize(trajectory.multi_dof_joint_trajectory.points.size());
        for (std::size_t j = 0 ; j < trajectory.multi_dof_joint_trajectory.points.size() ; ++j)
        {
          parts[i].multi_dof_joint_trajectory.points[j].time_from_start = trajectory.multi_dof_joint_trajectory.points[j].time_from_start;
          parts[i].multi_dof_joint_trajectory.points[j].transforms.resize(bijection.size());
          for (std::size_t k = 0 ; k < bijection.size() ; ++k)
            parts[i].multi_dof_joint_trajectory.points[j].transforms[k] = trajectory.multi_dof_joint_trajectory.points[j].transforms[bijection[k]];
        }
      }
      if (!intersect_single.empty())
      {
        std::vector<std::string> &jnames = parts[i].joint_trajectory.joint_names;
        jnames.insert(jnames.end(), intersect_single.begin(), intersect_single.end());
        parts[i].joint_trajectory.header = trajectory.joint_trajectory.header;
        std::map<std::string, std::size_t> index;
        for (std::size_t j = 0 ; j < trajectory.joint_trajectory.joint_names.size() ; ++j)
          index[trajectory.joint_trajectory.joint_names[j]] = j;
        std::vector<std::size_t> bijection(jnames.size());
        for (std::size_t j = 0 ; j < jnames.size() ; ++j)
          bijection[j] = index[jnames[j]];
        parts[i].joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        for (std::size_t j = 0 ; j < trajectory.joint_trajectory.points.size() ; ++j)
        {
          parts[i].joint_trajectory.points[j].time_from_start = trajectory.joint_trajectory.points[j].time_from_start;
          if (!trajectory.joint_trajectory.points[j].positions.empty())
          {
            parts[i].joint_trajectory.points[j].positions.resize(bijection.size());
            for (std::size_t k = 0 ; k < bijection.size() ; ++k)
              parts[i].joint_trajectory.points[j].positions[k] = trajectory.joint_trajectory.points[j].positions[bijection[k]];
          }
          if (!trajectory.joint_trajectory.points[j].velocities.empty())
          {
            parts[i].joint_trajectory.points[j].velocities.resize(bijection.size());
            for (std::size_t k = 0 ; k < bijection.size() ; ++k)
              parts[i].joint_trajectory.points[j].velocities[k] = trajectory.joint_trajectory.points[j].velocities[bijection[k]] * execution_velocity_scaling_;
          }
          if (!trajectory.joint_trajectory.points[j].accelerations.empty())
          {
            parts[i].joint_trajectory.points[j].accelerations.resize(bijection.size());
            for (std::size_t k = 0 ; k < bijection.size() ; ++k)
              parts[i].joint_trajectory.points[j].accelerations[k] = trajectory.joint_trajectory.points[j].accelerations[bijection[k]];
          }
          if (!trajectory.joint_trajectory.points[j].effort.empty())
          {
            parts[i].joint_trajectory.points[j].effort.resize(bijection.size());
            for (std::size_t k = 0 ; k < bijection.size() ; ++k)
              parts[i].joint_trajectory.points[j].effort[k] = trajectory.joint_trajectory.points[j].effort[bijection[k]];
          }
        }
      }
    }
  }
  return true;
}

bool TrajectoryExecutionManager::configure(TrajectoryExecutionContext &context, const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers)
{
  if (trajectory.multi_dof_joint_trajectory.points.empty() &&  trajectory.joint_trajectory.points.empty())
  {
    ROS_WARN("The trajectory to execute is empty");
    return false;
  }
  std::set<std::string> actuated_joints;
  actuated_joints.insert(trajectory.multi_dof_joint_trajectory.joint_names.begin(),
                         trajectory.multi_dof_joint_trajectory.joint_names.end());
  actuated_joints.insert(trajectory.joint_trajectory.joint_names.begin(),
                         trajectory.joint_trajectory.joint_names.end());
  if (actuated_joints.empty())
  {
    ROS_WARN("The trajectory to execute specifies no joints");
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
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
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
    for (std::size_t i = 0 ; i < controllers.size() ; ++i)
      if (known_controllers_.find(controllers[i]) == known_controllers_.end())
      {
        reloadControllerInformation();
        reloaded = true;
        break;
      }
    if (reloaded)
      for (std::size_t i = 0 ; i < controllers.size() ; ++i)
        if (known_controllers_.find(controllers[i]) == known_controllers_.end())
        {
          ROS_ERROR("Controller '%s' is not known", controllers[i].c_str());
          return false;
        }
    if (selectControllers(actuated_joints, controllers, context.controllers_))
    {
      if (distributeTrajectory(trajectory, context.controllers_, context.trajectory_parts_))
        return true;
    }
  }
  std::stringstream ss;
  for (std::set<std::string>::const_iterator it = actuated_joints.begin() ; it != actuated_joints.end() ; ++it)
    ss << *it << " ";
  ROS_ERROR("Unable to identify any set of controllers that can actuate the specified joints: [ %s]", ss.str().c_str());

  std::stringstream ss2;
  std::map<std::string, ControllerInformation>::const_iterator mi;
  for(mi = known_controllers_.begin(); mi != known_controllers_.end(); mi++)
  {
    ss2 << "controller '" << mi->second.name_ << "' controls joints:\n";

    std::set<std::string>::const_iterator ji;
    for(ji = mi->second.joints_.begin(); ji != mi->second.joints_.end(); ji++)
    {
      ss2 << "  " << *ji << std::endl;
    }
  }
  ROS_ERROR("Known controllers and their joints:\n%s", ss2.str().c_str());
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
  for (std::size_t i = 0 ; i < active_handles_.size() ; ++i)
    try
    {
      active_handles_[i]->cancelExecution();
    }
    catch(...)
    {
      ROS_ERROR("Exception caught when canceling execution.");
    }
}

void TrajectoryExecutionManager::stopExecution(bool auto_clear)
{
  stop_continuous_execution_ = true;
  continuous_execution_condition_.notify_all();

  if (!execution_complete_)
  {
    execution_state_mutex_.lock();
    if (!execution_complete_)
    {
      // we call cancel for all active handles; we know these are not being modified as we loop through them because of the lock
      // we mark execution_complete_ as true ahead of time. Using this flag, executePart() will know that an external trigger to stop has been received
      execution_complete_ = true;
      stopExecutionInternal();

      // we set the status here; executePart() will not set status when execution_complete_ is true ahead of time
      last_execution_status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      execution_state_mutex_.unlock();
      ROS_INFO("Stopped trajectory execution.");

      // wait for the execution thread to finish
      execution_thread_->join();
      execution_thread_.reset();

      if (auto_clear)
        clear();
    }
    else
      execution_state_mutex_.unlock();
  }
  else
    if (execution_thread_) // just in case we have some thread waiting to be joined from some point in the past, we join it now
    {
      execution_thread_->join();
      execution_thread_.reset();
    }
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback &callback, bool auto_clear)
{
  execute(callback, PathSegmentCompleteCallback(), auto_clear);
}

void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback &callback, const PathSegmentCompleteCallback &part_callback, bool auto_clear)
{
  stopExecution(false);
  execution_complete_ = false;
  // start the execution thread
  execution_thread_.reset(new boost::thread(&TrajectoryExecutionManager::executeThread, this, callback, part_callback, auto_clear));
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::waitForExecution()
{
  {
    boost::unique_lock<boost::mutex> ulock(execution_state_mutex_);
    while (!execution_complete_)
      execution_complete_condition_.wait(ulock);
  }
  {
    boost::unique_lock<boost::mutex> ulock(continuous_execution_mutex_);
    while (!continuous_execution_queue_.empty())
      continuous_execution_condition_.wait(ulock);
  }

  // this will join the thread for executing sequences of trajectories
  stopExecution(false);

  return last_execution_status_;
}

void TrajectoryExecutionManager::clear()
{
  if (execution_complete_)
  {
    for (std::size_t i = 0 ; i < trajectories_.size() ; ++i)
      delete trajectories_[i];
    trajectories_.clear();
    {
      boost::mutex::scoped_lock slock(continuous_execution_mutex_);
      while (!continuous_execution_queue_.empty())
      {
        delete continuous_execution_queue_.front();
        continuous_execution_queue_.pop_front();
      }
    }
  }
  else
    ROS_ERROR("Cannot push a new trajectory while another is being executed");
}

void TrajectoryExecutionManager::executeThread(const ExecutionCompleteCallback &callback, const PathSegmentCompleteCallback &part_callback, bool auto_clear)
{
  // if we already got a stop request before we even started anything, we abort
  if (execution_complete_)
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (callback)
      callback(last_execution_status_);
    return;
  }

  ROS_DEBUG("Starting trajectory execution ...");
  // assume everything will be OK
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;

  // execute each trajectory, one after the other (executePart() is blocking) or until one fails.
  // on failure, the status is set by executePart(). Otherwise, it will remain as set above (success)
  for (std::size_t i = 0 ; i < trajectories_.size() ; ++i)
  {
    bool epart = executePart(i);
    if (epart && part_callback)
      part_callback(i);
    if (!epart || execution_complete_)
      break;
  }

  ROS_DEBUG("Completed trajectory execution with status %s ...", last_execution_status_.asString().c_str());

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
  TrajectoryExecutionContext &context = *trajectories_[part_index];

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
        for (std::size_t i = 0 ; i < context.controllers_.size() ; ++i)
        {
          moveit_controller_manager::MoveItControllerHandlePtr h;
          try
          {
            h = controller_manager_->getControllerHandle(context.controllers_[i]);
          }
          catch(...)
          {
            ROS_ERROR("Exception caught when retrieving controller handle");
          }
          if (!h)
          {
            active_handles_.clear();
            current_context_ = -1;
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            ROS_ERROR("No controller handle for controller '%s'. Aborting.", context.controllers_[i].c_str());
            return false;
          }
          active_handles_[i] = h;
        }
        handles = active_handles_; // keep a copy for later, to avoid thread safety issues
        for (std::size_t i = 0 ; i < context.trajectory_parts_.size() ; ++i)
        {
          bool ok = false;
          try
          {
            ok = active_handles_[i]->sendTrajectory(context.trajectory_parts_[i]);
          }
          catch(...)
          {
            ROS_ERROR("Exception caught when sending trajectory to controller");
          }
          if (!ok)
          {
            for (std::size_t j = 0 ; j < i ; ++j)
              try
              {
                active_handles_[j]->cancelExecution();
              }
              catch(...)
              {
                ROS_ERROR("Exception caught when canceling execution");
              }
            ROS_ERROR("Failed to send trajectory part %zu of %zu to controller %s", i + 1, context.trajectory_parts_.size(), active_handles_[i]->getName().c_str());
            if (i > 0)
              ROS_ERROR("Cancelling previously sent trajectory parts");
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
    for (std::size_t i = 0 ; i < context.trajectory_parts_.size() ; ++i)
    {
      ros::Duration d(0.0);
      if (!context.trajectory_parts_[i].joint_trajectory.points.empty())
      {
        if (context.trajectory_parts_[i].joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[i].joint_trajectory.header.stamp - current_time;
        if (context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp > current_time)
          d = std::max(d, context.trajectory_parts_[i].multi_dof_joint_trajectory.header.stamp - current_time);
        d += std::max(context.trajectory_parts_[i].joint_trajectory.points.empty() ? ros::Duration(0.0) :
                      context.trajectory_parts_[i].joint_trajectory.points.back().time_from_start,
                      context.trajectory_parts_[i].multi_dof_joint_trajectory.points.empty() ? ros::Duration(0.0) :
                      context.trajectory_parts_[i].multi_dof_joint_trajectory.points.back().time_from_start);

        if (longest_part < 0 ||
            std::max(context.trajectory_parts_[i].joint_trajectory.points.size(),
                     context.trajectory_parts_[i].multi_dof_joint_trajectory.points.size()) >
            std::max(context.trajectory_parts_[longest_part].joint_trajectory.points.size(),
                     context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size()))
          longest_part = i;
      }
      expected_trajectory_duration = std::max(d, expected_trajectory_duration);
    }
    // add 10% + 0.5s to the expected duration; this is just to allow things to finish propery

    expected_trajectory_duration = expected_trajectory_duration * allowed_execution_duration_scaling_ + ros::Duration(allowed_goal_duration_margin_);

    if (longest_part >= 0)
    {
      boost::mutex::scoped_lock slock(time_index_mutex_);

      // construct a map from expected time to state index, for easy access to expected state location
      if (context.trajectory_parts_[longest_part].joint_trajectory.points.size() >= context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size())
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].joint_trajectory.header.stamp - current_time;
        for (std::size_t j = 0 ; j < context.trajectory_parts_[longest_part].joint_trajectory.points.size() ; ++j)
          time_index_.push_back(current_time + d + context.trajectory_parts_[longest_part].joint_trajectory.points[j].time_from_start);
      }
      else
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp - current_time;
        for (std::size_t j = 0 ; j < context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size() ; ++j)
          time_index_.push_back(current_time + d + context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points[j].time_from_start);
      }
    }

    bool result = true;
    for (std::size_t i = 0 ; i < handles.size() ; ++i)
    {
      if (execution_duration_monitoring_)
      {
        if (!handles[i]->waitForExecution(expected_trajectory_duration))
          if (!execution_complete_ && ros::Time::now() - current_time > expected_trajectory_duration)
          {
            ROS_ERROR("Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was %lf seconds). Stopping trajectory.", expected_trajectory_duration.toSec());
            {
              boost::mutex::scoped_lock slock(execution_state_mutex_);
              stopExecutionInternal(); // this is trally tricky. we can't call stopExecution() here, so we call the internal function only
            }
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::TIMED_OUT;
            result = false;
            break;
          }
      }
      else
        handles[i]->waitForExecution();

      // if something made the trajectory stop, we stop this thread too
      if (execution_complete_)
      {
        result = false;
        break;
      }
      else
        if (handles[i]->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        {
          ROS_WARN_STREAM("Controller handle " << handles[i]->getName() << " reports status "
            << handles[i]->getLastExecutionStatus().asString());
          last_execution_status_ = handles[i]->getLastExecutionStatus();
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

std::pair<int, int> TrajectoryExecutionManager::getCurrentExpectedTrajectoryIndex() const
{
  boost::mutex::scoped_lock slock(time_index_mutex_);
  if (current_context_ < 0)
    return std::make_pair(-1, -1);
  if (time_index_.empty())
    return std::make_pair((int)current_context_, -1);
  std::vector<ros::Time>::const_iterator it = std::lower_bound(time_index_.begin(), time_index_.end(), ros::Time::now());
  int pos = it - time_index_.begin();
  return std::make_pair((int)current_context_, pos);
}

const std::vector<TrajectoryExecutionManager::TrajectoryExecutionContext*>& TrajectoryExecutionManager::getTrajectories() const
{
  return trajectories_;
}

moveit_controller_manager::ExecutionStatus TrajectoryExecutionManager::getLastExecutionStatus() const
{
  return last_execution_status_;
}

bool TrajectoryExecutionManager::ensureActiveControllersForGroup(const std::string &group)
{
  const robot_model::JointModelGroup *joint_model_group = robot_model_->getJointModelGroup(group);
  if (joint_model_group)
    return ensureActiveControllersForJoints(joint_model_group->getJointModelNames());
  else
    return false;
}

bool TrajectoryExecutionManager::ensureActiveControllersForJoints(const std::vector<std::string> &joints)
{
  std::vector<std::string> all_controller_names;
  for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
    all_controller_names.push_back(it->first);
  std::vector<std::string> selected_controllers;
  std::set<std::string> jset;
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
  {
    const robot_model::JointModel *jm = robot_model_->getJointModel(joints[i]);
    if (jm)
    {
      if (jm->isPassive() || jm->getMimic() != NULL || jm->getType() == robot_model::JointModel::FIXED)
        continue;
      jset.insert(joints[i]);
    }
  }
  
  if (selectControllers(jset, all_controller_names, selected_controllers))
    return ensureActiveControllers(selected_controllers);
  else
    return false;
}

bool TrajectoryExecutionManager::ensureActiveController(const std::string &controller)
{
  return ensureActiveControllers(std::vector<std::string>(1, controller));
}

bool TrajectoryExecutionManager::ensureActiveControllers(const std::vector<std::string> &controllers)
{
  updateControllersState(DEFAULT_CONTROLLER_INFORMATION_VALIDITY_AGE);

  if (manage_controllers_)
  {
    std::vector<std::string> controllers_to_activate;
    std::vector<std::string> controllers_to_deactivate;
    std::set<std::string> joints_to_be_activated;
    std::set<std::string> joints_to_be_deactivated;
    for (std::size_t i = 0 ; i < controllers.size() ; ++i)
    {
      std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.find(controllers[i]);
      if (it == known_controllers_.end())
      {
        ROS_ERROR_STREAM("Controller " << controllers[i] << " is not known");
        return false;
      }
      if (!it->second.state_.active_)
      {
        ROS_DEBUG_STREAM("Need to activate " << controllers[i]);
        controllers_to_activate.push_back(controllers[i]);
        joints_to_be_activated.insert(it->second.joints_.begin(), it->second.joints_.end());
        for (std::set<std::string>::iterator kt = it->second.overlapping_controllers_.begin() ;
             kt != it->second.overlapping_controllers_.end() ; ++kt)
        {
          const ControllerInformation &ci = known_controllers_[*kt];
          if (ci.state_.active_)
          {
            controllers_to_deactivate.push_back(*kt);
            joints_to_be_deactivated.insert(ci.joints_.begin(), ci.joints_.end());
          }
        }
      }
      else
        ROS_DEBUG_STREAM("Controller " << controllers[i] << " is already active");
    }
    std::set<std::string> diff;
    std::set_difference(joints_to_be_deactivated.begin(), joints_to_be_deactivated.end(),
                        joints_to_be_activated.begin(), joints_to_be_activated.end(),
                        std::inserter(diff, diff.end()));
    if (!diff.empty())
    {
      // find the set of controllers that do not overlap with the ones we want to activate so far
      std::vector<std::string> possible_additional_controllers;
      for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
      {
        bool ok = true;
        for (std::size_t k = 0 ; k < controllers_to_activate.size() ; ++k)
          if (it->second.overlapping_controllers_.find(controllers_to_activate[k]) != it->second.overlapping_controllers_.end())
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
        controllers_to_activate.insert(controllers_to_activate.end(), additional_controllers.begin(), additional_controllers.end());
      else
        return false;
    }
    if (!controllers_to_activate.empty() || !controllers_to_deactivate.empty())
    {
      if (controller_manager_)
      {
        // load controllers to be activated, if needed, and reset the state update cache
        for (std::size_t a = 0 ; a < controllers_to_activate.size() ; ++a)
        {
          ControllerInformation &ci = known_controllers_[controllers_to_activate[a]];
          ci.last_update_ = ros::Time();
        }
        // reset the state update cache
        for (std::size_t a = 0 ; a < controllers_to_deactivate.size() ; ++a)
          known_controllers_[controllers_to_deactivate[a]].last_update_ = ros::Time();
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
    for (std::map<std::string, ControllerInformation>::const_iterator it = known_controllers_.begin() ; it != known_controllers_.end() ; ++it)
      if (it->second.state_.active_)
        originally_active.insert(it->first);
    return std::includes(originally_active.begin(), originally_active.end(), controllers.begin(), controllers.end());
  }
}

}
