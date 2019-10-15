/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Henning Kayser */

#include <stdexcept>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/common_planning_interface_objects/common_objects.h>

#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace moveit
{
namespace planning_interface
{
constexpr char LOGNAME[] = "moveit_cpp";
constexpr char PLANNING_SCENE_MONITOR_NAME[] = "moveit_cpp_planning_scene";
constexpr char PLANNING_PLUGIN_PARAM[] = "planning_plugin";

MoveItCpp::MoveItCpp(const ros::NodeHandle& nh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : MoveItCpp(Options(nh), nh, tf_buffer)
{
}

MoveItCpp::MoveItCpp(const Options& opt, const ros::NodeHandle& nh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  if (!tf_buffer_)
    tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // Configure planning scene monitor
  if (!loadPlanningSceneMonitor(opt.planning_scene_monitor_options))
  {
    std::string error = "Unable to configure planning scene monitor";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  robot_model_ = planning_scene_monitor_->getRobotModel();
  if (!robot_model_)
  {
    std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                        "parameter server.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  bool load_planning_pipelines = true;
  if (load_planning_pipelines && !loadPlanningPipelines(opt.planning_pipeline_options))
  {
    std::string error = "Failed to load planning pipelines from parameter server";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  // TODO(henningkayser): configure trajectory execution manager
  trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(
      robot_model_, planning_scene_monitor_->getStateMonitor()));

  ROS_INFO_NAMED(LOGNAME, "MoveItCpp running");
}

MoveItCpp::MoveItCpp(MoveItCpp&& other)
{
  other.clearContents();
}

MoveItCpp::~MoveItCpp()
{
  ROS_INFO_NAMED(LOGNAME, "Deleting MoveItCpp");
  clearContents();
}

MoveItCpp& MoveItCpp::operator=(MoveItCpp&& other)
{
  if (this != &other)
  {
    this->robot_description_ = other.robot_description_;
    this->node_handle_ = other.node_handle_;
    this->tf_buffer_ = other.tf_buffer_;
    this->robot_model_ = other.robot_model_;
    this->planning_scene_monitor_ = other.planning_scene_monitor_;
    other.clearContents();
  }

  return *this;
}

bool MoveItCpp::loadPlanningSceneMonitor(const PlanningSceneMonitorOptions& opt)
{
  planning_scene_monitor_.reset(
      new planning_scene_monitor::PlanningSceneMonitor(opt.robot_description, tf_buffer_, opt.name));
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Configuring Planning Scene Monitor");
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Start state and scene monitors
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s' for joint states", opt.joint_state_topic.c_str());
    planning_scene_monitor_->startStateMonitor(opt.joint_state_topic, opt.attached_collision_object_topic);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          opt.publish_planning_scene_topic);
    planning_scene_monitor_->startSceneMonitor(opt.monitored_planning_scene_topic);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
    return false;
  }
  // TODO(henningkayser): fix and remove lines below
  ros::spinOnce();
  ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  if (opt.wait_for_initial_state_timeout > 0.0)
  {
    return planning_scene_monitor_->getStateMonitor()->waitForCurrentState(ros::Time::now(),
                                                                           opt.wait_for_initial_state_timeout);
  }

  return true;
}

bool MoveItCpp::loadPlanningPipelines(const PlanningPipelineOptions& opt)
{
  ros::NodeHandle node_handle(opt.parent_namespace.empty() ? "~" : opt.parent_namespace);
  for (const auto& planning_pipeline_name : opt.pipeline_names)
  {
    if (planning_pipelines_.count(planning_pipeline_name) > 0)
    {
      ROS_WARN_NAMED(LOGNAME, "Skipping duplicate entry for planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    ROS_INFO_NAMED(LOGNAME, "Loading planning pipeline '%s'", planning_pipeline_name.c_str());
    ros::NodeHandle child_nh(node_handle, planning_pipeline_name);
    planning_pipeline::PlanningPipelinePtr pipeline;
    pipeline.reset(new planning_pipeline::PlanningPipeline(robot_model_, child_nh, PLANNING_PLUGIN_PARAM));

    if (!pipeline->getPlannerManager())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to initialize planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    planning_pipelines_[planning_pipeline_name] = pipeline;
  }

  if (planning_pipelines_.empty())
  {
    return false;
    ROS_ERROR_NAMED(LOGNAME, "Failed to load any planning pipelines.");
  }

  // Retrieve group/pipeline mapping for faster lookup
  std::vector<std::string> group_names = robot_model_->getJointModelGroupNames();
  for (const auto& pipeline_entry : planning_pipelines_)
  {
    for (const auto& group_name : group_names)
    {
      groups_pipelines_map_[group_name] = {};
      const auto& pipeline = pipeline_entry.second;
      for (const auto& planner_configuration : pipeline->getPlannerManager()->getPlannerConfigurations())
      {
        if (planner_configuration.second.group == group_name)
        {
          groups_pipelines_map_[group_name].insert(pipeline_entry.first);
        }
      }
    }
  }

  return true;
}

robot_model::RobotModelConstPtr MoveItCpp::getRobotModel() const
{
  ROS_DEBUG_NAMED(LOGNAME, "MoveItCpp::getRobotModel()");
  return robot_model_;
}

const ros::NodeHandle& MoveItCpp::getNodeHandle() const
{
  ROS_DEBUG_NAMED(LOGNAME, "MoveItCpp::getNodeHandle()");
  return node_handle_;
}

bool MoveItCpp::getCurrentState(robot_state::RobotStatePtr& current_state, double wait_seconds)
{
  if (wait_seconds > 0.0 &&
      !planning_scene_monitor_->getStateMonitor()->waitForCurrentState(ros::Time::now(), wait_seconds))
  {
    ROS_ERROR_NAMED(LOGNAME, "Did not receive robot state");
    return false;
  }
  {  // Lock planning scene
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    current_state.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // Unlock planning scene
  return true;
}

robot_state::RobotStatePtr MoveItCpp::getCurrentState(double wait)
{
  robot_state::RobotStatePtr current_state;
  getCurrentState(current_state, wait);
  return current_state;
}

const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& MoveItCpp::getPlanningPipelines() const
{
  return planning_pipelines_;
}

std::set<std::string> MoveItCpp::getPlanningPipelineNames(const std::string& group_name) const
{
  std::set<std::string> result_names;
  if (!group_name.empty() && groups_pipelines_map_.count(group_name) == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "There are no planning pipelines loaded for group '%s'.", group_name.c_str());
    return result_names;  // empty
  }
  for (const auto& pipeline_entry : planning_pipelines_)
  {
    const std::string& pipeline_name = pipeline_entry.first;
    // If group_name is defined and valid, skip pipelines that don't belong to the planning group
    if (!group_name.empty())
    {
      const auto& group_pipelines = groups_pipelines_map_.at(group_name);
      if (group_pipelines.find(pipeline_name) == group_pipelines.end())
        continue;
    }
    result_names.insert(pipeline_name);
  }
  return result_names;
}

const planning_scene_monitor::PlanningSceneMonitorPtr& MoveItCpp::getPlanningSceneMonitor() const
{
  return planning_scene_monitor_;
}
planning_scene_monitor::PlanningSceneMonitorPtr MoveItCpp::getPlanningSceneMonitorNonConst()
{
  return planning_scene_monitor_;
}

const trajectory_execution_manager::TrajectoryExecutionManagerPtr& MoveItCpp::getTrajectoryExecutionManager() const
{
  return trajectory_execution_manager_;
}

trajectory_execution_manager::TrajectoryExecutionManagerPtr MoveItCpp::getTrajectoryExecutionManagerNonConst()
{
  return trajectory_execution_manager_;
}

bool MoveItCpp::execute(const std::string& group_name, const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                        bool blocking)
{
  if (!robot_trajectory)
  {
    ROS_ERROR_NAMED(LOGNAME, "Robot trajectory is undefined");
    return false;
  }

  // Check if there are controllers that can handle the execution
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Execution failed! No active controllers configured for group '%s'", group_name.c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);
  if (blocking)
  {
    trajectory_execution_manager_->push(robot_trajectory_msg);
    trajectory_execution_manager_->execute();
    return trajectory_execution_manager_->waitForExecution();
  }
  trajectory_execution_manager_->pushAndExecute(robot_trajectory_msg);
  return true;
}

const std::shared_ptr<tf2_ros::Buffer>& MoveItCpp::getTFBuffer() const
{
  return tf_buffer_;
}

void MoveItCpp::clearContents()
{
  robot_description_.clear();
  tf_buffer_.reset();
  planning_scene_monitor_.reset();
  robot_model_.reset();
  planning_pipelines_.clear();
}
}  //  planning_interface
}  //  moveit
