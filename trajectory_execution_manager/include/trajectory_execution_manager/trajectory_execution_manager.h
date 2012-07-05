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

#ifndef MOVEIT_TRAJECTORY_EXECUTION_MANAGER_TRAJECTORY_EXECUTION_MANAGER_
#define MOVEIT_TRAJECTORY_EXECUTION_MANAGER_TRAJECTORY_EXECUTION_MANAGER_

#include <planning_models/kinematic_model.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <moveit_controller_manager/moveit_controller_manager.h>
#include <boost/thread.hpp>
#include <pluginlib/class_loader.h>

namespace trajectory_execution_manager
{

// Two modes:
// Managed controllers
// Unmanaged controllers: given the trajectory, 
class TrajectoryExecutionManager
{
public:
  
  typedef boost::function<void(bool success)> ExecutionCompleteCallback;
  
  /// load the controller manager plugin
  /// Start listening for events on a topic.

  TrajectoryExecutionManager(const planning_models::KinematicModelConstPtr &kmodel) : 
    kinematic_model_(kmodel), node_handle_("~")
  {
    if (!node_handle_.getParam("manage_controllers", manage_controllers_))
      manage_controllers_ = false;
    initialize();
  }
  
  TrajectoryExecutionManager(const planning_models::KinematicModelConstPtr &kmodel, bool manage_controllers) :
    kinematic_model_(kmodel), node_handle_("~"), manage_controllers_(manage_controllers)
  {
    initialize();
  }
  
  bool isManagingControllers(void) const;
  
  const moveit_controller_manager::MoveItControllerManagerPtr& getControllerManager(void);
  
  /** \brief Execute a named event */
  void processEvent(const std::string &event);

  /** \brief Make sure the active controllers are such that trajectories that actuate joints in the specified group can be executed.
      \note If manage_controllers_ is false and the controllers that happen to be active do not cover the joints in the group to be actuated, this function fails. */
  bool ensureActiveControllersForGroup(const std::string &group);
  
  /** \brief Make sure the active controllers are such that trajectories that actuate joints in the specified set can be executed.
      \note If manage_controllers_ is false and the controllers that happen to be active do not cover the joints to be actuated, this function fails. */
  bool ensureActiveControllersForJoints(const std::vector<std::string> &joints);
  
  /** \brief Make sure a particular controller is active.
      \note If manage_controllers_ is false and the controllers that happen to be active to not include the one specified as argument, this function fails. */
  bool ensureActiveController(const std::string &controller);
  
  /** \brief Make sure a particular set of controllers are active.
      \note If manage_controllers_ is false and the controllers that happen to be active to not include the ones specified as argument, this function fails. */
  bool ensureActiveControllers(const std::vector<std::string> &controllers);

  /// Add a trajectory for future execution. Optionally specify a controller to use for the trajectory. If no controller is specified, a default is used.
  bool push(const moveit_msgs::RobotTrajectory &trajectory, const std::string &controller = "");

  /// Add a trajectory for future execution. Optionally specify a controller to use for the trajectory. If no controller is specified, a default is used.
  bool push(const trajectory_msgs::JointTrajectory &trajectory, const std::string &controller = "");

  /// Add a trajectory for future execution. Optionally specify a set of controllers to consider using for the trajectory. Multiple controllers can be used simultaneously
  /// to execute the different parts of the trajectory. If multiple controllers can be used, preference is given to the already loaded ones.
  /// If no controller is specified, a default is used.
  bool push(const trajectory_msgs::JointTrajectory &trajectory, const std::vector<std::string> &controllers);

  /// Add a trajectory for future execution. Optionally specify a set of controllers to consider using for the trajectory. Multiple controllers can be used simultaneously
  /// to execute the different parts of the trajectory. If multiple controllers can be used, preference is given to the already loaded ones.
  /// If no controller is specified, a default is used.
  bool push(const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers);
  
  /// Start the execution of pushed trajectories; this does not wait for completion, but calls a callback when done.
  void execute(const ExecutionCompleteCallback &callback = ExecutionCompleteCallback(), bool auto_clear = true);
  
  /// Wait until the execution is complete
  bool waitForExecution(void);
  
  /// Return the controller status for the last attempted execution 
  moveit_controller_manager::ExecutionStatus::Value getLastExecutionStatus(void) const;
  
  // this is a blocking call for the execution of the passed in trajectories
  bool executeAndWait(bool auto_clear = true);
  
  /// Stop whatever executions are active, if any
  void stopExecution(bool auto_clear = true);
  
  /// Clear the trajectories to execute
  void clear(void);
  
private:

  struct TrajectoryExecutionContext
  {
    /// The controllers to use for executing the different trajectory parts; 
    std::vector<std::string> controllers_;
    
    // The trajectory to execute, split in different parts (by joints), each set of joints corresponding to one controller
    std::vector<moveit_msgs::RobotTrajectory> trajectory_parts_;
  };
  
  struct ControllerInformation
  {
    std::string name_;
    std::set<std::string> joints_;
    std::set<std::string> overlapping_controllers_;
    moveit_controller_manager::MoveItControllerManager::ControllerState state_;
    ros::Time last_update_;
    
    bool operator<(ControllerInformation &other) const
    {
      if (joints_.size() != other.joints_.size())
        return joints_.size() < other.joints_.size();
      return name_ < other.name_;
    }
  };

  void initialize(void);

  void reloadControllerInformation(void);

  bool configure(TrajectoryExecutionContext &context, const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers);
  
  void updateControllersState(const ros::Duration &age);
  void updateControllerState(const std::string &controller, const ros::Duration &age);
  void updateControllerState(ControllerInformation &ci, const ros::Duration &age);

  bool distributeTrajectory(const moveit_msgs::RobotTrajectory &trajectory, const std::vector<std::string> &controllers, std::vector<moveit_msgs::RobotTrajectory> &parts);
  
  bool findControllers(const std::set<std::string> &actuated_joints, std::size_t controller_count, const std::vector<std::string> &available_controllers, std::vector<std::string> &selected_controllers);
  bool checkControllerCombination(std::vector<std::string> &controllers, const std::set<std::string> &actuated_joints);
  void generateControllerCombination(std::size_t start_index, std::size_t controller_count, const std::vector<std::string> &available_controllers, 
                                     std::vector<std::string> &selected_controllers, std::vector< std::vector<std::string> > &selected_options,
                                     const std::set<std::string> &actuated_joints);
  bool selectControllers(const std::set<std::string> &actuated_joints, const std::vector<std::string> &available_controllers, std::vector<std::string> &selected_controllers);
  bool areControllersActive(const std::vector<std::string> &controllers);
  
  void executeThread(const ExecutionCompleteCallback &callback, bool auto_clear);
  bool executePart(TrajectoryExecutionContext &context);
  

  void receiveEvent(const std_msgs::StringConstPtr &event);

  planning_models::KinematicModelConstPtr kinematic_model_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle root_node_handle_;
  ros::Subscriber event_topic_subscriber_;
  
  std::map<std::string, ControllerInformation> known_controllers_;
  bool manage_controllers_;
  
  boost::scoped_ptr<boost::thread> execution_thread_;
  boost::mutex execution_state_mutex_;
  boost::condition_variable execution_complete_condition_;
  moveit_controller_manager::ExecutionStatus::Value last_execution_status_;
  std::vector<moveit_controller_manager::MoveItControllerHandlePtr> active_handles_;
  bool execution_complete_;
    
  std::vector<TrajectoryExecutionContext> trajectories_;

  boost::scoped_ptr<pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> > controller_manager_loader_;
  moveit_controller_manager::MoveItControllerManagerPtr controller_manager_;

  bool verbose_;
};

}

#endif
