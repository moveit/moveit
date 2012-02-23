/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/** \author E. Gil Jones, Ken Anderson */

#ifndef _TRAJECTORY_CONTROLLER_HANDLER_H_
#define _TRAJECTORY_CONTROLLER_HANDLER_H_

#include <ros/ros.h>
#include <boost/function.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_execution/trajectory_recorder.h>

namespace trajectory_execution
{

namespace TrajectoryControllerStates
{
  /// \brief Possible states the controller could be in during execution.
  enum TrajectoryControllerState
  {
    IDLE = 0,
    PAUSED,
    EXECUTING,
    OVERSHOOTING,
    SUCCESS,
    OVERSHOOT_TIMEOUT,	// usually considered a success
    EXECUTION_FAILURE,
    EXECUTION_TIMEOUT,
    CANCELLED
  };
}
typedef TrajectoryControllerStates::TrajectoryControllerState TrajectoryControllerState;

/// \brief Callback gets called when the controller is finished, or the controller has timed-out.
typedef boost::function<void(TrajectoryControllerState)> TrajectoryFinishedCallbackFunction;

/// \brief Sets up the controller for execution, handles the responses,
/// and times-out the controller if necessary.
class TrajectoryControllerHandler {

public:

  TrajectoryControllerHandler() {};

  virtual ~TrajectoryControllerHandler() {
  }

  virtual bool initialize(const std::string& group_name,
                          const std::string& controller_name,
                          const std::string& ns)
  {
    group_name_ = group_name;
    controller_name_ = controller_name;
    ns_name_ = ns;
    monitor_overshoot_ = false;
    controller_state_ = TrajectoryControllerStates::IDLE;
    timeout_ = ros::Duration(100);
    group_controller_ns_combo_name_ = combineGroupControllerNamespaceNames(group_name_,controller_name_, ns_name_);  
    return true;
  };

  static std::string combineGroupControllerNamespaceNames(const std::string& group_name,
                                                          const std::string& controller_name,
                                                          const std::string& ns_name) {
    return(group_name+"_"+controller_name+"_"+ns_name);
  }

  /// \brief This function is called if the trajectory should monitor overshoot (after the trajectory is executed).
  /// Returns true if the subclass can monitor overshoot
  bool enableOvershoot(double max_overshoot_velocity_epsilon,
                       ros::Duration min_overshoot_time,
                       ros::Duration max_overshoot_time);

  /// \brief Disable overshoot monitoring. Overshoot monitoring is disabled by default.
  void disableOvershoot();

  /// \brief Sets a maximum exection time, otherwise the default max execution time will be used
  void setMaximumExecutionTime( ros::Duration max_execution_time ) {
    timeout_ = max_execution_time;
  }

  /// \brief Execute the trajectory while recording using recorder.
  /// Call the callback function when finished.
  /// To enable overshoot monitoring, call enableOvershoot function.  Overshoot is not monitored by default.
  virtual bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                 boost::shared_ptr<TrajectoryRecorder>& recorder,
                                 const TrajectoryFinishedCallbackFunction& traj_callback) = 0;

  /// \brief cancel the execution of the trajectory. This should be implemented by derived classes.
  virtual void cancelExecution() = 0;

  /// \brief This function gets called when our maximum execution time is exceeded.
  void timeout(const ros::TimerEvent& event);

  const trajectory_msgs::JointTrajectory& getLastGoalTrajectory() const {
    return goal_trajectory_;
  }

  const trajectory_msgs::JointTrajectory& getLastRecordedTrajectory() const {
    return recorded_trajectory_;
  }

  const trajectory_msgs::JointTrajectory& getLastOvershootTrajectory() const {
    return overshoot_trajectory_;
  }

  const std::string& getGroupName() const {
    return group_name_;
  }

  const std::string& getControllerName() const {
    return controller_name_;
  }

  const std::string& getGroupControllerComboName() const {
    return group_controller_ns_combo_name_;
  }

protected:

  bool addNewStateToRecordedTrajectory(const ros::Time& time,
                                       const std::map<std::string, double>& joint_positions,
                                       const std::map<std::string, double>& joint_velocities);
  bool addNewStateToTrajectory(const ros::Time& time,
                               const std::map<std::string, double>& joint_positions,
                               const std::map<std::string, double>& joint_velocities,
                               trajectory_msgs::JointTrajectory& trajectory);

  /// \brief Deregisters from the recorder, and executes callback to the monitor.
  /// _success should be set before calling this function.
  void done();
  /// \brief Sets a flag to deregister when returning execution to the recorder (after its callback).
  /// This function should be called from within the TrajectoryFinishedCallbackFunction
  void doneDelayed();

  void initializeRecordedTrajectory(const trajectory_msgs::JointTrajectory& goal_trajectory);
  void initializeOvershootTrajectory();
  // returns the index of the first point who's time_from_start_ value is equal to or greater than time_from_start.
  unsigned int findClosestIndex( ros::Duration time_from_start );

  std::string group_name_;
  std::string controller_name_;
  std::string ns_name_;
  std::string group_controller_ns_combo_name_;

  trajectory_msgs::JointTrajectory recorded_trajectory_;
  trajectory_msgs::JointTrajectory overshoot_trajectory_;
  trajectory_msgs::JointTrajectory goal_trajectory_;

  bool monitor_overshoot_;
  double max_overshoot_velocity_epsilon_;
  ros::Duration min_overshoot_time_;
  ros::Duration max_overshoot_time_;

  boost::shared_ptr<trajectory_execution::TrajectoryRecorder> recorder_;
  trajectory_execution::TrajectoryFinishedCallbackFunction trajectory_finished_callback_;
  TrajectoryControllerState	controller_state_;

  // Used for timeout
  ros::Duration timeout_;
  ros::NodeHandle nh_;
  ros::Timer timer_;

  //TODO - consider pause and resume execution

};

}

#endif
