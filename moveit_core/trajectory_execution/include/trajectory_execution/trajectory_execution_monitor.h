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

#ifndef _TRAJECTORY_EXECUTION_MONITOR_H_
#define _TRAJECTORY_EXECUTION_MONITOR_H_

#include <ros/ros.h>
#include <boost/function.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_execution/trajectory_recorder.h>
#include <trajectory_execution/trajectory_controller_handler.h>
#include <planning_models/kinematic_model.h>

namespace trajectory_execution
{

/// \brief Collection of information required for requesting the execution of a trajectory.
struct TrajectoryExecutionRequest {
  
  std::string group_name_;
  std::string controller_name_;
  std::string ns_name_;
  std::string recorder_name_;

  bool monitor_overshoot_;
  double max_overshoot_velocity_epsilon_;
  ros::Duration min_overshoot_time_;
  ros::Duration max_overshoot_time_;

  bool failure_ok_;
  bool test_for_close_enough_;
  double max_joint_distance_;
  double failure_time_factor_;
  
  trajectory_msgs::JointTrajectory trajectory_;
  boost::function<void(const std::string& group_name)> callback_function_;

  TrajectoryExecutionRequest();
};

/// \brief Return code indicating the state of the executed trajectory.
enum TrajectoryExecutionResult {
  NOT_ATTEMPTED = 0,
  SUCCEEDED,
  NO_RECORDER,
  NO_HANDLER,
  ALREADY_AT_GOAL,
  HANDLER_FAILED_ENTIRELY,
  HANDLER_REPORTS_FAILURE,
  HANDLER_REPORTS_FAILURE_BUT_OK,
  HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH
};

/// \brief Data that is filled in by the trajectory execution monitor.
struct TrajectoryExecutionData {

  /// \brief Error code
  TrajectoryExecutionResult result_;

  /// \brief Trajectory recorded during execution.
  trajectory_msgs::JointTrajectory recorded_trajectory_;
  /// \brief Trajectory recorded after the controller has declared the execution finished,
  /// but we continue to record the overshoot until the group stops moving.
  trajectory_msgs::JointTrajectory overshoot_trajectory_;

  /// \brief The duration of the recorded trajectory.
  ros::Duration time_;
  /// \brief The duration of the overshoot trajectory.
  ros::Duration overshoot_time_;
  /// \brief The angular distance of the recorded trajectory.
  /// See TrajectoryStats::distance() for more information.
  double angular_distance_;					// recorded
};

/// \brief Convenience structure to hold multiple executed trajectories
struct TrajectoryExecutionDataVector : public std::vector<TrajectoryExecutionData> 
{

  void reset() {
    clear();
    last_attempted_trajectory_index_ = 0;
  }

  unsigned int last_attempted_trajectory_index_;
};

/// \brief The function signature for callbacks executed at the completion og a trajectory
typedef boost::function<bool(TrajectoryExecutionDataVector)> ExecutionCompleteCallbackFn;

/// \brief Executes and monitors a set of trajectories.
class TrajectoryExecutionMonitor
{

 public:
  
 TrajectoryExecutionMonitor(const planning_models::KinematicModelConstPtr& kmodel,
			    bool manage_controllers=true) 
   : kmodel_(kmodel), 
     manage_controllers_(manage_controllers)
{};

  virtual ~TrajectoryExecutionMonitor() {};

  /// \brief Add a recorder to a list of recorders
  void addTrajectoryRecorder(boost::shared_ptr<TrajectoryRecorder>& trajectory_recorder);

  /// \brief Add a controller to a list of controllers
  void addTrajectoryControllerHandler(boost::shared_ptr<TrajectoryControllerHandler>& trajectory_controller_handler,
                                      bool is_default);

  /// \brief Execute a series of trajectories, in order.
  /// The callbacks will get called, in order, after each trajectory is finished executing.
  bool executeTrajectories(const std::vector<TrajectoryExecutionRequest>& to_execute,
                           const ExecutionCompleteCallbackFn& done_callback = ExecutionCompleteCallbackFn());

  /// \brief Execute one trajectory (calls executeTrajectories())
  bool executeTrajectory(const TrajectoryExecutionRequest& to_execute,
                         const ExecutionCompleteCallbackFn& done_callback = ExecutionCompleteCallbackFn())
  {
    std::vector<TrajectoryExecutionRequest> to_execute_v(1, to_execute);
    return executeTrajectories(to_execute_v, done_callback);
  }
  
  void switchAssociatedStopStartControllers(const std::string& group_name,
                                            const std::string& desired_controller);

  std::string getCurrentController(const std::string& group_name) const;

  std::string getDefaultControllerName(const std::string& group_name) const {
    std::map<std::string, boost::shared_ptr<TrajectoryControllerHandler> >::const_iterator it =
      default_trajectory_controller_handler_map_.find(group_name);    
    if(it != default_trajectory_controller_handler_map_.end())
      if (it->second)
        return it->second->getControllerName();
    return std::string("");
  }

  const std::map<std::string, bool>& getOriginalControllerConfiguration() const {
    return original_controller_configuration_map_;
  }

  virtual void loadController(const std::string& name) = 0;
  
  virtual void unloadController(const std::string& name) = 0;

  virtual bool getRunningControllerMap(std::map<std::string, bool>& controller_map) = 0;

  virtual void restoreOriginalControllers() = 0;

protected:
  
  virtual void switchControllers(const std::vector<std::string>& off_controllers,
                                 const std::vector<std::string>& on_controllers) = 0;
  
  bool sendTrajectory(const TrajectoryExecutionRequest& ter);
  
  /// \brief Gets called after the execution of a trajectory.
  void trajectoryFinishedCallbackFunction(TrajectoryControllerState controller_state);
                                    
  /// \brief Returns true if the executed trajectory endpoint is close to intended trajectory endpoint.
  bool closeEnough(const TrajectoryExecutionRequest& ter,
                   const TrajectoryExecutionData& ted);
  
  /// \brief Compare recorded trajectory to expected trajectory.
  void compareLastRecordedToStart(const TrajectoryExecutionRequest& last_ter,
                                  const TrajectoryExecutionRequest& next_ter,
                                  const TrajectoryExecutionData& ted);


  ExecutionCompleteCallbackFn result_callback_;
  TrajectoryExecutionDataVector execution_result_vector_;
  std::vector<TrajectoryExecutionRequest> execution_data_;
  unsigned int current_trajectory_index_;

  boost::shared_ptr<TrajectoryControllerHandler> last_requested_handler_;
  std::map<std::string, boost::shared_ptr<TrajectoryRecorder> > trajectory_recorder_map_;
  //map by combo_name
  std::map<std::string, boost::shared_ptr<TrajectoryControllerHandler> > trajectory_controller_handler_map_;
  
  //map by group_name
  std::map<std::string, boost::shared_ptr<TrajectoryControllerHandler> > default_trajectory_controller_handler_map_;

  std::vector<std::string> loaded_controllers_;

  //map from group to possible controllers to whether or not we have a controller handler
  std::map<std::string, std::map<std::string, bool> > group_possible_controllers_map_;

  //map from controllers to groups, mostly for preclusion
  std::map<std::string, std::map<std::string, bool> > controller_possible_group_map_;

  std::map<std::string, bool> original_controller_configuration_map_;
  std::map<std::string, std::string> group_default_controllers_;
  std::map<std::string, std::string> controllers_default_group_;

  //map from group to running controller
  std::map<std::string, std::string> current_group_controller_name_map_;

  //map from running controller to associated group
  std::map<std::string, std::string> current_controller_group_name_map_;

  planning_models::KinematicModelConstPtr kmodel_;
  bool manage_controllers_;
};

}

#endif
