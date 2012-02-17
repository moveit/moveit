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

#include <trajectory_execution_monitor/trajectory_execution_monitor.h>
#include <trajectory_execution_monitor/trajectory_stats.h>
#include <planning_models/kinematic_state.h>

using namespace trajectory_execution_monitor;

TrajectoryExecutionRequest::TrajectoryExecutionRequest() :
  monitor_overshoot_(false),
  max_overshoot_velocity_epsilon_(20),
  min_overshoot_time_(ros::Duration(0.5)),
  max_overshoot_time_(ros::Duration(0.01)),
  failure_ok_(false),
  test_for_close_enough_(false),
  max_joint_distance_(0.01),
  failure_time_factor_(1.5)
{}

void TrajectoryExecutionMonitor::addTrajectoryRecorder(boost::shared_ptr<TrajectoryRecorder>& trajectory_recorder) {
  trajectory_recorder_map_[trajectory_recorder->getName()] = trajectory_recorder;
}

void TrajectoryExecutionMonitor::addTrajectoryControllerHandler(boost::shared_ptr<TrajectoryControllerHandler>& trajectory_controller_handler) {
  trajectory_controller_handler_map_[trajectory_controller_handler->getGroupControllerComboName()] = 
    trajectory_controller_handler;
}

void TrajectoryExecutionMonitor::executeTrajectories(const std::vector<TrajectoryExecutionRequest>& to_execute,
                                                     const boost::function<bool(TrajectoryExecutionDataVector)>& done_callback) {
  
  execution_data_ = &to_execute;
  execution_result_vector_.reset();
  result_callback_ = done_callback;
  
  current_trajectory_index_ = 0;
  
  if(!sendTrajectory((*execution_data_)[current_trajectory_index_])) {
    result_callback_(execution_result_vector_);
  }
};

bool TrajectoryExecutionMonitor::sendTrajectory(const TrajectoryExecutionRequest& ter) {
  
  execution_result_vector_.resize(execution_result_vector_.size()+1);
  
  std::string recorder_name = ter.recorder_name_;
  if(trajectory_recorder_map_.size() == 1) {
    recorder_name = trajectory_recorder_map_.begin()->second->getName();
  }
     
  if(trajectory_recorder_map_.find(ter.recorder_name_) == trajectory_recorder_map_.end() && 
     trajectory_recorder_map_.size() > 1) {
    execution_result_vector_.back().result_ = NO_RECORDER;
    return false;
  } 
  
  std::string combo_name = TrajectoryControllerHandler::combineGroupAndControllerNames(ter.group_name_,
                                                                                       ter.controller_name_);
  
  if(trajectory_controller_handler_map_.find(combo_name) == trajectory_controller_handler_map_.end()) {
    execution_result_vector_.back().result_ = NO_HANDLER;
    return false;
  }
  
  boost::shared_ptr<TrajectoryRecorder>& requested_recorder = trajectory_recorder_map_.find(recorder_name)->second;
  last_requested_handler_ = trajectory_controller_handler_map_.find(combo_name)->second;

  // Enable overshoot, if required
  if(ter.monitor_overshoot_)
  {
    last_requested_handler_->enableOvershoot(ter.max_overshoot_velocity_epsilon_, ter.min_overshoot_time_, ter.max_overshoot_time_);
  }
  else
  {
    last_requested_handler_->disableOvershoot();
  }

  // Set the max execution time
  trajectory_msgs::JointTrajectory traj = ter.trajectory_;
  ros::Duration traj_dur = TrajectoryStats::getDuration(traj);
  if( traj_dur > ros::Duration(0.1) )
  {
    last_requested_handler_->setMaximumExecutionTime(traj_dur * ter.failure_time_factor_);
  }

  traj.header.stamp = ros::Time::now();
  if(!last_requested_handler_->executeTrajectory(
                                                 traj,
                                                 requested_recorder,
                                                 boost::bind(&TrajectoryExecutionMonitor::trajectoryFinishedCallbackFunction, this, _1))
     )
  {
    execution_result_vector_.back().result_ = HANDLER_FAILED_ENTIRELY;
  }
  return true;
}

void TrajectoryExecutionMonitor::trajectoryFinishedCallbackFunction(
                                                                    TrajectoryControllerState controller_state )
{
  //adding this in any case
  execution_result_vector_.back().recorded_trajectory_ = last_requested_handler_->getLastRecordedTrajectory();
  execution_result_vector_.back().overshoot_trajectory_ = last_requested_handler_->getLastOvershootTrajectory();

  const TrajectoryExecutionRequest& req = (*execution_data_)[current_trajectory_index_];
  bool ok = (	controller_state==TrajectoryControllerStates::SUCCESS ||
                controller_state==TrajectoryControllerStates::OVERSHOOT_TIMEOUT);
  bool continue_execution =
    ok ||
    req.failure_ok_ ||
    (	req.test_for_close_enough_ && closeEnough(req,execution_result_vector_.back()));

  if(	continue_execution )
  {
    ROS_INFO_STREAM("Trajectory finished");

    // calculate stats
    TrajectoryExecutionData & data = execution_result_vector_.back();
    data.time_ = TrajectoryStats::getDuration(data.recorded_trajectory_);
    data.overshoot_time_ = TrajectoryStats::getDuration(data.overshoot_trajectory_);
    data.angular_distance_ = TrajectoryStats::getAngularDistance(data.recorded_trajectory_);

    if(!ok) {
      if(req.failure_ok_) {
        execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE_BUT_OK;
      } else {
        execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH;
      }
    } else {
      execution_result_vector_.back().result_ = SUCCEEDED;
    }
    if(req.callback_function_) {
      req.callback_function_(req.group_name_);
    }

    current_trajectory_index_++;
    if(current_trajectory_index_ >= execution_data_->size()) {
      result_callback_(execution_result_vector_);
      return;
    }
    for(int i = (int)current_trajectory_index_-1; i >= 0; i--) {
      if((*execution_data_)[i].group_name_ == (*execution_data_)[current_trajectory_index_].group_name_) {
        ROS_INFO_STREAM("Last index is " << i << " current is " << current_trajectory_index_);
	compareLastRecordedToStart((*execution_data_)[i],
				   (*execution_data_)[current_trajectory_index_],
				   execution_result_vector_[i]);
	break;
      }
    }

    // Start next trajectory
    if(!sendTrajectory((*execution_data_)[current_trajectory_index_])) {
      result_callback_(execution_result_vector_);
    }
  } else {
    ROS_ERROR_STREAM( "Trajectory finished with failure.  controller_state=" << controller_state <<
                      ". Stopping the remaining trajectories" << std::endl );
    execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE;
    result_callback_(execution_result_vector_);
  }
};

bool TrajectoryExecutionMonitor::closeEnough(const TrajectoryExecutionRequest& ter,
                                             const TrajectoryExecutionData& ted) {
  if(ted.recorded_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("No points in recorded trajectory");
    return false;
  }
  ROS_WARN_STREAM("Comparing trajectories trajectory");
  double total_distance = TrajectoryStats::distance(ter.trajectory_.points.back(),ted.recorded_trajectory_.points.back());
  if(total_distance < ter.max_joint_distance_) {
    ROS_INFO_STREAM("Allowing because max distance low " << total_distance);
    return true;
  }
  for(unsigned int i = 0; i < ter.trajectory_.points.back().positions.size(); i++) {
    ROS_INFO_STREAM("Distance for " << ter.trajectory_.joint_names[i] << " is " << fabs(ter.trajectory_.points.back().positions[i]-ted.recorded_trajectory_.points.back().positions[i]));
  }

  ROS_INFO_STREAM("Not allowing because max distance high " << total_distance);
  return false;
}

void TrajectoryExecutionMonitor::compareLastRecordedToStart(const TrajectoryExecutionRequest& last_ter,
                                                            const TrajectoryExecutionRequest& next_ter,
                                                            const TrajectoryExecutionData& ted)
{
  if(ted.recorded_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("No points in recorded trajectory for comparison");
    return;
  }

  planning_models::KinematicState last_recorded_state(kmodel_);
  planning_models::KinematicState last_requested_state(kmodel_);
  planning_models::KinematicState next_requested_state(kmodel_);
  
  last_recorded_state.setToDefaultValues();
  last_requested_state.setToDefaultValues();
  next_requested_state.setToDefaultValues();

  std::map<std::string, double> last_recorded_values;
  for(unsigned int i = 0; i < ted.recorded_trajectory_.points.back().positions.size(); i++) {
    ROS_INFO_STREAM("Last recorded " << ted.recorded_trajectory_.joint_names[i] << " value " << ted.recorded_trajectory_.points.back().positions[i]);
    last_recorded_values[ted.recorded_trajectory_.joint_names[i]] = ted.recorded_trajectory_.points.back().positions[i];
  }
  
  std::map<std::string, double> last_requested_values;
  for(unsigned int i = 0; i < last_ter.trajectory_.points.front().positions.size(); i++) {
    ROS_INFO_STREAM("Last requested " << last_ter.trajectory_.joint_names[i] << " value " << last_ter.trajectory_.points.back().positions[i]);
    last_requested_values[last_ter.trajectory_.joint_names[i]] = last_ter.trajectory_.points.back().positions[i];
  }

  std::map<std::string, double> next_requested_values;
  for(unsigned int i = 0; i < next_ter.trajectory_.points.front().positions.size(); i++) {
    ROS_INFO_STREAM("Next requested " << next_ter.trajectory_.joint_names[i] << " value " << next_ter.trajectory_.points.front().positions[i]);
    next_requested_values[next_ter.trajectory_.joint_names[i]] = next_ter.trajectory_.points.front().positions[i];
  }

  last_recorded_state.setStateValues(last_recorded_values);
  last_requested_state.setStateValues(last_requested_values);
  next_requested_state.setStateValues(next_requested_values);

  Eigen::Affine3d recorded_pose = last_recorded_state.getLinkState("r_wrist_roll_link")->getGlobalLinkTransform();
  Eigen::Affine3d last_requested_pose = last_requested_state.getLinkState("r_wrist_roll_link")->getGlobalLinkTransform();
  Eigen::Affine3d next_requested_pose = next_requested_state.getLinkState("r_wrist_roll_link")->getGlobalLinkTransform();

  ROS_INFO_STREAM("Diff in last requested versus recorded is "  
		  << fabs(recorded_pose.translation().x()-last_requested_pose.translation().x()) << " "
		  << fabs(recorded_pose.translation().y()-last_requested_pose.translation().y()) << " "
		  << fabs(recorded_pose.translation().z()-last_requested_pose.translation().z()));

  ROS_INFO_STREAM("Diff is last requested versus next requested " 
		  << fabs(last_requested_pose.translation().x()-next_requested_pose.translation().x()) << " " 
                  << fabs(last_requested_pose.translation().y()-next_requested_pose.translation().y()) << " " 
                  << fabs(last_requested_pose.translation().z()-next_requested_pose.translation().z())); 

  ROS_INFO_STREAM("Diff is last recorded versus next requested " 
		  << fabs(recorded_pose.translation().x()-next_requested_pose.translation().x()) << " " 
                  << fabs(recorded_pose.translation().y()-next_requested_pose.translation().y()) << " " 
                  << fabs(recorded_pose.translation().z()-next_requested_pose.translation().z())); 
}
