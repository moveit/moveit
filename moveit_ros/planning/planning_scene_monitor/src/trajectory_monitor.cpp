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

/* Author: Ioan Sucan */

#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <ros/rate.h>
#include <limits>

planning_scene_monitor::TrajectoryMonitor::TrajectoryMonitor(const CurrentStateMonitorConstPtr &state_monitor, double sampling_frequency) :
  current_state_monitor_(state_monitor), sampling_frequency_(5.0)
{
  setSamplingFrequency(sampling_frequency);
}

planning_scene_monitor::TrajectoryMonitor::~TrajectoryMonitor(void)
{
  stopTrajectoryMonitor();
}

void planning_scene_monitor::TrajectoryMonitor::setSamplingFrequency(double sampling_frequency)
{
  if (sampling_frequency <= std::numeric_limits<double>::epsilon())
    ROS_ERROR("The sampling frequency for trajectory states should be positive");
  else
    sampling_frequency_ = sampling_frequency;
}

bool planning_scene_monitor::TrajectoryMonitor::isActive(void) const
{
  return record_states_thread_;
}

void planning_scene_monitor::TrajectoryMonitor::startTrajectoryMonitor(void)
{
  if (!record_states_thread_)
  {
    record_states_thread_.reset(new boost::thread(boost::bind(&TrajectoryMonitor::recordStates, this)));
    ROS_DEBUG("Started trajectory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::stopTrajectoryMonitor(void)
{
  if (record_states_thread_)
  {
    boost::scoped_ptr<boost::thread> copy;
    copy.swap(record_states_thread_);
    copy->join();
    ROS_DEBUG("Stopped trajectory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::clearTrajectory(void)
{  
  bool restart = isActive();
  if (restart)
    stopTrajectoryMonitor();
  trajectory_states_.clear();
  trajectory_stamps_.clear();
  if (restart)
    startTrajectoryMonitor();
}

void planning_scene_monitor::TrajectoryMonitor::recordStates(void)
{
  if (!current_state_monitor_)
    return;
  
  ros::Rate rate(sampling_frequency_);
  while (record_states_thread_)
  {
    rate.sleep();
    std::pair<kinematic_state::KinematicStatePtr, ros::Time> state = current_state_monitor_->getCurrentStateAndTime();
    trajectory_states_.push_back(state.first);
    trajectory_stamps_.push_back(state.second);
    if (state_add_callback_)
      state_add_callback_(trajectory_states_.back(), trajectory_stamps_.back());
  }
}

void planning_scene_monitor::TrajectoryMonitor::getTrajectory(moveit_msgs::RobotTrajectory &trajectory)
{
  std::vector<ros::Duration> durations(trajectory_stamps_.size(), ros::Duration(0.0));
  for (std::size_t i = 1 ; i < trajectory_stamps_.size() ; ++i)
    durations[i] = trajectory_stamps_[i] - trajectory_stamps_[i - 1];
  trajectory_processing::convertToRobotTrajectory(trajectory, trajectory_states_, durations);
}
