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

#include "planning_scene_monitor/trajectory_monitor.h"
#include <ros/rate.h>

planning_scene_monitor::TrajectoryMonitor::TrajectoryMonitor(const CurrentStateMonitorConstPtr &state_monitor, double sampling_frequency) :
  current_state_monitor_(state_monitor), sampling_frequency_(sampling_frequency), record_states_thread_(NULL)
{
}

planning_scene_monitor::TrajectoryMonitor::~TrajectoryMonitor(void)
{
  stopTrajectoryMonitor();
}

bool planning_scene_monitor::TrajectoryMonitor::isActive(void) const
{
  return record_states_thread_ != NULL;
}

void planning_scene_monitor::TrajectoryMonitor::startTrajectoryMonitor(void)
{
  if (!record_states_thread_)
  {
    record_states_thread_ = new boost::thread(boost::bind(&TrajectoryMonitor::recordStates, this));
    ROS_DEBUG("Started trajecory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::stopTrajectoryMonitor(void)
{
  if (record_states_thread_)
  {
    boost::thread *copy = record_states_thread_;
    record_states_thread_ = NULL;
    copy->join();
    ROS_DEBUG("Stopped trajecory monitor");
  }
}

void planning_scene_monitor::TrajectoryMonitor::clearTrajectory(void)
{  
  bool restart = isActive();
  if (restart)
    stopTrajectoryMonitor();
  trajectory_states_.clear();
  if (restart)
    startTrajectoryMonitor();
}

void planning_scene_monitor::TrajectoryMonitor::recordStates(void)
{
  ros::Rate rate(sampling_frequency_);
  while (record_states_thread_)
  {
    rate.sleep();
    trajectory_states_.push_back(current_state_monitor_->getCurrentState());
    if (state_add_callback_)
      state_add_callback_(trajectory_states_.back());
  }
}
