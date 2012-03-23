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

#ifndef _TRAJECTORY_EXECUTION_MONITOR_ROS_H_
#define _TRAJECTORY_EXECUTION_MONITOR_ROS_H_

#include <ros/ros.h>
#include <trajectory_execution/trajectory_execution_monitor.h>
#include <pluginlib/class_loader.h>

namespace trajectory_execution_ros
{

class TrajectoryExecutionMonitorRos : public trajectory_execution::TrajectoryExecutionMonitor
{
public:
  TrajectoryExecutionMonitorRos(const planning_models::KinematicModelConstPtr& kmodel, 
				bool manage_controllers=true);

  virtual ~TrajectoryExecutionMonitorRos();
  
  virtual void loadController(const std::string& name);
  virtual void unloadController(const std::string& name);
  virtual void restoreOriginalControllers();

  virtual bool getRunningControllerMap(std::map<std::string, bool>& controller_map);
  
  void unloadAllLoadedControllers();

protected:
  virtual void switchControllers(const std::vector<std::string>& on_controllers,
                                 const std::vector<std::string>& off_controllers);
  
  ros::ServiceClient loader_service_;
  ros::ServiceClient unloader_service_;
  ros::ServiceClient switcher_service_;
  ros::ServiceClient lister_service_;

  pluginlib::ClassLoader<trajectory_execution::TrajectoryControllerHandler> controller_handler_loader_;
  pluginlib::ClassLoader<trajectory_execution::TrajectoryRecorder> recorder_loader_;
  

};

}

#endif
