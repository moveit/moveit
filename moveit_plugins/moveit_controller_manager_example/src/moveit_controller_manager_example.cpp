/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.hpp>
#include <map>

namespace moveit_controller_manager_example
{
class ExampleControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ExampleControllerHandle(const std::string& name) : moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& t)
  {
    // do whatever is needed to actually execute this trajectory
    return true;
  }

  virtual bool cancelExecution()
  {
    // do whatever is needed to cancel execution
    return true;
  }

  virtual bool waitForExecution(const ros::Duration&)
  {
    // wait for the current execution to finish
    return true;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return moveit_controller_manager::ExecutionStatus(moveit_controller_manager::ExecutionStatus::SUCCEEDED);
  }
};

class MoveItControllerManagerExample : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItControllerManagerExample()
  {
  }

  virtual ~MoveItControllerManagerExample()
  {
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name)
  {
    return moveit_controller_manager::MoveItControllerHandlePtr(new ExampleControllerHandle(name));
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string>& names)
  {
    names.resize(1);
    names[0] = "my_example_controller";
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal
   * with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string& name, std::vector<std::string>& joints)
  {
    joints.clear();
    if (name == "my_example_controller")
    {
      // declare which joints this controller actuates
      joints.push_back("joint1");
      joints.push_back("joint2");
      joints.push_back("joint3");
      joints.push_back("joint4");
      // ...
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate)
  {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
};

}  // end namespace moveit_controller_manager_example

PLUGINLIB_EXPORT_CLASS(moveit_controller_manager_example::MoveItControllerManagerExample,
                       moveit_controller_manager::MoveItControllerManager);
