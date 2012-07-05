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

#ifndef MOVEIT_MOVEIT_CONTROLLER_MANAGER_
#define MOVEIT_MOVEIT_CONTROLLER_MANAGER_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <moveit_msgs/RobotTrajectory.h>

namespace moveit_controller_manager
{

namespace ExecutionStatus
{
enum Value
  {
    RUNNING, SUCCEEDED, FAILED
  };
}
  
class MoveItControllerHandle
{
public:
  
  MoveItControllerHandle(const std::string &name) : name_(name)
  {
  }

  virtual ~MoveItControllerHandle(void)
  {
  }
  
  const std::string& getName(const std::string &name) const
  {
    return name_;
  }
  
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory) = 0;
  virtual bool cancelExecution(void) = 0;
  virtual void waitForExecution(void) = 0;
  virtual ExecutionStatus::Value getLastExecutionStatus(void) = 0;
  
protected:
  
  std::string name_;
  
};

typedef boost::shared_ptr<MoveItControllerHandle> MoveItControllerHandlePtr;
typedef boost::shared_ptr<const MoveItControllerHandle> MoveItControllerHandleConstPtr;

class MoveItControllerManager
{
public:
  
  struct ControllerState
  {
    bool active_;
    bool loaded_;
    bool default_;
  };
      
  MoveItControllerManager(void)
  {
  }

  virtual ~MoveItControllerManager(void)
  {
  }
  
  virtual MoveItControllerHandlePtr getControllerHandle(const std::string &name) = 0;
  
  virtual void getControllersList(std::vector<std::string> &names) = 0;
  virtual void getActiveControllers(std::vector<std::string> &names) = 0;
  virtual void getLoadedControllers(std::vector<std::string> &names) = 0;

  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints) = 0;
  virtual ControllerState getControllerState(const std::string &name) = 0;

  /// Load a controller, but do not activate it by default
  virtual bool loadController(const std::string &name) = 0;
  virtual bool unloadController(const std::string &name) = 0;

  /// Activate and deactivate loaded controllers
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) = 0;
};

typedef boost::shared_ptr<MoveItControllerManager> MoveItControllerManagerPtr;
typedef boost::shared_ptr<const MoveItControllerManager> MoveItControllerManagerConstPtr;

}

#endif
