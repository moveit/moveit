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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef TEST_MOVEIT_CONTROLLER_MANAGER_
#define TEST_MOVEIT_CONTROLLER_MANAGER_

#include <moveit/controller_manager/controller_manager.h>

namespace test_moveit_controller_manager
{
class TestMoveItControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  TestMoveItControllerHandle(const std::string& name) : MoveItControllerHandle(name)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
  {
    return true;
  }

  virtual bool cancelExecution()
  {
    return true;
  }

  virtual bool waitForExecution(const ros::Duration& timeout = ros::Duration(0))
  {
    return false;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }
};

class TestMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  static const int ACTIVE = 1;
  static const int DEFAULT = 2;

  TestMoveItControllerManager()
  {
    controllers_["right_arm"] = DEFAULT;
    controllers_["left_arm"] = ACTIVE + DEFAULT;
    controllers_["arms"] = 0;
    controllers_["base"] = DEFAULT;
    controllers_["head"] = 0;
    controllers_["left_arm_head"] = 0;

    controller_joints_["right_arm"].push_back("rj1");
    controller_joints_["right_arm"].push_back("rj2");

    controller_joints_["left_arm"].push_back("lj1");
    controller_joints_["left_arm"].push_back("lj2");
    controller_joints_["left_arm"].push_back("lj3");

    controller_joints_["arms"].insert(controller_joints_["arms"].end(), controller_joints_["left_arm"].begin(),
                                      controller_joints_["left_arm"].end());
    controller_joints_["arms"].insert(controller_joints_["arms"].end(), controller_joints_["right_arm"].begin(),
                                      controller_joints_["right_arm"].end());

    controller_joints_["base"].push_back("basej");
    controller_joints_["head"].push_back("headj");

    controller_joints_["left_arm_head"].insert(controller_joints_["left_arm_head"].end(),
                                               controller_joints_["left_arm"].begin(),
                                               controller_joints_["left_arm"].end());
    controller_joints_["left_arm_head"].insert(controller_joints_["left_arm_head"].end(),
                                               controller_joints_["head"].begin(), controller_joints_["head"].end());
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name)
  {
    return moveit_controller_manager::MoveItControllerHandlePtr(new TestMoveItControllerHandle(name));
  }

  virtual void getControllersList(std::vector<std::string>& names)
  {
    names.clear();
    for (std::map<std::string, int>::const_iterator it = controllers_.begin(); it != controllers_.end(); ++it)
      names.push_back(it->first);
  }

  virtual void getActiveControllers(std::vector<std::string>& names)
  {
    names.clear();
    for (std::map<std::string, int>::const_iterator it = controllers_.begin(); it != controllers_.end(); ++it)
      if (it->second & ACTIVE)
        names.push_back(it->first);
  }

  virtual void getControllerJoints(const std::string& name, std::vector<std::string>& joints)
  {
    joints = controller_joints_[name];
  }

  virtual moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = controllers_[name] & ACTIVE;
    state.default_ = false;
    return state;
  }

  virtual bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate)
  {
    for (std::size_t i = 0; i < deactivate.size(); ++i)
    {
      controllers_[deactivate[i]] &= ~ACTIVE;
      std::cout << "Deactivated controller " << deactivate[i] << std::endl;
    }
    for (std::size_t i = 0; i < activate.size(); ++i)
    {
      controllers_[activate[i]] |= ACTIVE;
      std::cout << "Activated controller " << activate[i] << std::endl;
    }
    return true;
  }

protected:
  std::map<std::string, int> controllers_;
  std::map<std::string, std::vector<std::string> > controller_joints_;
};
}
#endif
