/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <ros/ros.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <map>

namespace moveit_simple_controller_manager
{
class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItSimpleControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED("manager", "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR_STREAM_NAMED("manager", "Name and joints must be specifed for each controller");
        continue;
      }

      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        std::string action_ns;
        if (controller_list[i].hasMember("ns"))
        {
          /* TODO: this used to be called "ns", renaming to "action_ns" and will remove in the future */
          action_ns = std::string(controller_list[i]["ns"]);
          ROS_WARN_NAMED("manager", "Use of 'ns' is deprecated, use 'action_ns' instead.");
        }
        else if (controller_list[i].hasMember("action_ns"))
          action_ns = std::string(controller_list[i]["action_ns"]);
        else
          ROS_WARN_NAMED("manager", "Please note that 'action_ns' no longer has a default value.");

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM_NAMED("manager", "The list of joints for controller " << name
                                                                                 << " is not specified as an array");
          continue;
        }

        if (!controller_list[i].hasMember("type"))
        {
          ROS_ERROR_STREAM_NAMED("manager", "No type specified for controller " << name);
          continue;
        }

        std::string type = std::string(controller_list[i]["type"]);

        ActionBasedControllerHandleBasePtr new_handle;
        if (type == "GripperCommand")
        {
          new_handle.reset(new GripperControllerHandle(name, action_ns));
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            if (controller_list[i].hasMember("parallel"))
            {
              if (controller_list[i]["joints"].size() != 2)
              {
                ROS_ERROR_STREAM_NAMED("manager", "Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<GripperControllerHandle*>(new_handle.get())
                  ->setParallelJawGripper(controller_list[i]["joints"][0], controller_list[i]["joints"][1]);
            }
            else
            {
              if (controller_list[i].hasMember("command_joint"))
                static_cast<GripperControllerHandle*>(new_handle.get())
                    ->setCommandJoint(controller_list[i]["command_joint"]);
              else
                static_cast<GripperControllerHandle*>(new_handle.get())
                    ->setCommandJoint(controller_list[i]["joints"][0]);
            }

            if (controller_list[i].hasMember("allow_failure"))
              static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);

            ROS_INFO_STREAM_NAMED("manager", "Added GripperCommand controller for " << name);
            controllers_[name] = new_handle;
          }
        }
        else if (type == "FollowJointTrajectory")
        {
          new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
          if (static_cast<FollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM_NAMED("manager", "Added FollowJointTrajectory controller for " << name);
            controllers_[name] = new_handle;
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("manager", "Unknown controller type: " << type.c_str());
          continue;
        }
        if (!controllers_[name])
        {
          controllers_.erase(name);
          continue;
        }

        /* add list of joints, used by controller manager and moveit */
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          controllers_[name]->addJoint(std::string(controller_list[i]["joints"][j]));
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED("manager", "Caught unknown exception while parsing controller information");
      }
    }
  }

  virtual ~MoveItSimpleControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name)
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      ROS_FATAL_STREAM_NAMED("manager", "No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string>& names)
  {
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM_NAMED("manager", "Returned " << names.size() << " controllers in list");
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
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN_NAMED("manager", "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                                "not loaded on the param server?",
                     name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default -- that's what makes this thing simple.
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
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
