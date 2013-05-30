/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  Copyright (c) 2013, Michael E. Ferguson
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

/* Author: Ioan Sucan, E. Gil Jones, Michael Ferguson */
/* This is a simplified controller manager which uses control_msgs
 * actions. It is based off the earlier pr2_controller_manager. */

#include <ros/ros.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <map>

namespace moveit_simple_controller_manager
{

/*
 * The gripper ...
 */
class GripperControllerHandle : public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  GripperControllerHandle(const std::string &name, const std::string &ns  = "gripper_action") :
    ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns),
    closing_(false)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_INFO_STREAM("new trajectory to " << name_);
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The simple gripper controller cannot execute multi-dof trajectories.");
      return false;
    }
    
    if (trajectory.joint_trajectory.points.size() != 1)
    {
      ROS_ERROR("The simple gripper controller expects a joint trajectory with one point only, but %u provided)", (unsigned int)trajectory.joint_trajectory.points.size());
      return false;
    }

    if (trajectory.joint_trajectory.points[0].positions.empty())
    {
      ROS_ERROR("The simple gripper controller expects a joint trajectory with one point that specifies at least one position, but 0 positions provided)");
      return false;
    }
    
    /* TODO: currently sending velocity as effort, make this better. */
    control_msgs::GripperCommandGoal goal;
    if (!trajectory.joint_trajectory.points[0].velocities.empty())
      goal.command.max_effort = trajectory.joint_trajectory.points[0].velocities[0];
    goal.command.position = trajectory.joint_trajectory.points[0].positions[0];
    controller_action_client_->sendGoal(goal,
                    boost::bind(&GripperControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&GripperControllerHandle::controllerActiveCallback, this),
                    boost::bind(&GripperControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }
  
private:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::GripperCommandResultConstPtr& result)
  {
    // the gripper action reports failure when closing the gripper and an object is inside
    //if (state == actionlib::SimpleClientGoalState::ABORTED && closing_)
      finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    //else
      //finishControllerExecution(state);
  }
  
  void controllerActiveCallback() 
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }
  
  void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
  {
  }
  
  bool closing_;
};

/*
 * This is generally used for arms, but could also be used for multi-dof hands.
 */
class SimpleFollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>
{
public:
  
  SimpleFollowJointTrajectoryControllerHandle(const std::string &name, const std::string &ns = "follow_joint_trajectory") :
    ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, ns)
  {  
  }
  
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_INFO_STREAM("new trajectory to " << name_);
    if (!controller_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The FollowJointTrajectory controller cannot execute multi-dof trajectories.");
      return false;
    }
    if (done_)
      ROS_DEBUG_STREAM("Sending trajectory to FollowJointTrajectory action for controller " << name_);
    else
      ROS_DEBUG_STREAM("Sending continuation for the currently executed trajectory to FollowJointTrajectory action for controller " << name_);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;
    controller_action_client_->sendGoal(goal,
                    boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
                    boost::bind(&SimpleFollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }
  
protected:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    finishControllerExecution(state);
  }
  
  void controllerActiveCallback() 
  {
    ROS_DEBUG_STREAM("Controller " << name_ << " started execution");
  }
  
  void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
  {
  }
};












class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  
  MoveItSimpleControllerManager() : node_handle_("~")
  { 
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM("MoveitSimpleControllerManager: No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("MoveitSimpleControllerManager: controller_list should be specified as an array");
      return;
    }
    
    /* actually create each controller */
    for (int i = 0 ; i < controller_list.size() ; ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR("MoveitSimpleControllerManager: Name and joints must be specifed for each controller");
        continue;
      }
      
      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        std::string ns;
        if (controller_list[i].hasMember("ns"))
          ns = std::string(controller_list[i]["ns"]);

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM("MoveitSimpleControllerManager: The list of joints for controller " << name << " is not specified as an array");
          continue;
        }

        if (!controller_list[i].hasMember("type"))
        {
          ROS_ERROR_STREAM("MoveitSimpleControllerManager: No type specified for controller " << name);
          continue;
        }

        std::string type = std::string(controller_list[i]["type"]);

        //moveit_controller_manager::MoveItControllerHandlePtr new_handle;
        ActionBasedControllerHandleBasePtr new_handle;
        if ( type == "GripperCommand" )
        {
          new_handle.reset(ns.empty() ? new GripperControllerHandle(name) : new GripperControllerHandle(name, ns));
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM("MoveitSimpleControllerManager: Added GripperCommand controller for " << name );
            controllers_[name] = new_handle;
          }
        }
        else if ( type == "FollowJointTrajectory" )
        {
          new_handle.reset(ns.empty() ? new SimpleFollowJointTrajectoryControllerHandle(name) : new SimpleFollowJointTrajectoryControllerHandle(name, ns));
          if (static_cast<SimpleFollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM("MoveitSimpleControllerManager: Added FollowJointTrajectory controller for " << name );
            controllers_[name] = new_handle;
          }
        }

        for (int j = 0 ; j < controller_list[i]["joints"].size() ; ++j)
          controllers_[name]->addJoint(std::string(controller_list[i]["joints"][j]));
      }
      catch (...)
      {
        ROS_ERROR("MoveitSimpleControllerManager: Unable to parse controller information");
      }
    }
  }
  
  virtual ~MoveItSimpleControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr /*moveit_controller_manager::MoveItControllerHandlePtr*/>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      ROS_FATAL_STREAM("No such controller: " << name);
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {    
    for (std::map<std::string, ActionBasedControllerHandleBasePtr /*moveit_controller_manager::MoveItControllerHandlePtr*/>::const_iterator it = controllers_.begin() ; it != controllers_.end() ; ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr /*moveit_controller_manager::MoveItControllerHandlePtr*/>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all loaded, active, and default -- that's what makes this thing simple.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.loaded_ = true;
    state.active_ = true;
    state.default_ = true;
    return state;
  }
  
  /* All of our controllers are already loaded. */
  virtual bool loadController(const std::string &name) { return true; }
  
  /* Cannot unload our controllers */
  virtual bool unloadController(const std::string &name) { return false; }
  
  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }
  
protected:
  
  ros::NodeHandle node_handle_;
  std::map<std::string, ActionBasedControllerHandleBasePtr/*moveit_controller_manager::MoveItControllerHandlePtr*/> controllers_;
};

}

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
