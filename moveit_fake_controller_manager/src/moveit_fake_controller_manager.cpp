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
#include <pluginlib/class_list_macros.h>
#include <map>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace moveit_fake_controller_manager
{

static const double POSITION_STEP_FACTOR = 2; //10;
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string JOINT_MODEL_GROUP = "whole_body";
static const std::string JOINT_MODEL_GROUP_POSE = "home";

class FakeControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  FakeControllerHandle(const std::string &name, ros::NodeHandle &nh, const std::vector<std::string> &joints) :
    moveit_controller_manager::MoveItControllerHandle(name),
    nh_(nh),
    joints_(joints),
    loop_hz_(50)
  {
    ROS_INFO_STREAM("Fake controller '" << name << "' loaded");
    std::stringstream ss;
    ss << "With joints [ ";
    for (std::size_t i = 0 ; i < joints.size() ; ++i)
      ss << joints[i] << " ";
    ss << "]";
    ROS_INFO("%s", ss.str().c_str());

    // Load the loader
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

    // Load joint state publisher
    pub_ = nh_.advertise<sensor_msgs::JointState>("fake_controller_joint_states", 100, false);

    // Create default values to publish
    //js_.header = t.joint_trajectory.header;
    js_.name = joints_;
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
      js_.position.push_back(0.0);
      js_.velocity.push_back(0.0);
      js_.effort.push_back(0.0);
    }

    // Load default joint values
    loadDefaultJointValues();

    // Populate the commanded positions
    commanded_.joint_trajectory.points.resize(1);
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
      commanded_.joint_trajectory.points.back().positions.push_back(0.1);
    }

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &FakeControllerHandle::update, this);
  }

  void loadDefaultJointValues()
  {
    // Note: the js_ vector should already be populated with zeros

    // Load the robot model
    robot_model::RobotModelPtr robot_model = robot_model_loader_->getModel(); // Get a shared pointer to the robot

    if (robot_model->hasJointModelGroup(JOINT_MODEL_GROUP))
    {
      moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("whole_body");

      // Load a robot state
      moveit::core::RobotState robot_state(robot_model);

      // Check for existance of joint model group
      if (robot_state.setToDefaultValues(jmg, JOINT_MODEL_GROUP_POSE))
      {
        ROS_INFO_STREAM_NAMED("loadDefaultJointValues","Set joints to pose " << JOINT_MODEL_GROUP_POSE);

        for (std::size_t i = 0; i < joints_.size(); ++i)
        {
          const moveit::core::JointModel* jm = robot_state.getJointModel(joints_[i]);

          // Error check
          if (!jm)
          {
            ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find joint model group: " << joints_[i]);
            continue;
          }
          if (jm->getVariableCount() != 1)
          {
            ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Fake joint controller does not currently accept more than 1 variable per joint");
            continue;
          }

          // Set position from SRDF
          js_.position[i] = robot_state.getJointPositions(jm)[0];
        }
      }
      else
        ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find pose " << JOINT_MODEL_GROUP_POSE << " for the fake controller manager");
    }
    else
      ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find joint model group " << JOINT_MODEL_GROUP << " for the fake controller manager");
  }

  void update(const ros::TimerEvent& e)
  {
    double p_error;
    for (std::size_t i = 0; i < commanded_.joint_trajectory.joint_names.size(); ++i)
    {
      // Position
      p_error = commanded_.joint_trajectory.points.back().positions[i] - js_.position[i];

      // scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
      js_.position[i] += p_error * POSITION_STEP_FACTOR / loop_hz_;
    }

    pub_.publish(js_);
  }

  void getJoints(std::vector<std::string> &joints) const
  {
    joints = joints_;
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &t)
  {
    ROS_INFO("Fake execution of trajectory");
    commanded_ = t;

    return true;
  }

  virtual bool cancelExecution()
  {
    ROS_INFO("Fake trajectory execution cancel");
    return true;
  }

  virtual bool waitForExecution(const ros::Duration &)
  {
    ROS_INFO_STREAM_NAMED("waitForExecution","Sleep " << commanded_.joint_trajectory.points.back().time_from_start);
    ros::Duration(commanded_.joint_trajectory.points.back().time_from_start).sleep();
    return true;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return moveit_controller_manager::ExecutionStatus(moveit_controller_manager::ExecutionStatus::SUCCEEDED);
  }

private:
  ros::NodeHandle nh_;
  std::vector<std::string> joints_;
  ros::Publisher pub_;
  sensor_msgs::JointState js_;
  double loop_hz_;
  ros::Timer non_realtime_loop_;
  moveit_msgs::RobotTrajectory commanded_; // store the goal positions
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
};


class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  MoveItFakeControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM("MoveItFakeControllerManager: No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("MoveItFakeControllerManager: controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0 ; i < controller_list.size() ; ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR("MoveItFakeControllerManager: Name and joints must be specifed for each controller");
        continue;
      }

      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM("MoveItFakeControllerManager: The list of joints for controller " << name << " is not specified as an array");
          continue;
        }
        std::vector<std::string> joints;
        for (int j = 0 ; j < controller_list[i]["joints"].size() ; ++j)
          joints.push_back(std::string(controller_list[i]["joints"][j]));

        controllers_[name].reset(new FakeControllerHandle(name, node_handle_, joints));
      }
      catch (...)
      {
        ROS_ERROR("MoveItFakeControllerManager: Unable to parse controller information");
      }
    }
  }

  virtual ~MoveItFakeControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      ROS_FATAL_STREAM("No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {
    for (std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controllers_.begin() ; it != controllers_.end() ; ++it)
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
    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      static_cast<FakeControllerHandle*>(it->second.get())->getJoints(joints);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }

protected:

  ros::NodeHandle node_handle_;
  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
};

} // end namespace moveit_fake_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
