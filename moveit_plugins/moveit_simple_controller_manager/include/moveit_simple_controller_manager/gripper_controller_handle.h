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

#ifndef MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <control_msgs/GripperCommandAction.h>
#include <set>

namespace moveit_simple_controller_manager
{


/*
 * This is an interface for a gripper using control_msgs/GripperCommandAction action interface (single DOF).
 */
class GripperControllerHandle : public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  GripperControllerHandle(const std::string &name, const std::string &ns) :
    ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns),
    allow_failure_(false)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_DEBUG_STREAM_NAMED("GripperController", "Received new trajectory for " << name_);

    if (!controller_action_client_)
      return false;

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController", "Gripper cannot execute multi-dof trajectories.");
      return false;
    }

    if (trajectory.joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController", "GripperController expects a joint trajectory with one point only, but %u provided)", (unsigned int)trajectory.joint_trajectory.points.size());
      return false;
    }
    if (trajectory.joint_trajectory.points.size() > 1)
      ROS_WARN_NAMED("GripperController", "GripperController expects a joint trajectory with one point only, but %u provided)", (unsigned int)trajectory.joint_trajectory.points.size());

    if (trajectory.joint_trajectory.joint_names.empty())
    {
      ROS_ERROR_NAMED("GripperController", "No joint names specified");
      return false;
    }

    int gripper_joint_index = -1;
    for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
      if (command_joints_.find(trajectory.joint_trajectory.joint_names[i]) != command_joints_.end())
      {
    gripper_joint_index = i;
    break;
      }
    if (gripper_joint_index < 0)
    {
      ROS_WARN_NAMED("GripperController", "No command_joint was specified for the MoveIt controller gripper handle. Please see GripperControllerHandle::addCommandJoint() and GripperControllerHandle::setCommandJoint(). Assuming index 0.");
      gripper_joint_index = 0;
    }

    if (trajectory.joint_trajectory.points[0].positions.size() <= gripper_joint_index)
    {
      ROS_ERROR_NAMED("GripperController", "GripperController expects a joint trajectory with one point that specifies at least the position of joint '%s', but insufficient positions provided",
                      trajectory.joint_trajectory.joint_names[gripper_joint_index].c_str());
      return false;
    }

    /* TODO: currently sending velocity as effort, make this better. */
    control_msgs::GripperCommandGoal goal;
    if (trajectory.joint_trajectory.points[0].velocities.size() > gripper_joint_index)
      goal.command.max_effort = trajectory.joint_trajectory.points[0].velocities[gripper_joint_index];

    goal.command.position = trajectory.joint_trajectory.points[0].positions[gripper_joint_index];
    controller_action_client_->sendGoal(goal,
                    boost::bind(&GripperControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&GripperControllerHandle::controllerActiveCallback, this),
                    boost::bind(&GripperControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  void setCommandJoint(const std::string& name)
  {
    command_joints_.clear();
    addCommandJoint(name);
  }

  void addCommandJoint(const std::string& name)
  {
    command_joints_.insert(name);
  }

  void allowFailure(bool allow)
  {
    allow_failure_ = allow;
  }

private:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::GripperCommandResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::ABORTED && allow_failure_)
      finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    else
      finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM("GripperController: " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
  {
  }

  /*
   * Some gripper drivers may indicate a failure if they do not close all the way when
   * an object is in the gripper.
   */
  bool allow_failure_;

  /*
   * A GripperCommand message has only a single float64 for the
   * "command", thus only a single joint angle can be sent -- however,
   * due to the complexity of making grippers look correct in a URDF,
   * they typically have >1 joints. The "command_joint" is the joint
   * whose position value will be sent in the GripperCommand action. A
   * set of names is provided for acceptable joint names. If any of
   * the joints specified is found, the value corresponding to that
   * joint is considered the command. E.g., for a two-arm robot, we
   * could have two names: one for the joint of each arm.
   */
  std::set<std::string> command_joints_;
};


} // end namespace moveit_simple_controller_manager

#endif // MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE
