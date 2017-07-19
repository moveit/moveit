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

#ifndef MULTI_DOF_FOLLOW_JOINT_CONTROLLER_HANDLE
#define MULTI_DOF_FOLLOW_JOINT_CONTROLLER_HANDLE

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <ney_wheel_action_controler/MultiDofFollowJointTrajectoryAction.h>

namespace moveit_simple_controller_manager
{

class MultiDofFollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<ney_wheel_action_controler::MultiDofFollowJointTrajectoryAction>
{

public:

  MultiDofFollowJointTrajectoryControllerHandle(const std::string &name, const std::string &action_ns) :
    ActionBasedControllerHandle<ney_wheel_action_controler::MultiDofFollowJointTrajectoryAction>(name, action_ns)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_DEBUG_STREAM("MultiDofFollowJointTrajectoryController: new trajectory to " << name_);

    if (!controller_action_client_)
      return false;

    if (trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("MultiDofFollowJointTrajectoryController: cannot execute single-dof trajectories.");
      return false;
    }

    if (done_)
      ROS_DEBUG_STREAM("MultiDofFollowJointTrajectoryController: sending trajectory to " << name_);
    else
      ROS_DEBUG_STREAM("MultiDofFollowJointTrajectoryController: sending continuation for the currently executed trajectory to " << name_);

    ney_wheel_action_controler::MultiDofFollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.multi_dof_joint_trajectory;
    controller_action_client_-> sendGoal(goal,
                    boost::bind(&MultiDofFollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&MultiDofFollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
                    boost::bind(&MultiDofFollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

protected:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const ney_wheel_action_controler::MultiDofFollowJointTrajectoryResultConstPtr& result)
  {
    ROS_DEBUG_STREAM("controllerDoneCallback");
    finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM("MultiDofFollowJointTrajectoryController: " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const ney_wheel_action_controler::MultiDofFollowJointTrajectoryFeedbackConstPtr& feedback)
  {
    ROS_DEBUG_STREAM("MultiDofFollowJointTrajectoryController: " << name_ << " Feedback");
  }
};


} // end namespace moveit_simple_controller_manager

#endif // MOVEIT_PLUGINS_FOLLOW_TRAJECTORY_CONTROLLER_HANDLE
