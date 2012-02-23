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

/** \author E. Gil Jones */

#include <trajectory_execution_ros/follow_joint_trajectory_controller_handler.h>
#include <pluginlib/class_list_macros.h>

namespace trajectory_execution_ros 
{

bool FollowJointTrajectoryControllerHandler::initialize(const std::string& group_name, 
                                                        const std::string& controller_name,
                                                        const std::string& ns_name)
{
  TrajectoryControllerHandler::initialize(group_name, controller_name, ns_name);
  
  follow_joint_trajectory_action_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(controller_name+"/"+ns_name, true));
  while(ros::ok() && !follow_joint_trajectory_action_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO_STREAM("Waiting for the follow joint trajectory action for group " << group_name << " on the topic " << controller_name << " to come up");
  }
  return true;
}

bool FollowJointTrajectoryControllerHandler::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                                               boost::shared_ptr<trajectory_execution::TrajectoryRecorder>& recorder,
                                                               const trajectory_execution::TrajectoryFinishedCallbackFunction& traj_callback)
{
  recorder_ = recorder;
  trajectory_finished_callback_ = traj_callback;
  
  initializeRecordedTrajectory(trajectory);
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  
  follow_joint_trajectory_action_client_->sendGoal(goal,
                                                   boost::bind(&FollowJointTrajectoryControllerHandler::controllerDoneCallback, this, _1, _2),
                                                   boost::bind(&FollowJointTrajectoryControllerHandler::controllerActiveCallback, this),
                                                   boost::bind(&FollowJointTrajectoryControllerHandler::controllerFeedbackCallback, this, _1));
  recorder_->registerCallback(group_controller_ns_combo_name_, 
                              boost::bind(&FollowJointTrajectoryControllerHandler::addNewStateToRecordedTrajectory, this, _1, _2, _3));
  return true;
}

void FollowJointTrajectoryControllerHandler::cancelExecution() {   
}

void FollowJointTrajectoryControllerHandler::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                                    const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO_STREAM("Controller is done with state " << state.toString() );
  
  if( controller_state_ == trajectory_execution::TrajectoryControllerStates::EXECUTING ||
      controller_state_ == trajectory_execution::TrajectoryControllerStates::OVERSHOOTING )
  {
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      controller_state_ = trajectory_execution::TrajectoryControllerStates::SUCCESS;
    }
    else
    {
      ROS_WARN_STREAM("Failed state is " << state.toString() << " code " << result->error_code);
      controller_state_ = trajectory_execution::TrajectoryControllerStates::EXECUTION_FAILURE;
    }
    
    // record overshoot
    if( controller_state_==trajectory_execution::TrajectoryControllerStates::SUCCESS )
    {
      if( monitor_overshoot_ )
      {
        initializeOvershootTrajectory();
      }
      else
      {
        done();
      }
    }
    else
    {
      ROS_WARN_STREAM("Controller returned an error.  Not recording the overshoot.");
      done();
    }
  }
}

void FollowJointTrajectoryControllerHandler::controllerActiveCallback() 
{
  ROS_DEBUG_STREAM("Controller went active");
}

void FollowJointTrajectoryControllerHandler::controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Got feedback");
}

}

PLUGINLIB_DECLARE_CLASS(trajectory_execution_ros, FollowJointTrajectoryControllerHandler,
                        trajectory_execution_ros::FollowJointTrajectoryControllerHandler,
                        trajectory_execution::TrajectoryControllerHandler);
