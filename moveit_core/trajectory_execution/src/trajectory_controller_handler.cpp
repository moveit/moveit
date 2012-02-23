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

#include <trajectory_execution/trajectory_controller_handler.h>
#include <trajectory_execution/trajectory_stats.h>

using namespace trajectory_execution;

unsigned int TrajectoryControllerHandler::findClosestIndex( ros::Duration time_from_start )
{
  for(unsigned int i=0; i<overshoot_trajectory_.points.size(); i++)
  {
  trajectory_msgs::JointTrajectoryPoint& p = overshoot_trajectory_.points[i];
    //if( overshoot_trajectory_.points[i].time_from_start > time )
    if( p.time_from_start > time_from_start )
      return i;
  }

  return overshoot_trajectory_.points.size()-1;
}

bool TrajectoryControllerHandler::addNewStateToRecordedTrajectory(const ros::Time& time,
                                                                  const std::map<std::string, double>& joint_positions,
                                                                  const std::map<std::string, double>& joint_velocities)
{
  if( controller_state_ == TrajectoryControllerStates::OVERSHOOTING )
  {
    ros::Duration dur = time - overshoot_trajectory_.header.stamp;
    if( dur > min_overshoot_time_ )
    {
      // calculate the angular distance over the last X seconds
      unsigned int closest_index = findClosestIndex( dur-min_overshoot_time_ );
      double max_vel = TrajectoryStats::getMaxAngularVelocity(overshoot_trajectory_, closest_index);

      if( max_vel <= max_overshoot_velocity_epsilon_ )
      {	// Settled
        controller_state_ = TrajectoryControllerStates::SUCCESS;
        doneDelayed();
        return false;
      }
    }
  }

  if( controller_state_ == TrajectoryControllerStates::EXECUTING )
  {
    return addNewStateToTrajectory(time, joint_positions, joint_velocities, recorded_trajectory_);
  }
  else if( controller_state_ == TrajectoryControllerStates::OVERSHOOTING )
  {
    return addNewStateToTrajectory(time, joint_positions, joint_velocities, overshoot_trajectory_);
  }
  return false;
}

bool TrajectoryControllerHandler::addNewStateToTrajectory(const ros::Time& time,
                                                          const std::map<std::string, double>& joint_positions,
                                                          const std::map<std::string, double>& joint_velocities,
                                                          trajectory_msgs::JointTrajectory& trajectory)
{
  ros::Time start_time;

  trajectory_msgs::JointTrajectoryPoint p;
  for(unsigned int i = 0; i < trajectory.joint_names.size(); i++) {
    const std::string& jn = trajectory.joint_names[i];
    if(joint_positions.find(jn) == joint_positions.end()) {
      return false;
    }
    p.positions.push_back(joint_positions.at(jn));
    if(joint_velocities.find(jn) == joint_velocities.end()) {
      p.velocities.push_back(joint_velocities.at(jn));
    }
    p.time_from_start = time-trajectory.header.stamp;
  }
  trajectory.points.push_back(p);

  return true;
}

bool TrajectoryControllerHandler::enableOvershoot(
    double max_overshoot_velocity_epsilon,
    ros::Duration min_overshoot_time,
    ros::Duration max_overshoot_time )
{
  monitor_overshoot_ = true;
  max_overshoot_velocity_epsilon_ = max_overshoot_velocity_epsilon;
  min_overshoot_time_ = min_overshoot_time;
  max_overshoot_time_ = max_overshoot_time;
  return true;
}

void TrajectoryControllerHandler::disableOvershoot()
{
  monitor_overshoot_ = false;
}


void TrajectoryControllerHandler::timeout(const ros::TimerEvent& event)
{
  if( controller_state_ == TrajectoryControllerStates::OVERSHOOTING )
  {
    ROS_ERROR("overshoot exceeded %f seconds", max_overshoot_time_.toSec());
    controller_state_ = TrajectoryControllerStates::OVERSHOOT_TIMEOUT;
    done();
  }
  else if( controller_state_ == TrajectoryControllerStates::EXECUTING )
  {
    ROS_ERROR("Execution exceeded %f seconds", timeout_.toSec() );
    controller_state_ = TrajectoryControllerStates::EXECUTION_TIMEOUT;
    done();
  }
}

void TrajectoryControllerHandler::initializeRecordedTrajectory(const trajectory_msgs::JointTrajectory& goal_trajectory)
{
  goal_trajectory_ = goal_trajectory;

  recorded_trajectory_.header.stamp = ros::Time::now();
  recorded_trajectory_.joint_names = goal_trajectory.joint_names;
  recorded_trajectory_.points.clear();

  controller_state_ = TrajectoryControllerStates::EXECUTING;

  timer_ = nh_.createTimer(timeout_, &TrajectoryControllerHandler::timeout, this, true, true);
}

void TrajectoryControllerHandler::initializeOvershootTrajectory()
{
  overshoot_trajectory_.header.stamp = ros::Time::now();
  overshoot_trajectory_.joint_names = goal_trajectory_.joint_names;
  overshoot_trajectory_.points.clear();

  controller_state_ = TrajectoryControllerStates::OVERSHOOTING;

  timer_.stop();
  timer_ = nh_.createTimer(max_overshoot_time_, &TrajectoryControllerHandler::timeout, this, true, true);
}

void TrajectoryControllerHandler::done()
{
  timer_.stop();
  const TrajectoryControllerState state = controller_state_;
  controller_state_ = TrajectoryControllerStates::IDLE;
  recorder_->deregisterCallback(group_controller_ns_combo_name_);
  trajectory_finished_callback_( state );
}

void TrajectoryControllerHandler::doneDelayed()
{
  timer_.stop();
  const TrajectoryControllerState state = controller_state_;
  controller_state_ = TrajectoryControllerStates::IDLE;
  recorder_->delayedDeregisterCallback(group_controller_ns_combo_name_);
  trajectory_finished_callback_( state );
}
