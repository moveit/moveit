/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan, Adam Leeper */

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/math/constants/constants.hpp>
#include <numeric>

robot_trajectory::RobotTrajectory::RobotTrajectory(const robot_model::RobotModelConstPtr &kmodel, const std::string &group) :
  kmodel_(kmodel),
  group_(group.empty() ? NULL : kmodel->getJointModelGroup(group))
{
}

void robot_trajectory::RobotTrajectory::setGroupName(const std::string &group_name)
{
  group_ = kmodel_->getJointModelGroup(group_name);
}

const std::string& robot_trajectory::RobotTrajectory::getGroupName() const
{
  if (group_)
    return group_->getName();
  static const std::string empty;
  return empty;
}

double robot_trajectory::RobotTrajectory::getAverageSegmentDuration() const
{
  if (duration_from_previous_.empty())
    return 0.0;
  else
    return std::accumulate(duration_from_previous_.begin(), duration_from_previous_.end(), 0.0) / (double)duration_from_previous_.size();
}

void robot_trajectory::RobotTrajectory::swap(robot_trajectory::RobotTrajectory &other)
{
  kmodel_.swap(other.kmodel_);
  std::swap(group_, other.group_);
  waypoints_.swap(other.waypoints_);
  duration_from_previous_.swap(duration_from_previous_);
}

void robot_trajectory::RobotTrajectory::append(const RobotTrajectory &source, double dt)
{
  waypoints_.insert(waypoints_.end(), source.waypoints_.begin(),source.waypoints_.end());
  std::size_t index = duration_from_previous_.size();
  duration_from_previous_.insert(duration_from_previous_.end(), source.duration_from_previous_.begin(),source.duration_from_previous_.end());
  if (duration_from_previous_.size() > index)
    duration_from_previous_[index] += dt;
}

void robot_trajectory::RobotTrajectory::reverse()
{
  std::reverse(waypoints_.begin(), waypoints_.end());
  if (!duration_from_previous_.empty())
  {
    duration_from_previous_.push_back(duration_from_previous_.front());
    std::reverse(duration_from_previous_.begin(), duration_from_previous_.end());
    duration_from_previous_.pop_back();
  }
}

void robot_trajectory::RobotTrajectory::unwind()
{
  if (waypoints_.empty())
    return;

  const std::vector<const robot_model::JointModel*> &cont_joints = group_ ?
    group_->getContinuousJointModels() : kmodel_->getContinuousJointModels();

  for (std::size_t i = 0 ; i < cont_joints.size() ; ++i)
  {
    // unwrap continuous joints
    double running_offset = 0.0;
    double last_value = waypoints_[0]->getJointState(cont_joints[i])->getVariableValues()[0];

    for (std::size_t j = 1 ; j < waypoints_.size() ; ++j)
    {
      robot_state::JointState *js = waypoints_[j]->getJointState(cont_joints[i]);
      std::vector<double> current_value = js->getVariableValues();
      if (last_value > current_value[0] + boost::math::constants::pi<double>())
        running_offset += 2.0 * boost::math::constants::pi<double>();
      else
        if (current_value[0] > last_value + boost::math::constants::pi<double>())
          running_offset -= 2.0 * boost::math::constants::pi<double>();

      last_value = current_value[0];
      if (running_offset > std::numeric_limits<double>::epsilon() || running_offset < -std::numeric_limits<double>::epsilon())
      {
        current_value[0] += running_offset;
        js->setVariableValues(current_value);
      }
    }
  }
}

void robot_trajectory::RobotTrajectory::unwind(const robot_state::RobotState &state)
{
  if (waypoints_.empty())
    return;

  const std::vector<const robot_model::JointModel*> &cont_joints = group_ ?
    group_->getContinuousJointModels() : kmodel_->getContinuousJointModels();

  for (std::size_t i = 0 ; i < cont_joints.size() ; ++i)
  {
    const robot_state::JointState *jstate = state.getJointState(cont_joints[i]);
    std::vector<double> reference_value = jstate->getVariableValues();
    jstate->getJointModel()->enforceBounds(reference_value);

    // unwrap continuous joints
    double running_offset = jstate->getVariableValues()[0] - reference_value[0];

    robot_state::JointState *js0 =  waypoints_[0]->getJointState(cont_joints[i]);
    double last_value = js0->getVariableValues()[0];
    if (running_offset > std::numeric_limits<double>::epsilon() || running_offset < -std::numeric_limits<double>::epsilon())
    {
      std::vector<double> current_value = js0->getVariableValues();
      current_value[0] += running_offset;
      js0->setVariableValues(current_value);
    }

    for (std::size_t j = 1 ; j < waypoints_.size() ; ++j)
    {
      robot_state::JointState *js = waypoints_[j]->getJointState(cont_joints[i]);
      std::vector<double> current_value = js->getVariableValues();
      if (last_value > current_value[0] + boost::math::constants::pi<double>())
        running_offset += 2.0 * boost::math::constants::pi<double>();
      else
        if (current_value[0] > last_value + boost::math::constants::pi<double>())
          running_offset -= 2.0 * boost::math::constants::pi<double>();

      last_value = current_value[0];
      if (running_offset > std::numeric_limits<double>::epsilon() || running_offset < -std::numeric_limits<double>::epsilon())
      {
        current_value[0] += running_offset;
        js->setVariableValues(current_value);
      }
    }
  }
}

void robot_trajectory::RobotTrajectory::clear()
{
  waypoints_.clear();
  duration_from_previous_.clear();
}

void robot_trajectory::RobotTrajectory::getRobotTrajectoryMsg(moveit_msgs::RobotTrajectory &trajectory) const
{
  trajectory = moveit_msgs::RobotTrajectory();
  if (waypoints_.empty())
    return;
  const std::vector<const robot_model::JointModel*> &jnt = group_ ? group_->getJointModels() : kmodel_->getJointModels();

  std::vector<const robot_model::JointModel*> onedof;
  std::vector<const robot_model::JointModel*> mdof;
  trajectory.joint_trajectory.joint_names.clear();
  trajectory.multi_dof_joint_trajectory.joint_names.clear();

  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() == 1)
    {
      trajectory.joint_trajectory.joint_names.push_back(jnt[i]->getName());
      onedof.push_back(jnt[i]);
    }
    else
    {
      trajectory.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
      mdof.push_back(jnt[i]);
    }
  if (!onedof.empty())
  {
    trajectory.joint_trajectory.header.frame_id = kmodel_->getModelFrame();
    trajectory.joint_trajectory.header.stamp = ros::Time(0);
    trajectory.joint_trajectory.points.resize(waypoints_.size());
  }

  if (!mdof.empty())
  {
    trajectory.multi_dof_joint_trajectory.header.frame_id = kmodel_->getModelFrame();
    trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    trajectory.multi_dof_joint_trajectory.points.resize(waypoints_.size());
  }

  static const ros::Duration zero_duration(0.0);
  double total_time = 0.0;
  for (std::size_t i = 0 ; i < waypoints_.size() ; ++i)
  {
    if (duration_from_previous_.size() > i)
      total_time += duration_from_previous_[i];

    if (!onedof.empty())
    {
      trajectory.joint_trajectory.points[i].positions.resize(onedof.size());
      trajectory.joint_trajectory.points[i].velocities.reserve(onedof.size());
      for (std::size_t j = 0 ; j < onedof.size() ; ++j)
      {
        const robot_state::JointState *js = waypoints_[i]->getJointState(onedof[j]->getName());
        trajectory.joint_trajectory.points[i].positions[j] = js->getVariableValues()[0];
        // if we have velocities, copy those too
        if (!js->getVelocities().empty())
          trajectory.joint_trajectory.points[i].velocities.push_back(js->getVelocities()[0]);
        if (!js->getAccelerations().empty())
          trajectory.joint_trajectory.points[i].accelerations.push_back(js->getAccelerations()[0]);
      }
      // clear velocities if we have an incomplete specification
      if (trajectory.joint_trajectory.points[i].velocities.size() != onedof.size())
        trajectory.joint_trajectory.points[i].velocities.clear();

      if (duration_from_previous_.size() > i)
        trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(total_time);
      else
        trajectory.joint_trajectory.points[i].time_from_start = zero_duration;
    }
    if (!mdof.empty())
    {
      trajectory.multi_dof_joint_trajectory.points[i].transforms.resize(mdof.size());
      for (std::size_t j = 0 ; j < mdof.size() ; ++j)
        tf::transformEigenToMsg(waypoints_[i]->getJointState(mdof[j]->getName())->getVariableTransform(), trajectory.multi_dof_joint_trajectory.points[i].transforms[j]);
      if (duration_from_previous_.size() > i)
        trajectory.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(total_time);
      else
        trajectory.multi_dof_joint_trajectory.points[i].time_from_start = zero_duration;
    }
  }
}

void robot_trajectory::RobotTrajectory::setRobotTrajectoryMsg(const robot_state::RobotState &reference_state,
                                                              const moveit_msgs::RobotTrajectory &trajectory)
{
  // make a copy just in case the next clear() removes the memory for the reference passed in
  robot_state::RobotState copy = reference_state;
  clear();

  std::size_t state_count = std::max(trajectory.joint_trajectory.points.size(),
                                     trajectory.multi_dof_joint_trajectory.points.size());
  ros::Time last_time_stamp = trajectory.joint_trajectory.points.empty() ? trajectory.multi_dof_joint_trajectory.header.stamp : trajectory.joint_trajectory.header.stamp;
  ros::Time this_time_stamp = last_time_stamp;

  for (std::size_t i = 0 ; i < state_count ; ++i)
  {
    moveit_msgs::RobotState rs;
    if (trajectory.joint_trajectory.points.size() > i)
    {
      rs.joint_state.header = trajectory.joint_trajectory.header;
      rs.joint_state.header.stamp = rs.joint_state.header.stamp + trajectory.joint_trajectory.points[i].time_from_start;
      rs.joint_state.name = trajectory.joint_trajectory.joint_names;
      rs.joint_state.position = trajectory.joint_trajectory.points[i].positions;
      rs.joint_state.velocity = trajectory.joint_trajectory.points[i].velocities;
      this_time_stamp = rs.joint_state.header.stamp;
    }
    if (trajectory.multi_dof_joint_trajectory.points.size() > i)
    {
      rs.multi_dof_joint_state.joint_names = trajectory.multi_dof_joint_trajectory.joint_names;
      rs.multi_dof_joint_state.header = trajectory.multi_dof_joint_trajectory.header;
      rs.multi_dof_joint_state.header.stamp = trajectory.multi_dof_joint_trajectory.header.stamp + trajectory.multi_dof_joint_trajectory.points[i].time_from_start;
      rs.multi_dof_joint_state.joint_transforms = trajectory.multi_dof_joint_trajectory.points[i].transforms;
      this_time_stamp = rs.multi_dof_joint_state.header.stamp;
    }

    robot_state::RobotStatePtr st(new robot_state::RobotState(copy));
    robot_state::robotStateMsgToRobotState(rs, *st);
    addSuffixWayPoint(st, (this_time_stamp - last_time_stamp).toSec());
    last_time_stamp = this_time_stamp;
  }
}

void robot_trajectory::RobotTrajectory::setRobotTrajectoryMsg(const robot_state::RobotState &reference_state,
                                                              const moveit_msgs::RobotState &state, const moveit_msgs::RobotTrajectory &trajectory)
{
  robot_state::RobotState st(reference_state);
  robot_state::robotStateMsgToRobotState(state, st);
  setRobotTrajectoryMsg(st, trajectory);
}

void robot_trajectory::RobotTrajectory::findWayPointIndicesForDurationAfterStart( const double& duration, int& before,
                                                                                  int& after, double &blend ) const
{
  if(duration < 0.0)
  {
    before = 0;
    after = 0;
    blend = 0;
    return;
  }

  // Find indicies
  std::size_t index = 0, num_points = waypoints_.size();
  double running_duration = 0.0;
  for( ; index < num_points; index++)
  {
    running_duration += duration_from_previous_[index];
    if( running_duration >= duration )
      break;
  }
  before = std::max<int>(index - 1, 0);
  after = std::min<int>(index, num_points - 1);

  // Compute duration blend
  double before_time = running_duration - duration_from_previous_[index];
  if(after == before)
    blend = 1.0;
  else
    blend = (duration - before_time) / duration_from_previous_[index];
}

double robot_trajectory::RobotTrajectory::getWaypointDurationFromStart(std::size_t index) const
{
  if(index >= duration_from_previous_.size())
    return -1.0; // or something else? should we throw an exception?

  double time = 0;
  for(std::size_t i = 0; i <= index; ++i)
    time += duration_from_previous_[i];
  return time;
}

bool robot_trajectory::RobotTrajectory::getStateAtDurationFromStart(const double request_duration,
                                                                    robot_state::RobotStatePtr& output_state) const
{
  if( !getWayPointCount() )
    return false;

  int before=0, after=0;
  double blend = 1.0;
  findWayPointIndicesForDurationAfterStart(request_duration, before, after, blend);
  //logDebug("Interpolating %.3f of the way between index %d and %d.", blend, before, after);
  waypoints_[before]->interpolate(*waypoints_[after], blend, *output_state);
  // TODO at some point we should allow for different interpolation types, or visitor classes.
  return true;
}
