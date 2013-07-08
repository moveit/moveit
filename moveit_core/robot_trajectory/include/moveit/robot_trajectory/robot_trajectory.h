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

#ifndef MOVEIT_ROBOT_TRAJECTORY_KINEMATIC_TRAJECTORY_
#define MOVEIT_ROBOT_TRAJECTORY_KINEMATIC_TRAJECTORY_

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <deque>

namespace robot_trajectory
{

class RobotTrajectory
{
public:
  RobotTrajectory(const robot_model::RobotModelConstPtr &kmodel, const std::string &group);

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return kmodel_;
  }

  const robot_model::JointModelGroup* getGroup() const
  {
    return group_;
  }

  const std::string& getGroupName() const;

  void setGroupName(const std::string &group_name);

  std::size_t getWayPointCount() const
  {
    return waypoints_.size();
  }

  const robot_state::RobotState& getWayPoint(std::size_t index) const
  {
    return *waypoints_[index];
  }

  const robot_state::RobotState& getLastWayPoint() const
  {
    return *waypoints_.back();
  }

  const robot_state::RobotState& getFirstWayPoint() const
  {
    return *waypoints_.front();
  }

  robot_state::RobotStatePtr& getWayPointPtr(std::size_t index)
  {
    return waypoints_[index];
  }

  robot_state::RobotStatePtr& getLastWayPointPtr()
  {
    return waypoints_.back();
  }

  robot_state::RobotStatePtr& getFirstWayPointPtr()
  {
    return waypoints_.front();
  }

  const std::deque<double>& getWayPointDurations() const
  {
    return duration_from_previous_;
  }

  /** @brief  Returns the duration after start that a waypoint will be reached.
   *  @param  The waypoint index.
   *  @return The duration from start; retuns -1.0 if index is out of range.
   */
  double getWaypointDurationFromStart(std::size_t index) const;

  double getWayPointDurationFromPrevious(std::size_t index) const
  {
    if (duration_from_previous_.size() > index)
      return duration_from_previous_[index];
    else
      return 0.0;
  }

  void setWayPointDurationFromPrevious(std::size_t index, double value)
  {
    if (duration_from_previous_.size() <= index)
      duration_from_previous_.resize(index + 1, 0.0);
    duration_from_previous_[index] = value;
  }

  bool empty() const
  {
    return waypoints_.empty();
  }

  void addSuffixWayPoint(const robot_state::RobotState &state, double dt)
  {
    addSuffixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  void addSuffixWayPoint(const robot_state::RobotStatePtr &state, double dt)
  {
    waypoints_.push_back(state);
    duration_from_previous_.push_back(dt);
  }

  void addPrefixWayPoint(const robot_state::RobotState &state, double dt)
  {
    addPrefixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  void addPrefixWayPoint(const robot_state::RobotStatePtr &state, double dt)
  {
    waypoints_.push_front(state);
    duration_from_previous_.push_front(dt);
  }

  void insertWayPoint(std::size_t index, const robot_state::RobotState &state, double dt)
  {
    insertWayPoint(index, robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  void insertWayPoint(std::size_t index, const robot_state::RobotStatePtr &state, double dt)
  {
    waypoints_.insert(waypoints_.begin() + index, state);
    duration_from_previous_.insert(duration_from_previous_.begin() + index, dt);
  }

  void append(const RobotTrajectory &source, double dt);

  void swap(robot_trajectory::RobotTrajectory &other);

  void clear();

  double getAverageSegmentDuration() const;

  void getRobotTrajectoryMsg(moveit_msgs::RobotTrajectory &trajectory) const;

  void setRobotTrajectoryMsg(const robot_state::RobotState &reference_state,
                             const moveit_msgs::RobotTrajectory &trajectory);
  void setRobotTrajectoryMsg(const robot_state::RobotState &reference_state,
                             const moveit_msgs::RobotState &state, const moveit_msgs::RobotTrajectory &trajectory);


  void reverse();

  void unwind();
  void unwind(const robot_state::RobotState &state);

  /** @brief Finds the waypoint indicies before and after a duration from start.
   *  @param The duration from start.
   *  @param The waypoint index before the supplied duration.
   *  @param The waypoint index after (or equal to) the supplied duration.
   *  @param The progress (0 to 1) between the two waypoints, based on time (not based on joint distances).
   */
  void findWayPointIndicesForDurationAfterStart(const double& duration, int& before, int& after, double &blend) const;

  // TODO support visitor function for interpolation, or at least different types.
  /** @brief Gets a robot state corresponding to a supplied duration from start for the trajectory, using linear time interpolation.
   *  @param The duration from start.
   *  @param The resulting robot state.
   *  @return True if state is valid, false otherwise (trajectory is empty).
   */
  bool getStateAtDurationFromStart(const double request_duration,
                                   robot_state::RobotStatePtr& output_state) const;

private:

  robot_model::RobotModelConstPtr kmodel_;
  const robot_model::JointModelGroup *group_;
  std::deque<robot_state::RobotStatePtr> waypoints_;
  std::deque<double> duration_from_previous_;
};

typedef boost::shared_ptr<RobotTrajectory> RobotTrajectoryPtr;
typedef boost::shared_ptr<const RobotTrajectory> RobotTrajectoryConstPtr;

}

#endif
