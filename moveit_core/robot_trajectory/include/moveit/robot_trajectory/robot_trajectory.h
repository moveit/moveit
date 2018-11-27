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

/* Author: Ioan Sucan, Adam Leeper */

#ifndef MOVEIT_ROBOT_TRAJECTORY_KINEMATIC_TRAJECTORY_
#define MOVEIT_ROBOT_TRAJECTORY_KINEMATIC_TRAJECTORY_

#include <moveit/macros/class_forward.h>
#include <moveit/macros/deprecation.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <deque>

namespace robot_trajectory
{
MOVEIT_CLASS_FORWARD(RobotTrajectory);

/** \brief Maintain a sequence of waypoints and the time durations
    between these waypoints */
class RobotTrajectory
{
public:
  RobotTrajectory(const robot_model::RobotModelConstPtr& robot_model, const std::string& group);

  RobotTrajectory(const robot_model::RobotModelConstPtr& robot_model, const robot_model::JointModelGroup* group);

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  const robot_model::JointModelGroup* getGroup() const
  {
    return group_;
  }

  const std::string& getGroupName() const;

  void setGroupName(const std::string& group_name);

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
   *  @return The duration from start; returns overall duration if index is out of range.
   */
  double getWayPointDurationFromStart(std::size_t index) const;

  MOVEIT_DEPRECATED double getWaypointDurationFromStart(std::size_t index) const;

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

  /**
   * \brief Add a point to the trajectory
   * \param state - current robot state
   * \param dt - duration from previous
   */
  void addSuffixWayPoint(const robot_state::RobotState& state, double dt)
  {
    addSuffixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  /**
   * \brief Add a point to the trajectory
   * \param state - current robot state
   * \param dt - duration from previous
   */
  void addSuffixWayPoint(const robot_state::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.push_back(state);
    duration_from_previous_.push_back(dt);
  }

  void addPrefixWayPoint(const robot_state::RobotState& state, double dt)
  {
    addPrefixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  void addPrefixWayPoint(const robot_state::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.push_front(state);
    duration_from_previous_.push_front(dt);
  }

  void insertWayPoint(std::size_t index, const robot_state::RobotState& state, double dt)
  {
    insertWayPoint(index, robot_state::RobotStatePtr(new robot_state::RobotState(state)), dt);
  }

  void insertWayPoint(std::size_t index, const robot_state::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.insert(waypoints_.begin() + index, state);
    duration_from_previous_.insert(duration_from_previous_.begin() + index, dt);
  }

  /**
   * \brief Add a specified part of a trajectory to the end of the current trajectory. The default (when \p start_index
   * and \p end_index are omitted) is to add the whole trajectory.
   * \param source - the trajectory containing the part to append to the end of current trajectory
   * \param dt - time step between last traj point in current traj and first traj point of append traj
   * \param start_index - index of first traj point of the part to append from the source traj, the default is to add
   * from the start of the source traj
   * \param end_index - index of last traj point of the part to append from the source traj, the default is to add until
   * the end of the source traj
   */
  void append(const RobotTrajectory& source, double dt, size_t start_index = 0,
              size_t end_index = std::numeric_limits<std::size_t>::max());

  void swap(robot_trajectory::RobotTrajectory& other);

  void clear();

  double getAverageSegmentDuration() const;

  void getRobotTrajectoryMsg(moveit_msgs::RobotTrajectory& trajectory) const;

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Each
     point in the trajectory
      to be constructed internally is obtained by copying the reference state and overwriting the content from a
     trajectory point in \e trajectory. */
  void setRobotTrajectoryMsg(const robot_state::RobotState& reference_state,
                             const trajectory_msgs::JointTrajectory& trajectory);

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Each
     point in the trajectory
      to be constructed internally is obtained by copying the reference state and overwriting the content from a
     trajectory point in \e trajectory. */
  void setRobotTrajectoryMsg(const robot_state::RobotState& reference_state,
                             const moveit_msgs::RobotTrajectory& trajectory);

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Before
     use, the reference state is updated
      using \e state. Each point in the trajectory  to be constructed internally is obtained by copying the reference
     state and overwriting the content
      from a trajectory point in \e trajectory. */
  void setRobotTrajectoryMsg(const robot_state::RobotState& reference_state, const moveit_msgs::RobotState& state,
                             const moveit_msgs::RobotTrajectory& trajectory);

  void reverse();

  void unwind();
  void unwind(const robot_state::RobotState& state);

  /** @brief Finds the waypoint indicies before and after a duration from start.
   *  @param The duration from start.
   *  @param The waypoint index before the supplied duration.
   *  @param The waypoint index after (or equal to) the supplied duration.
   *  @param The progress (0 to 1) between the two waypoints, based on time (not based on joint distances).
   */
  void findWayPointIndicesForDurationAfterStart(const double& duration, int& before, int& after, double& blend) const;

  // TODO support visitor function for interpolation, or at least different types.
  /** @brief Gets a robot state corresponding to a supplied duration from start for the trajectory, using linear time
   * interpolation.
   *  @param The duration from start.
   *  @param The resulting robot state.
   *  @return True if state is valid, false otherwise (trajectory is empty).
   */
  bool getStateAtDurationFromStart(const double request_duration, robot_state::RobotStatePtr& output_state) const;

private:
  robot_model::RobotModelConstPtr robot_model_;
  const robot_model::JointModelGroup* group_;
  std::deque<robot_state::RobotStatePtr> waypoints_;
  std::deque<double> duration_from_previous_;
};
}

#endif
