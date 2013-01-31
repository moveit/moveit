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

/* Author: Ioan Sucan */

#ifndef MOVEIT_KINEMATIC_TRAJECTORY_KINEMATIC_TRAJECTORY_
#define MOVEIT_KINEMATIC_TRAJECTORY_KINEMATIC_TRAJECTORY_

#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

namespace kinematic_trajectory
{

class KinematicTrajectory
{
public:
  KinematicTrajectory(const kinematic_model::KinematicModelConstPtr &kmodel, const std::string &group);
  
  const kinematic_model::KinematicModelConstPtr& getKinematicModel() const
  {
    return kmodel_;
  }
  
  const kinematic_model::JointModelGroup* getGroup() const
  {
    return group_;
  }
  
  const std::string& getGroupName() const;
  
  void setGroupName(const std::string &group_name);
  
  std::size_t getWayPointCount(void) const
  {
    return waypoints_.size();
  } 
  
  const kinematic_state::KinematicState& getWayPoint(std::size_t index) const
  {
    return *waypoints_[index];
  }
  
  const kinematic_state::KinematicState& getLastWayPoint() const
  {
    return *waypoints_.back();
  }

  const kinematic_state::KinematicState& getFirstWayPoint() const
  {
    return *waypoints_.front();
  }
  
  kinematic_state::KinematicStatePtr& getWayPointPtr(std::size_t index)
  {
    return waypoints_[index];
  }

  kinematic_state::KinematicStatePtr& getLastWayPointPtr() 
  {
    return waypoints_.back();
  }

  kinematic_state::KinematicStatePtr& getFirstWayPointPtr() 
  {
    return waypoints_.front();
  }

  const std::vector<double>& getWayPointDurations(void) const
  {
    return duration_from_previous_;
  }
  
  double getWayPointDurationFromPrevious(std::size_t index)
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
  
  void addWayPoint(const kinematic_state::KinematicState &state, double dt)
  {
    addWayPoint(kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(state)), dt);
  }

  void addWayPoint(const kinematic_state::KinematicStatePtr &state, double dt)
  {
    waypoints_.push_back(state);
    duration_from_previous_.push_back(dt);
  }

  void insertWayPoint(std::size_t index, const kinematic_state::KinematicState &state, double dt)
  {
    insertWayPoint(index, kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(state)), dt);
  }

  void insertWayPoint(std::size_t index, const kinematic_state::KinematicStatePtr &state, double dt)  
  {
    waypoints_.insert(waypoints_.begin() + index, state); 
    duration_from_previous_.insert(duration_from_previous_.begin() + index, dt);
  }
  
  void append(const KinematicTrajectory &source, double dt);

  void swap(kinematic_trajectory::KinematicTrajectory &other);
  void swap(std::vector<kinematic_state::KinematicStatePtr> &other);
  
  void clear();
  
  double getAverageSegmentDuration(void) const;
  
  void getRobotTrajectoryMsg(moveit_msgs::RobotTrajectory &msg) const;
  
  void setRobotTrajectoryMsg(const kinematic_state::KinematicState &reference_state,
                             const moveit_msgs::RobotTrajectory &trajectory);
  void setRobotTrajectoryMsg(const kinematic_state::KinematicState &reference_state,
                             const moveit_msgs::RobotState &state, const moveit_msgs::RobotTrajectory &trajectory);
  
    
  void reverse(void);

  void unwind();
  void unwind(const kinematic_state::KinematicState &state);
  
private:

  kinematic_model::KinematicModelConstPtr kmodel_;
  const kinematic_model::JointModelGroup *group_;
  std::vector<kinematic_state::KinematicStatePtr> waypoints_;
  std::vector<double> duration_from_previous_;
};

typedef boost::shared_ptr<KinematicTrajectory> KinematicTrajectoryPtr;
typedef boost::shared_ptr<const KinematicTrajectory> KinematicTrajectoryConstPtr;

}

#endif
