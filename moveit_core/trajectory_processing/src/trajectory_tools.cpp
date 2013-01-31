/*********************************************************************
* Software License Agreement (BSD License)
*
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

/* Author: Ioan Sucan */

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_state/conversions.h>
#include <boost/math/constants/constants.hpp>
#include <eigen_conversions/eigen_msg.h>
  
namespace trajectory_processing
{

void convertToKinematicStates(std::vector<kinematic_state::KinematicStatePtr> &states,
                              const moveit_msgs::RobotState &start_state, const moveit_msgs::RobotTrajectory &trajectory,
                              const kinematic_state::KinematicState &reference_state, const kinematic_state::TransformsConstPtr &transforms)
{
  states.clear();
  kinematic_state::KinematicState start(reference_state);
  kinematic_state::robotStateToKinematicState(*transforms, start_state, start);
  std::size_t state_count = std::max(trajectory.joint_trajectory.points.size(),
                                     trajectory.multi_dof_joint_trajectory.points.size());
  states.resize(state_count);
  for (std::size_t i = 0 ; i < state_count ; ++i)
  {
    moveit_msgs::RobotState rs;
    robotTrajectoryPointToRobotState(trajectory, i, rs);
    kinematic_state::KinematicStatePtr st(new kinematic_state::KinematicState(start));
    kinematic_state::robotStateToKinematicState(*transforms, rs, *st);
    states[i] = st;
  }
}

double getTrajectoryDuration(const moveit_msgs::RobotTrajectory &trajectory)
{
  if (trajectory.joint_trajectory.points.size() > 1 || trajectory.multi_dof_joint_trajectory.points.size() > 1)
    return (trajectory.joint_trajectory.points.size() > trajectory.multi_dof_joint_trajectory.points.size()) ? 
      trajectory.joint_trajectory.points.back().time_from_start.toSec() : trajectory.multi_dof_joint_trajectory.points.back().time_from_start.toSec();
  else
    return 0.0;
}

bool isTrajectoryEmpty(const moveit_msgs::RobotTrajectory &trajectory)
{ 
  return trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty();
}

double averageSegmentDuration(const moveit_msgs::RobotTrajectory &trajectory)
{   
  if (trajectory.joint_trajectory.points.size() > 1 || trajectory.multi_dof_joint_trajectory.points.size() > 1)
    return (trajectory.joint_trajectory.points.size() > trajectory.multi_dof_joint_trajectory.points.size()) ? 
      trajectory.joint_trajectory.points.back().time_from_start.toSec() / (double)(trajectory.joint_trajectory.points.size() - 1) :
      trajectory.multi_dof_joint_trajectory.points.back().time_from_start.toSec() / (double)(trajectory.multi_dof_joint_trajectory.points.size() - 1);
  else
    return 0.0;
}

std::size_t trajectoryPointCount(const moveit_msgs::RobotTrajectory &trajectory)
{
  return std::max(trajectory.joint_trajectory.points.size(), trajectory.multi_dof_joint_trajectory.points.size());
}

bool robotTrajectoryPointToRobotState(const moveit_msgs::RobotTrajectory &rt, std::size_t index, moveit_msgs::RobotState &rs)
{
  bool result = false;
  if (rt.joint_trajectory.points.size() > index)
  {
    rs.joint_state.header = rt.joint_trajectory.header;
    rs.joint_state.header.stamp = rs.joint_state.header.stamp + rt.joint_trajectory.points[index].time_from_start;
    rs.joint_state.name = rt.joint_trajectory.joint_names;
    rs.joint_state.position = rt.joint_trajectory.points[index].positions;
    rs.joint_state.velocity = rt.joint_trajectory.points[index].velocities;
    result = true;
  }
  else
    rs.joint_state = sensor_msgs::JointState();
  if (rt.multi_dof_joint_trajectory.points.size() > index)
  {
    rs.multi_dof_joint_state.joint_names = rt.multi_dof_joint_trajectory.joint_names;
    rs.multi_dof_joint_state.header.stamp = rt.joint_trajectory.header.stamp + rt.multi_dof_joint_trajectory.points[index].time_from_start;
    rs.multi_dof_joint_state.joint_values = rt.multi_dof_joint_trajectory.points[index].values;
    result = true;
  }
  else
    rs.multi_dof_joint_state = moveit_msgs::MultiDOFJointState();
  return result;
}

}
