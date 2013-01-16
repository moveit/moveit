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

void convertToRobotTrajectory(moveit_msgs::RobotTrajectory &trajectory,
                              const std::vector<kinematic_state::KinematicStateConstPtr> &states, 
                              const std::vector<ros::Duration> &stamps, const std::string &group)
{
  trajectory = moveit_msgs::RobotTrajectory();
  if (states.empty())
    return;
  const kinematic_model::KinematicModel &kmodel = *states.front()->getKinematicModel();
  const std::vector<const kinematic_model::JointModel*> &jnt = 
    (!group.empty() && kmodel.hasJointModelGroup(group)) ? kmodel.getJointModelGroup(group)->getJointModels() : kmodel.getJointModels();
  
  std::vector<const kinematic_model::JointModel*> onedof;
  std::vector<const kinematic_model::JointModel*> mdof;
  trajectory.joint_trajectory.header.frame_id = kmodel.getModelFrame();
  trajectory.joint_trajectory.header.stamp = ros::Time::now();
  trajectory.joint_trajectory.joint_names.clear();
  trajectory.multi_dof_joint_trajectory.joint_names.clear();
  trajectory.multi_dof_joint_trajectory.child_frame_ids.clear();
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() == 1)
    {
      trajectory.joint_trajectory.joint_names.push_back(jnt[i]->getName());
      onedof.push_back(jnt[i]);
    }
    else
    {
      trajectory.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
      trajectory.multi_dof_joint_trajectory.frame_ids.push_back(trajectory.joint_trajectory.header.frame_id);
      trajectory.multi_dof_joint_trajectory.child_frame_ids.push_back(jnt[i]->getChildLinkModel()->getName());
      mdof.push_back(jnt[i]);
    }
  if (!onedof.empty())
    trajectory.joint_trajectory.points.resize(states.size());
  if (!mdof.empty())
    trajectory.multi_dof_joint_trajectory.points.resize(states.size());
  static const ros::Duration zero_duration(0.0);
  for (std::size_t i = 0 ; i < states.size() ; ++i)
  {
    if (!onedof.empty())
    {
      trajectory.joint_trajectory.points[i].positions.resize(onedof.size());
      for (std::size_t j = 0 ; j < onedof.size() ; ++j)
	trajectory.joint_trajectory.points[i].positions[j] = states[i]->getJointState(onedof[j]->getName())->getVariableValues()[0];
      trajectory.joint_trajectory.points[i].time_from_start = stamps.size() > i ? stamps[i] : zero_duration;
    }
    if (!mdof.empty())
    {
      trajectory.multi_dof_joint_trajectory.points[i].poses.resize(mdof.size());
      for (std::size_t j = 0 ; j < mdof.size() ; ++j)
      {
	tf::poseEigenToMsg(states[i]->getJointState(mdof[j]->getName())->getVariableTransform(),
                           trajectory.multi_dof_joint_trajectory.points[i].poses[j]);
      }
      trajectory.multi_dof_joint_trajectory.points[i].time_from_start = stamps.size() > i ? stamps[i] : zero_duration;
    }
  }
}

void convertToRobotTrajectory(moveit_msgs::RobotTrajectory &trajectory, const std::vector<kinematic_state::KinematicStateConstPtr> &states, const std::string &group)
{
  convertToRobotTrajectory(trajectory, states, std::vector<ros::Duration>(), group);
}

void addPrefixState(const kinematic_state::KinematicState &prefix, moveit_msgs::RobotTrajectory &trajectory,
                    double dt_offset, const kinematic_state::TransformsConstPtr &transforms)
{
  ros::Duration dt(dt_offset);
  
  if (!trajectory.joint_trajectory.points.empty() && !trajectory.joint_trajectory.joint_names.empty())
  {
    trajectory_msgs::JointTrajectoryPoint new_start = trajectory.joint_trajectory.points.front();
    std::vector<double> vals;
    for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
    {
      const kinematic_state::JointState *js = prefix.getJointState(trajectory.joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (js->getVariableValues().size() != 1)
      {
        logError("Unexpected number of joint values. Got %u when 1 should have been found", (unsigned int)js->getVariableValues().size());
        break;
      }
      vals.push_back(js->getVariableValues()[0]);
    }
    
    if (vals.size() == new_start.positions.size())
    {
      new_start.positions = vals;
      
      //insert extra point at the start of trajectory
      trajectory.joint_trajectory.points.insert(trajectory.joint_trajectory.points.begin(), new_start);
      
      // add duration offset for the trajectory
      for (std::size_t j = 1 ; j < trajectory.joint_trajectory.points.size() ; ++j)
        trajectory.joint_trajectory.points[j].time_from_start += dt;
    }
    else
      logError("The number of joints in the solution reported by the planner does not match with the known set of joints");
  }
  
  if (!trajectory.multi_dof_joint_trajectory.points.empty() && !trajectory.multi_dof_joint_trajectory.joint_names.empty())
  {
    moveit_msgs::MultiDOFJointTrajectoryPoint new_start = trajectory.multi_dof_joint_trajectory.points.front();
    EigenSTL::vector_Affine3d poses;
    for (std::size_t i = 0 ; i < trajectory.multi_dof_joint_trajectory.joint_names.size() ; ++i)
    {
      const kinematic_state::JointState *js = prefix.getJointState(trajectory.multi_dof_joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (trajectory.multi_dof_joint_trajectory.child_frame_ids.size() <= i ||
          js->getJointModel()->getChildLinkModel()->getName() != trajectory.multi_dof_joint_trajectory.child_frame_ids[i])
      {
        logError("Unmatched multi-dof joint: '%s'", js->getJointModel()->getChildLinkModel()->getName().c_str());
        break;
      }
      const Eigen::Affine3d& t = js->getVariableTransform();
      if (trajectory.multi_dof_joint_trajectory.frame_ids.size() >= i &&
          trajectory.multi_dof_joint_trajectory.frame_ids[i] != prefix.getKinematicModel()->getModelFrame())
      {
        if (transforms)
          poses.push_back(transforms->getTransform(prefix, trajectory.multi_dof_joint_trajectory.frame_ids[i]) * t);
        else
        {
          logError("Transform to frame '%s' is not known", trajectory.multi_dof_joint_trajectory.frame_ids[i].c_str());
          break;
        }
      }
      else
        poses.push_back(t);
    }
    if (poses.size() == new_start.poses.size())
    {
      for (std::size_t i = 0 ; i < poses.size() ; ++i)
        tf::poseEigenToMsg(poses[i], new_start.poses[i]);
      
      //insert extra point at the start of trajectory
      trajectory.multi_dof_joint_trajectory.points.insert(trajectory.multi_dof_joint_trajectory.points.begin(), new_start);
      
      // add duration offset for the trajectory
      for (std::size_t j = 1 ; j < trajectory.joint_trajectory.points.size() ; ++j)
        trajectory.multi_dof_joint_trajectory.points[j].time_from_start += dt;
    }      
    else
      logError("The number of mulit-dof joints in the solution reported by the planner does not match with the known set of joints");
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
    rs.multi_dof_joint_state.frame_ids = rt.multi_dof_joint_trajectory.frame_ids;
    rs.multi_dof_joint_state.child_frame_ids = rt.multi_dof_joint_trajectory.child_frame_ids;
    rs.multi_dof_joint_state.stamp = rt.joint_trajectory.header.stamp + rt.multi_dof_joint_trajectory.points[index].time_from_start;
    rs.multi_dof_joint_state.poses = rt.multi_dof_joint_trajectory.points[index].poses;
    result = true;
  }
  else
    rs.multi_dof_joint_state = moveit_msgs::MultiDOFJointState();
  return result;
}

void reverseTrajectory(moveit_msgs::RobotTrajectory &trajectory)
{
  std::reverse(trajectory.joint_trajectory.points.begin(), trajectory.joint_trajectory.points.end());
  std::reverse(trajectory.multi_dof_joint_trajectory.points.begin(), trajectory.multi_dof_joint_trajectory.points.end());
}

}
