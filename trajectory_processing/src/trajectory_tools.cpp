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

#include "trajectory_processing/trajectory_tools.h"
#include <ros/console.h>

namespace trajectory_processing
{

void addPrefixState(const planning_models::KinematicState &prefix, moveit_msgs::RobotTrajectory &trajectory,
                    double dt_offset, const planning_models::TransformsConstPtr &transforms)
{
  ros::Duration dt(dt_offset);
  
  if (!trajectory.joint_trajectory.points.empty() && !trajectory.joint_trajectory.joint_names.empty())
  {
    trajectory_msgs::JointTrajectoryPoint new_start = trajectory.joint_trajectory.points.front();
    std::vector<double> vals;
    for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
    {
      const planning_models::KinematicState::JointState *js = prefix.getJointState(trajectory.joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (js->getVariableValues().size() != 1)
      {
        ROS_ERROR("Unexpected number of joint values. Got %u when 1 should have been found", (unsigned int)js->getVariableValues().size());
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
      ROS_ERROR("The number of joints in the solution reported by the planner does not match with the known set of joints");
  }
  
  if (!trajectory.multi_dof_joint_trajectory.points.empty() && !trajectory.multi_dof_joint_trajectory.joint_names.empty())
  {
    moveit_msgs::MultiDOFJointTrajectoryPoint new_start = trajectory.multi_dof_joint_trajectory.points.front();
    std::vector<Eigen::Affine3d> poses;
    for (std::size_t i = 0 ; i < trajectory.multi_dof_joint_trajectory.joint_names.size() ; ++i)
    {
      const planning_models::KinematicState::JointState *js = prefix.getJointState(trajectory.multi_dof_joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (trajectory.multi_dof_joint_trajectory.child_frame_ids.size() <= i ||
          js->getJointModel()->getChildLinkModel()->getName() != trajectory.multi_dof_joint_trajectory.child_frame_ids[i])
      {
        ROS_ERROR("Unmatched multi-dof joint: '%s'", js->getJointModel()->getChildLinkModel()->getName().c_str());
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
          ROS_ERROR("Transform to frame '%s' is not known", trajectory.multi_dof_joint_trajectory.frame_ids[i].c_str());
          break;
        }
      }
      else
        poses.push_back(t);
    }
    if (poses.size() == new_start.poses.size())
    {
      for (std::size_t i = 0 ; i < poses.size() ; ++i)
        planning_models::msgFromPose(poses[i], new_start.poses[i]);
      
      //insert extra point at the start of trajectory
      trajectory.multi_dof_joint_trajectory.points.insert(trajectory.multi_dof_joint_trajectory.points.begin(), new_start);
      
      // add duration offset for the trajectory
      for (std::size_t j = 1 ; j < trajectory.joint_trajectory.points.size() ; ++j)
        trajectory.multi_dof_joint_trajectory.points[j].time_from_start += dt;
    }      
    else
      ROS_ERROR("The number of mulit-dof joints in the solution reported by the planner does not match with the known set of joints");
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


}
