/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Sachin Chitta */

#include <trajectory_processing/linear_trajectory.h>

namespace trajectory_processing
{

LinearTrajectory::LinearTrajectory()
{
  apply_limits_ = true;
}

double LinearTrajectory::calculateMinimumTime(const trajectory_msgs::JointTrajectoryPoint &start, 
                                              const trajectory_msgs::JointTrajectoryPoint &end, 
                                              const std::vector<moveit_msgs::JointLimits> &limits)
{
  double minJointTime(MAX_ALLOWABLE_TIME);
  double segmentTime(0);
  int num_joints = (int) start.positions.size();

  for(int i = 0; i < num_joints; i++)
  {
    double diff = jointDiff(start.positions[i],end.positions[i],limits[i]);
    minJointTime = fabs(diff) / limits[i].max_velocity;
    if(segmentTime < minJointTime)
      segmentTime = minJointTime;
  }
  return segmentTime;
}

bool LinearTrajectory::parameterize(const trajectory_msgs::JointTrajectory& trajectory_in, 
                                    const std::vector<moveit_msgs::JointLimits> &limits,
                                    spline_msgs::SplineTrajectory& spline)
{
  int num_traj = trajectory_in.points.size();
  int num_joints = trajectory_in.joint_names.size();
  spline.names = trajectory_in.joint_names;
  spline.segments.resize(num_traj-1);

  for(int i=0; i<num_joints; i++)
  {
    if(!limits[i].has_velocity_limits)
    {
      ROS_ERROR("Trying to apply velocity limits without supplying them. Set velocity_limits in the message and set has_velocity_limits flag to true.");
      return false;
    }
  }
  for (int i=1; i< num_traj; ++i)
  {
    spline.segments[i-1].joints.resize(num_joints);
    for(int j=0; j < num_joints; j++)
    {
      spline.segments[i-1].joints[j].coefficients.resize(2);
    }

    double dT = (trajectory_in.points[i].time_from_start - trajectory_in.points[i-1].time_from_start).toSec();
    if(apply_limits_)
    {
      double dTMin = calculateMinimumTime(trajectory_in.points[i-1],trajectory_in.points[i],limits);
      if(dTMin > dT) // if minimum time required to satisfy limits is greater than time available, stretch this segment
        dT = dTMin;      
    }
    spline.segments[i-1].duration = ros::Duration(dT);
    for(int j=0; j<num_joints; j++)
    {
      spline.segments[i-1].joints[j].coefficients[0] = trajectory_in.points[i-1].positions[j];
      spline.segments[i-1].joints[j].coefficients[1] = jointDiff(trajectory_in.points[i-1].positions[j],trajectory_in.points[i].positions[j],limits[j])/spline.segments[i-1].duration.toSec();
    }
  }
  return true;
}
}
