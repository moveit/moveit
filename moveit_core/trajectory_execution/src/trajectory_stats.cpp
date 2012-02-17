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
 *
 *
 *********************************************************************/

/* \author: Ken Anderson */

#include <cmath>
#include <ros/ros.h>
#include <trajectory_execution_monitor/trajectory_stats.h>

using namespace std;
using namespace trajectory_execution_monitor;

ros::Duration TrajectoryStats::getDuration(const trajectory_msgs::JointTrajectory& trajectory)
{
  size_t tsize = trajectory.points.size();
  if( tsize < 1 )
  {
    return ros::Duration(0,0);
  }

  const trajectory_msgs::JointTrajectoryPoint& point1 = trajectory.points[0];
  const trajectory_msgs::JointTrajectoryPoint& point2 = trajectory.points[tsize-1];
  return point2.time_from_start - point1.time_from_start;
}

double TrajectoryStats::getAngularDistance(const trajectory_msgs::JointTrajectory& trajectory, unsigned int start_index )
{
  double angular_diff_sum = 0.0;
  size_t tsize = trajectory.points.size();

  // Loop through trajectory points
  if( tsize > 1 )
  {
    for(unsigned int i=start_index; i<tsize-1; i++)
    {
      trajectory_msgs::JointTrajectoryPoint point1 = trajectory.points[i];
      trajectory_msgs::JointTrajectoryPoint point2 = trajectory.points[i+1];

      if(point1.positions.size() != point2.positions.size())
      {
        ROS_ERROR_STREAM("Invalid Trajectory, the number of joints is inconsistent");
        return 0.0;
      }

      angular_diff_sum += distance(point1,point2);
    }
  }

  return angular_diff_sum;
}

double TrajectoryStats::getMaxAngularVelocity(const trajectory_msgs::JointTrajectory& trajectory, unsigned int start_index )
{
  double max_angular_vel = 0.0;
  size_t tsize = trajectory.points.size();

  // Loop through trajectory points
  if( tsize > 1 )
  {
    for(unsigned int i=start_index; i<tsize-1; i++)
    {
      trajectory_msgs::JointTrajectoryPoint point1 = trajectory.points[i];
      trajectory_msgs::JointTrajectoryPoint point2 = trajectory.points[i+1];

      if(point1.positions.size() != point2.positions.size())
      {
        ROS_ERROR_STREAM("Invalid Trajectory, the number of joints is inconsistent");
        return 0.0;
      }

      const double seconds = point2.time_from_start.toSec() - point1.time_from_start.toSec();
      const double dist = distance(point1,point2);
      const double velocity = std::abs( dist/seconds );

      if( velocity > max_angular_vel )
      {
        max_angular_vel = velocity;
      }
    }
  }

  return max_angular_vel;
}

double TrajectoryStats::distance(
  const trajectory_msgs::JointTrajectoryPoint& point1,
  const trajectory_msgs::JointTrajectoryPoint& point2,
  const std::vector<std::string> joint_names
)
{
  double total_distance = 0.0;
  for(unsigned int i = 0; i < point1.positions.size(); i++) {
    if(joint_names.size()>i)
    {
      ROS_DEBUG_STREAM("Distance for " << joint_names[i] << " is " << fabs(point1.positions[i]-point2.positions[i]));
    }
    total_distance += fabs(point1.positions[i]-point2.positions[i]);
  }
  return total_distance;
}
