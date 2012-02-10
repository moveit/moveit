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

#include <trajectory_processing/lspb_trajectory.h>

namespace trajectory_processing
{
LSPBTrajectory::LSPBTrajectory()
{
  apply_limits_ = true;
}

double LSPBTrajectory::calculateMinimumTime(const trajectory_msgs::JointTrajectoryPoint &start, 
                                            const trajectory_msgs::JointTrajectoryPoint &end, 
                                            const std::vector<moveit_msgs::JointLimits> &limits)
{
  double minJointTime(MAX_ALLOWABLE_TIME);
  double segmentTime(0);
  int num_joints = (int) start.positions.size();

  for(int i = 0; i < num_joints; i++)
  {
    minJointTime = minSegmentTime(start.positions[i],end.positions[i],start.velocities[i],end.velocities[i],limits[i]);
    if(segmentTime < minJointTime)
      segmentTime = minJointTime;
  }
  return segmentTime;
}

/*  double LSPBTrajectory::minSegmentTime(double q0, double q1, double v0, double v1, manipulation_msgs::Limits limit)
    {
    double vmax = limit.max_velocity;
    double amax = limit.max_acceleration;
    double diff = jointDiff(q0,q1,limit);

    double tb = std::max(fabs(vmax/amax),sqrt(fabs(diff)/amax));

    double acc(0);
    if(diff>0)
    acc = amax;
    else
    acc = -amax;
    double dist_tb = acc*tb*tb;
    double ts = (diff - dist_tb)/(acc*tb);
    if(ts < 0)
    ts = 0;

    return (2*tb+ts);
    }
*/

double LSPBTrajectory::minSegmentTime(const double &q0, 
                                      const double &q1, 
                                      const double &v0, 
                                      const double &v1, 
                                      const moveit_msgs::JointLimits &limit)
{
  double vmax = limit.max_velocity;
  double amax = limit.max_acceleration;
  double diff = jointDiff(q0,q1,limit);

  double tb = fabs(vmax/amax);

  double acc(0);
  if(diff>0)
    acc = amax;
  else
    acc = -amax;
  double dist_tb = acc*tb*tb;
  double ts = (diff - dist_tb)/(acc*tb);
  if(ts < 0)
  {
    ts = 0.0;
  }
  return (2*tb+ts);
}


double LSPBTrajectory::blendTime(const double &aa,
                                 const double &bb,
                                 const double &cc)
{
  double disc = (pow(bb,2) - 4*aa*cc);
  if(disc < 0)
  {   
    ROS_DEBUG("Blend time quadratic coeff: %f %f %f",aa,bb,cc);
    return 0.0;
  }
  double tb1 = (-bb + sqrt(disc))/(2*aa);
  double tb2 = (-bb - sqrt(disc))/(2*aa);
  if(std::isnan(tb1))
    tb1 = 0.0;
  if(std::isnan(tb2))
    tb2 = 0.0;
  return std::min(tb1,tb2);
}

/*  double LSPBTrajectory::blendTime(double diff,double a,double tf)
    {
    return fabs(diff/(a*tf));
    }
*/
bool LSPBTrajectory::parameterize(const trajectory_msgs::JointTrajectory& trajectory_in,
                                  const std::vector<moveit_msgs::JointLimits>& limits,
                                  spline_msgs::LSPBTrajectoryMsg& spline)
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
    if(!limits[i].has_acceleration_limits)
    {
      ROS_ERROR("Trying to apply acceleration limits without supplying them. Set acceleration_limits in the message and set has_acceleration_limits flag to true.");
      return false;
    }
  }
  for (int i=1; i< num_traj; ++i)
  {
    spline.segments[i-1].joints.resize(num_joints);
    for(unsigned int j=0; j < spline.segments[i-1].joints.size(); j++)
      spline.segments[i-1].joints[j].coefficients.resize(3);

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
      double diff = jointDiff(trajectory_in.points[i-1].positions[j],trajectory_in.points[i].positions[j],limits[j]);
      double acc = 0.0;
      if(diff > 0)
        acc = limits[j].max_acceleration;
      else
        acc = - limits[j].max_acceleration;
      double tb = blendTime(acc,-acc*dT,diff);
      //        double tb = blendTime(diff,acc,dT);

      spline.segments[i-1].joints[j].coefficients[0] = trajectory_in.points[i-1].positions[j];
      spline.segments[i-1].joints[j].coefficients[1] = 0.0;
      spline.segments[i-1].joints[j].coefficients[2] = 0.5*acc;
      spline.segments[i-1].joints[j].quadratic_segment_duration = tb;
      spline.segments[i-1].joints[j].linear_segment_duration = std::max(dT-2*tb,0.0);
    }
  }
  return true;
}
}
