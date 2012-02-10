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

#include <trajectory_processing/cubic_parameterized_trajectory.h>

namespace trajectory_processing
{
  CubicParameterizedTrajectory::CubicParameterizedTrajectory()
  {
    apply_limits_ = true;
  }

  double CubicParameterizedTrajectory::getDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                                                   const trajectory_msgs::JointTrajectoryPoint &end,
                                                   const std::vector<moveit_msgs::JointLimits> &limits)
  {
    double position = 0.0;
    for(unsigned int i=0; i < start.positions.size(); i++)
      position += jointDiff(start.positions[i],end.positions[i],limits[i]) * jointDiff(start.positions[i],end.positions[i],limits[i]);
    position = sqrt(position);
    return position;
  }

  double CubicParameterizedTrajectory::getVelocityLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                                                      const trajectory_msgs::JointTrajectoryPoint &end,
                                                      const std::vector<moveit_msgs::JointLimits> &limits)
  {
    double velocity = DBL_MAX;
    for(unsigned int i=0; i < start.positions.size(); i++)
    {
      double tmp_vel = limits[i].max_velocity/fabs(end.positions[i]-start.positions[i]);
      if(velocity > tmp_vel)
        velocity = tmp_vel;
    }
    return velocity;
  }

  double CubicParameterizedTrajectory::getAccelerationLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                                                            const trajectory_msgs::JointTrajectoryPoint &end,
                                                            const std::vector<moveit_msgs::JointLimits> &limits)
  {
    double acceleration = DBL_MAX;
    for(unsigned int i=0; i < start.positions.size(); i++)
    {
      double tmp_vel = limits[i].max_acceleration/fabs(end.positions[i]-start.positions[i]);
      if(acceleration > tmp_vel)
        acceleration = tmp_vel;
    }
    return acceleration;
  }

  bool CubicParameterizedTrajectory::hasAccelerationLimits(const std::vector<moveit_msgs::JointLimits> &limits)
  {
    for(unsigned int i=0; i < limits.size(); i++)
      if(!limits[i].has_acceleration_limits)
        return false;
    return true;
  }

  void CubicParameterizedTrajectory::getLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                                              const trajectory_msgs::JointTrajectoryPoint &end,
                                              const std::vector<moveit_msgs::JointLimits> &limits,
                                              moveit_msgs::JointLimits &limit_out)
  {
    limit_out.has_position_limits = true;
    limit_out.min_position = 0;
    limit_out.max_position = 1;
    limit_out.has_velocity_limits = true;
    limit_out.max_velocity = getVelocityLimit(start,end,limits);

    limit_out.has_acceleration_limits = false;
    if(hasAccelerationLimits(limits))     
    {
      limit_out.max_acceleration = getAccelerationLimit(start,end,limits);
      limit_out.has_acceleration_limits = true;
    }
  }


  double CubicParameterizedTrajectory::jointDiff(const double &start, 
                                                 const double &end,
                                                 const moveit_msgs::JointLimits &limit)
  {
    if(limit.has_position_limits)
      return end-start;
    else
      return angles::shortest_angular_distance(start,end);
  }

  void CubicParameterizedTrajectory::solveCubicSpline(const double &q0,
                                                      const double &q1,
                                                      const double &v0, 
                                                      const double &v1,
                                                      const double &dt,
                                                      std::vector<double> &coefficients)
  {
    coefficients.resize(4);
    double diff = q1-q0;
    coefficients[0] = q0;
    coefficients[1] = v0;
    coefficients[2] = (3*diff-(2*v0+v1)*dt)/(dt*dt);;
    coefficients[3] = (-2*diff+(v0+v1)*dt)/pow(dt,3);
  }

  bool CubicParameterizedTrajectory::parameterize(const trajectory_msgs::JointTrajectory& trajectory_in, 
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

      for(int j =0; j < num_joints; j++)
        spline.segments[i-1].joints[j].coefficients.resize(4);
      double dT = (trajectory_in.points[i].time_from_start - trajectory_in.points[i-1].time_from_start).toSec();

      // Start and end parameterized positions
      trajectory_msgs::JointTrajectoryPoint start,end;
      start.positions.resize(1);
      start.velocities.resize(1);
      start.positions[0] = 0.0;

      end.positions.resize(1);
      end.velocities.resize(1);
      end.positions[0] = 1.0;

      // Velocity limits on parameter
      moveit_msgs::JointLimits parameter_limits;
      parameter_limits.joint_name = "cubic_parameterization";
      getLimit(trajectory_in.points[i-1],trajectory_in.points[i],limits,parameter_limits);

      if(apply_limits_)
      {
        trajectory_processing::CubicTrajectory solver;
        spline_msgs::SplineTrajectory spline;
        trajectory_msgs::JointTrajectory traj;
        double dTMin;

        traj.points.push_back(start);
        traj.points.push_back(end);

        solver.parameterize(traj,limits,spline);
        trajectory_processing::getTotalTime(spline,dTMin);
        if(dTMin > dT) // if minimum time required to satisfy limits is greater than time available, stretch this segment
          dT = dTMin;      
      }

      std::vector<double> coefficients;
      solveCubicSpline(start.positions[0],
                       end.positions[0],
                       0.0,
                       0.0,
                       dT,
                       coefficients);
      spline.segments[i-1].duration = ros::Duration(dT);

      for(int j=0; j<num_joints; j++)
      {
        double diff = trajectory_in.points[i].positions[j] - trajectory_in.points[i-1].positions[j];

        spline.segments[i-1].joints[j].coefficients[0] = trajectory_in.points[i-1].positions[j] + diff * coefficients[0];
        spline.segments[i-1].joints[j].coefficients[1] = diff*coefficients[1];
        spline.segments[i-1].joints[j].coefficients[2] = diff*coefficients[2];
        spline.segments[i-1].joints[j].coefficients[3] = diff*coefficients[3];
      }
    }
    return true;
  }
}
