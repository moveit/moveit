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

/* Author: Ken Anderson */

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/JointLimits.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>

namespace trajectory_processing
{

static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 1.0;
static const double ROUNDING_THRESHOLD = 0.01;

IterativeParabolicTimeParameterization::IterativeParabolicTimeParameterization(unsigned int max_iterations,
                                                                               double max_time_change_per_it)
  : max_iterations_(max_iterations),
    max_time_change_per_it_(max_time_change_per_it)
{}

IterativeParabolicTimeParameterization::~IterativeParabolicTimeParameterization()
{}

void IterativeParabolicTimeParameterization::printPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const
{
  logDebug(  " time   [%i]= %f",i,point.time_from_start.toSec());
  if(point.positions.size() >= 7 )
  {
    logDebug(" pos_   [%i]= %f %f %f %f %f %f %f",i,
             point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
  }
  if(point.velocities.size() >= 7 )
  {
    logDebug("  vel_  [%i]= %f %f %f %f %f %f %f",i,
             point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
  }
  if(point.accelerations.size() >= 7 )
  {
    logDebug("   acc_ [%i]= %f %f %f %f %f %f %f",i,
             point.accelerations[0],point.accelerations[1],point.accelerations[2],point.accelerations[3],point.accelerations[4],point.accelerations[5],point.accelerations[6]);
  }
}

void IterativeParabolicTimeParameterization::printStats(const trajectory_msgs::JointTrajectory& trajectory,
                                                        const std::vector<moveit_msgs::JointLimits>& limits) const
{
  logDebug("jointNames= %s %s %s %s %s %s %s",
           limits[0].joint_name.c_str(),limits[1].joint_name.c_str(),limits[2].joint_name.c_str(),
           limits[3].joint_name.c_str(),limits[4].joint_name.c_str(),limits[5].joint_name.c_str(),
           limits[6].joint_name.c_str());
  logDebug("maxVelocities= %f %f %f %f %f %f %f",
           limits[0].max_velocity,limits[1].max_velocity,limits[2].max_velocity,
           limits[3].max_velocity,limits[4].max_velocity,limits[5].max_velocity,
           limits[6].max_velocity);
  logDebug("maxAccelerations= %f %f %f %f %f %f %f",
           limits[0].max_acceleration,limits[1].max_acceleration,limits[2].max_acceleration,
           limits[3].max_acceleration,limits[4].max_acceleration,limits[5].max_acceleration,
           limits[6].max_acceleration);
  // for every point in time:
  for (unsigned int i=0; i<trajectory.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint& point = trajectory.points[i];
    printPoint(point, i);
  }
}

// Applies velocity
void IterativeParabolicTimeParameterization::applyVelocityConstraints(trajectory_msgs::JointTrajectory& trajectory,
                                                                      const std::vector<moveit_msgs::JointLimits>& limits,
                                                                      std::vector<double> &time_diff) const
{

  // aleeper: This function computes the minimum time needed for each trajectory
  //          point by (forward_difference/ v_max), and recording the minimum time in
  //          the time_diff vector, which will be used later.
  const unsigned int num_points = trajectory.points.size();
  const unsigned int num_joints = trajectory.joint_names.size();

  // Initial values
  for (unsigned int i=0; i<num_points; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point = trajectory.points[i];
    point.velocities.resize(num_joints);
    point.accelerations.resize(num_joints);
  }

  for (unsigned int i=0; i<num_points-1; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point1 = trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint& point2 = trajectory.points[i+1];

    // Get velocity min/max
    for (unsigned int j=0; j<num_joints; ++j)
    {
      double v_max = 1.0;

      if( limits[j].has_velocity_limits )
      {
        v_max = limits[j].max_velocity;
      }

      const double dq1 = point1.positions[j];
      const double dq2 = point2.positions[j];
      const double t_min = std::abs(dq2-dq1) / v_max;

      if( t_min > time_diff[i] )
      {
        time_diff[i] = t_min;
      }
    }
  }
}

// Iteratively expand dt1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h
double IterativeParabolicTimeParameterization::findT1( const double dq1,
                                                       const double dq2,
                                                       double dt1,
                                                       const double dt2,
                                                       const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);

  while( std::abs( a ) > a_max )
  {
    v1 = (dq1)/dt1;
    v2 = (dq2)/dt2;
    a = 2.0*(v2-v1)/(dt1+dt2);
    dt1 *= mult_factor;
  }

  return dt1;
}

double IterativeParabolicTimeParameterization::findT2(const double dq1,
                                                      const double dq2,
                                                      const double dt1,
                                                      double dt2,
                                                      const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1)/dt1;
  double v2 = (dq2)/dt2;
  double a = 2.0*(v2-v1)/(dt1+dt2);

  while( std::abs( a ) > a_max )
  {
    v1 = (dq1)/dt1;
    v2 = (dq2)/dt2;
    a = 2.0*(v2-v1)/(dt1+dt2);
    dt2 *= mult_factor;
  }

  return dt2;
}

// Takes the time differences, and updates the values in the trajectory.
void updateTrajectory(trajectory_msgs::JointTrajectory& trajectory,
                      const std::vector<double>& time_diff,
                      const std::map<std::string, double>& velocity_map)
{

  // aleeper: Where does this magic number (0.2) come from?
  //          Let's try only building in a delay if we aren't doing stamped execution.
  double time_sum = velocity_map.empty() ? 0.2 : 0.0;
  unsigned int num_joints = trajectory.joint_names.size();
  const unsigned int num_points = trajectory.points.size();

  // Error check
  if(time_diff.size() < 1)
    return;

  bool has_start_velocity = !velocity_map.empty();

  // Times
  trajectory.points[0].time_from_start = ros::Duration(time_sum);
  for (unsigned int i=1; i<num_points; ++i)
  {
    time_sum += time_diff[i-1];
    trajectory.points[i].time_from_start = ros::Duration(time_sum);
  }

  // Return if there is only one point in the trajectory!
  if(trajectory.points.size() <= 1) return;

  /*
  // Velocities
  for (unsigned int j=0; j<num_joints; ++j)
  {
  trajectory.points[num_points-1].velocities[j] = 0.0;
  }
  for (unsigned int i=0; i<num_points-1; ++i)
  {
  trajectory_msgs::JointTrajectoryPoint& point1 = trajectory.points[i];
  trajectory_msgs::JointTrajectoryPoint& point2 = trajectory.points[i+1];
  for (unsigned int j=0; j<num_joints; ++j)
  {
  const double dq1 = point1.positions[j];
  const double dq2 = point2.positions[j];
  const double & dt1 = time_diff[i];
  const double v1 = (dq2-dq1)/(dt1);
  point1.velocities[j] = (i==0 && has_start_velocity) ? start_state.joint_state.velocity[j] : v1;
  }
  }
  */

  // Accelerations
  for (unsigned int i=0; i<num_points; ++i)
  {
    for (unsigned int j=0; j<num_joints; ++j)
    {
      double q1;
      double q2;
      double q3;
      double dt1;
      double dt2;

      if(i==0)
      { // First point
        q1 = trajectory.points[i+1].positions[j];
        q2 = trajectory.points[i].positions[j];
        q3 = trajectory.points[i+1].positions[j];
        dt1 = time_diff[i];
        dt2 = time_diff[i];
      }
      else if(i < num_points-1)
      { // middle points
        q1 = trajectory.points[i-1].positions[j];
        q2 = trajectory.points[i].positions[j];
        q3 = trajectory.points[i+1].positions[j];
        dt1 = time_diff[i-1];
        dt2 = time_diff[i];
      }
      else
      { // last point
        q1 = trajectory.points[i-1].positions[j];
        q2 = trajectory.points[i].positions[j];
        q3 = trajectory.points[i-1].positions[j];
        dt1 = time_diff[i-1];
        dt2 = time_diff[i-1];
      }

      double v1, v2, a;

      bool start_velocity = false;
      if(dt1 == 0.0 || dt2 == 0.0) {
        v1 = 0.0;
        v2 = 0.0;
        a = 0.0;
      } else {
        if(i==0 && has_start_velocity)
        {
          std::map<std::string, double>::const_iterator it = velocity_map.find(trajectory.joint_names[j]);
          if(it != velocity_map.end())
          {
            start_velocity = true;
            v1 = it->second;
          }
        }
        v1 = start_velocity ? v1 : (q2-q1)/dt1;
        //v2 = (q3-q2)/dt2;
        v2 = start_velocity ? v1 : (q3-q2)/dt2; // Needed to ensure continuous velocity for first point
        a = 2*(v2-v1)/(dt1+dt2);
      }
      trajectory.points[i].velocities[j] = (v2+v1)/2;
      trajectory.points[i].accelerations[j] = a;
    }
  }
}


// Applies Acceleration constraints
void IterativeParabolicTimeParameterization::applyAccelerationConstraints(const trajectory_msgs::JointTrajectory& trajectory,
                                                                          const std::vector<moveit_msgs::JointLimits>& limits,
                                                                          std::vector<double> & time_diff,
                                                                          const std::map<std::string, double>& velocity_map) const
{
  const unsigned int num_points = trajectory.points.size();
  const unsigned int num_joints = trajectory.joint_names.size();
  int num_updates = 0;
  int iteration = 0;
  bool backwards = false;
  double q1;
  double q2;
  double q3;
  double dt1;
  double dt2;
  double v1;
  double v2;
  double a;

  bool has_start_velocity = !velocity_map.empty();

  do
  {
    num_updates = 0;
    iteration++;

    // In this case we iterate through the joints on the outer loop.
    // This is so that any time interval increases have a chance to get propogated through the trajectory
    for (unsigned int j = 0; j < num_joints ; ++j)
    {
      // Loop forwards, then backwards
      for( int count=0; count<2; count++)
      {
        //logDebug("applyAcceleration: Iteration %i backwards=%i joint=%i", iteration, backwards, j);
        //updateTrajectory(trajectory, time_diff);
        //printStats(trajectory);

        for (unsigned int i=0; i<num_points-1; ++i)
        {
          unsigned int index = i;
          if(backwards)
          {
            index = (num_points-1)-i;
          }

          // Get acceleration limits
          double a_max = 1.0;
          if( limits[j].has_acceleration_limits )
          {
            a_max = limits[j].max_acceleration;
          }

          if(index==0)
          {     // First point
            q1 = trajectory.points[index+1].positions[j];
            q2 = trajectory.points[index].positions[j];
            q3 = trajectory.points[index+1].positions[j];
            dt1 = time_diff[index];
            dt2 = time_diff[index];
            assert(!backwards);
          }
          else if(index < num_points-1)
          { // middle points
            q1 = trajectory.points[index-1].positions[j];
            q2 = trajectory.points[index].positions[j];
            q3 = trajectory.points[index+1].positions[j];
            dt1 = time_diff[index-1];
            dt2 = time_diff[index];
          }
          else
          { // last point - careful, there are only numpoints-1 time intervals
            q1 = trajectory.points[index-1].positions[j];
            q2 = trajectory.points[index].positions[j];
            q3 = trajectory.points[index-1].positions[j];
            dt1 = time_diff[index-1];
            dt2 = time_diff[index-1];
            assert(backwards);
          }

          if(dt1 == 0.0 || dt2 == 0.0) {
            v1 = 0.0;
            v2 = 0.0;
            a = 0.0;
          } else {
            bool start_velocity = false;
            if(index==0 && has_start_velocity)
            {
              std::map<std::string, double>::const_iterator it = velocity_map.find(trajectory.joint_names[j]);
              if(it != velocity_map.end())
              {
                start_velocity = true;
                v1 = it->second;
              }
            }
            v1 = start_velocity ? v1 : (q2-q1)/dt1;
            v2 = (q3-q2)/dt2;
            a = 2*(v2-v1)/(dt1+dt2);
          }

          if( std::abs( a ) > a_max + ROUNDING_THRESHOLD )
          {
            if(!backwards)
            {
              dt2 = std::min( dt2+max_time_change_per_it_, findT2( q2-q1, q3-q2, dt1, dt2, a_max) );
              time_diff[index] = dt2;
            }
            else
            {
              dt1 = std::min( dt1+max_time_change_per_it_, findT1( q2-q1, q3-q2, dt1, dt2, a_max) );
              time_diff[index-1] = dt1;
            }
            num_updates++;

            if(dt1 == 0.0 || dt2 == 0.0) {
              v1 = 0.0;
              v2 = 0.0;
              a = 0.0;
            } else {
              v1 = (q2-q1)/dt1;
              v2 = (q3-q2)/dt2;
              a = 2*(v2-v1)/(dt1+dt2);
            }
          }
        }
        backwards = !backwards;
      }
    }
    //logDebug("applyAcceleration: num_updates=%i", num_updates);
  } while(num_updates > 0 && iteration < max_iterations_);
}

bool IterativeParabolicTimeParameterization::computeTimeStamps(trajectory_msgs::JointTrajectory& trajectory,
                                                               const std::vector<moveit_msgs::JointLimits>& limits) const
{
  static const moveit_msgs::RobotState start_state;
  return computeTimeStamps(trajectory, limits, start_state);
}

bool IterativeParabolicTimeParameterization::computeTimeStamps(trajectory_msgs::JointTrajectory& trajectory,
                                                               const std::vector<moveit_msgs::JointLimits>& limits,
                                                               const moveit_msgs::RobotState& start_state) const
{
  bool success = true;
  const std::size_t num_points = trajectory.points.size();
  std::vector<double> time_diff(num_points, 0.0);       // the time difference between adjacent points

  std::map<std::string, double> velocity_map;
  if (start_state.joint_state.name.size() == start_state.joint_state.velocity.size())
  {
    //logInform("We seem to have velocity data; populating...");
    for (std::size_t i = 0; i < start_state.joint_state.name.size(); ++i)
    {
      //logInform("joint: [%s], velocity: [%.3f]", start_state.joint_state.name[i].c_str(), start_state.joint_state.velocity[i] );
      velocity_map[start_state.joint_state.name[i]] = start_state.joint_state.velocity[i];
    }
  }

  applyVelocityConstraints(trajectory, limits, time_diff);
  applyAccelerationConstraints(trajectory, limits, time_diff, velocity_map);

  updateTrajectory(trajectory, time_diff, velocity_map);
  //  printStats(trajectory, limits);
  return success;
}

bool IterativeParabolicTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                                               const moveit_msgs::RobotState& start_state) const
{
  if (trajectory.empty())
    return true;

  if (!trajectory.getGroup())
  {
    logError("It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  // \todo this lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // \todo this is inefficient  (we should avoid conversions back & forth)
  moveit_msgs::RobotTrajectory msg;
  trajectory.getRobotTrajectoryMsg(msg);
  const std::vector<moveit_msgs::JointLimits> &jlim = trajectory.getGroup()->getVariableLimits();
  if (computeTimeStamps(msg.joint_trajectory, jlim, start_state))
  {
    trajectory.setRobotTrajectoryMsg(trajectory.getFirstWayPoint(), msg);
    return true;
  }
  else
    return false;
}

bool IterativeParabolicTimeParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory) const
{
  static const moveit_msgs::RobotState start_state;
  return computeTimeStamps(trajectory, start_state);
}

}
