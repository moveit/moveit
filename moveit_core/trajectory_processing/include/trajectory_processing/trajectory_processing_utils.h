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

/** \author Mrinal Kalakrishnan */


#ifndef TRAJECTORY_PROCESSING_UTILS_H_
#define TRAJECTORY_PROCESSING_UTILS_H_

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <spline_msgs/LSPBTrajectoryMsg.h>
#include <spline_msgs/SplineTrajectory.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <float.h>
namespace trajectory_processing
{


template <typename T>
void differentiate(const std::vector<T>& x, std::vector<T>& xd);

/**
 * \brief Solves the tridiagonal system of equations, Ax = d
 * A is an n by n square matrix which consists of:
 *      diagonal b (0 ... n-1)
 *      upper diagonal c (0 ... n-2)
 *      lower diagonal a (0 ... n-1)
 *
 * The solution goes into x. Time complexity: O(n)
 *
 * WARNING: modifies input arrays!!
 */
template <typename T>
void tridiagonalSolve(std::vector<T>& a,
                      std::vector<T>& b,
                      std::vector<T>& c,
                      std::vector<T>& d,
                      std::vector<T>& x);

/////////////////////////// inline implementations follow //////////////////////////////

template <typename T>
void differentiate(const std::vector<T>& x, std::vector<T>& xd)
{
  int size = x.size();
  xd.resize(size-1);
  for (int i=0; i<size-1; ++i)
  {
    xd[i] = x[i+1] - x[i];
  }
}

template <typename T>
void tridiagonalSolve(std::vector<T>& a,
                      std::vector<T>& b,
                      std::vector<T>& c,
                      std::vector<T>& d,
                      std::vector<T>& x)
{
  int n = (int)d.size();

  x.resize(n);

  // forward elimination
  for (int i=1; i<n; i++)
  {
    double m = a[i] / b[i-1];
    b[i] -= m*c[i-1];
    d[i] -= m*d[i-1];
  }

  // backward substitution
  x[n-1] = d[n-1]/b[n-1];
  for (int i=n-2; i>=0; i--)
  {
    x[i] = (d[i] - c[i]*x[i+1])/b[i];
  }
}

/**
 * \brief Ensures the consistency of a WaypointTrajWithLimits message, and resizes vel and acc arrays
 *
 * Ensures that the number of (joint) names matches the number of positions in each waypoint
 * Resizes the velocities and accelerations for every waypoint, filling in zeros if necessary
 * Ensures that time is strictly increasing
 */
template <typename T>
bool checkTrajectoryConsistency(T& waypoint_traj)
{
  unsigned int length = waypoint_traj.points.size();
  unsigned int num_joints = waypoint_traj.joint_names.size();

  double prev_time = -1.0;

  for (unsigned int i=0; i<length; i++)
  {
    if (waypoint_traj.points[i].positions.size() != num_joints)
    {
      ROS_ERROR("Number of positions (%d) at trajectory index %d doesn't match number of joint names (%d)",
                (int) waypoint_traj.points[i].positions.size(), (int) i, (int) num_joints);
      return false;
    }
    if (waypoint_traj.points[i].time_from_start.toSec() < prev_time)
    {
      ROS_ERROR("Time of waypoint at trajectory index %d (%f) is not greater than or equal to the previous time (%f)",
                (int) i, waypoint_traj.points[i].time_from_start.toSec(), prev_time);
      return false;
    }
    if(waypoint_traj.points[i].time_from_start.toSec() < 0.0)
    {
      ROS_ERROR("Time of waypoint at trajectory index %d (%f) is negative",
                (int) i, waypoint_traj.points[i].time_from_start.toSec());
      return false;
    }      
    prev_time = waypoint_traj.points[i].time_from_start.toSec();
    if(waypoint_traj.points[i].velocities.size() != waypoint_traj.points[i].positions.size()) 
      waypoint_traj.points[i].velocities.resize(num_joints, 0.0);
    if(waypoint_traj.points[i].accelerations.size() != waypoint_traj.points[i].positions.size()) 
      waypoint_traj.points[i].accelerations.resize(num_joints, 0.0);
  }
  return true;
}

/*! 
  \brief An internal helper function that uses information about the joint to compute proper angular differences if the joint is a rotary joint with limits
              
  \return if(limit.angle_wraparound)
  return angles::shortest_angular_distance(from,to)
  else
  return (to-from)
  \param from The start position of the joint
  \param to The end position of the joint
  \param limit A set of position, velocity and acceleration limits for this particular joint
*/
inline double jointDiff(const double &from, const double &to, const moveit_msgs::JointLimits &limit)
{
  if(!limit.has_position_limits)
  {
    return angles::shortest_angular_distance(from,to);
  }
  else
  {
    return (to-from);
  }
}





/*! 
  \brief Internal function that helps determine which spline segment corresponds to an input time. 
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory
  \param time Time at which trajectory needs to be sampled. time = 0.0 corresponds to start of the trajectory.
  \param spline_segment Reference to output spline segment containing the waypoint corresponding to the desired time
  \param segment_time Reference to output segment time corresponding to the waypoint in trajectory at the (absolute) input time. segment_time = 0.0 corresponds to the start of this spline segment.
  \param start_index (optional) parameter to specify the starting index to start the (linear) search for required input time
  \param end_index (optional) parameter to specify the end index for the (linear) search for required input time
*/
bool findSplineSegment(const spline_msgs::SplineTrajectory &spline,
                       const double& time, 
                       spline_msgs::SplineTrajectorySegment& spline_segment,
                       double& segment_time, 
                       int start_index=0, 
                       int end_index=-1);

bool findSplineSegment(const spline_msgs::LSPBTrajectoryMsg &spline,
                       const double& time, 
                       spline_msgs::LSPBTrajectorySegmentMsg& spline_segment,
                       double& segment_time, 
                       int start_index=0, 
                       int end_index=-1);


/*! 
  \brief Get the total time for the provided spline trajectory
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param t A reference that will be filled in with the total time for the input spline trajectory.
*/
bool getTotalTime(const spline_msgs::SplineTrajectory &spline, 
                  double &t);


/*! 
  \brief Get the total time for the provided spline trajectory
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param t A reference that will be filled in with the total time for the input spline trajectory.
*/
bool getTotalTime(const spline_msgs::LSPBTrajectoryMsg &spline, 
                  double &t);

bool sampleSplineTrajectory(const spline_msgs::SplineTrajectorySegment &spline, 
                            const double& input_time, 
                            trajectory_msgs::JointTrajectoryPoint &point_out);

bool sampleSplineTrajectory(const spline_msgs::LSPBTrajectorySegmentMsg &spline, 
                            const double& input_time, 
                            trajectory_msgs::JointTrajectoryPoint &point_out);

bool sampleSplineTrajectory(const spline_msgs::SplineTrajectory& spline, 
                            const std::vector<double> &times, 
                            trajectory_msgs::JointTrajectory& traj_out);

bool sampleSplineTrajectory(const spline_msgs::LSPBTrajectoryMsg& spline, 
                            const std::vector<double> &times, 
                            trajectory_msgs::JointTrajectory& traj_out);

/*! 
  \brief Write the trajectory out to a file after sampling at the specified times
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param times The set of times (in seconds) where the trajectory needs to be sampled, time = 0.0 corresponds to the start of the trajectory.
  \param filename The string representation of the name of the file where the trajectory should be written
*/
bool write(const spline_msgs::SplineTrajectory &spline, 
           const std::vector<double> &times, 
           const std::string &filename);

/*! 
  \brief Write the spline representation of the trajectory out
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param filename The string representation of the name of the file where the spline representation of the trajectory should be written
*/
bool writeSpline(const spline_msgs::SplineTrajectory &spline, 
                 const std::string &filename);

/*! 
  \brief Write the trajectory out to a file after sampling at the specified times
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param times The set of times (in seconds) where the trajectory needs to be sampled, time = 0.0 corresponds to the start of the trajectory.
  \param filename The string representation of the name of the file where the trajectory should be written
*/
bool write(const spline_msgs::LSPBTrajectoryMsg &spline, 
           const std::vector<double> &times, 
           const std::string &filename);

/*! 
  \brief Write the spline representation of the trajectory out
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param filename The string representation of the name of the file where the spline representation of the trajectory should be written
*/
bool writeSpline(const spline_msgs::LSPBTrajectoryMsg &spline, 
                 const std::string &filename);

static const double MAX_ALLOWABLE_TIME = FLT_MAX;/** Internal constant */

}
#endif /* SPLINE_SMOOTHER_UTILS_H_ */
