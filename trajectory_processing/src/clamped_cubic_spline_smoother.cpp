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

#include <trajectory_processing/clamped_cubic_spline_smoother.h>
#include <ros/ros.h>

namespace trajectory_processing
{
bool ClampedCubicTrajectorySmoother::smooth(const trajectory_msgs::JointTrajectory& trajectory_in,
                                            trajectory_msgs::JointTrajectory& trajectory_out)
{
  int length = trajectory_in.points.size();
  trajectory_out = trajectory_in;

  if (!checkTrajectoryConsistency(trajectory_out))
    return false;

  if (length<3)
    return true;

  if (length <= MAX_TRIDIAGONAL_SOLVER_ELEMENTS)
  {
    smoothSegment(trajectory_out.points);
  }
  else
  {
    ROS_ERROR("ClampedCubicTrajectorySmoother: does not support trajectory lengths > %d due to numerical instability.", MAX_TRIDIAGONAL_SOLVER_ELEMENTS);
    return false;
  }

  return true;
}

bool ClampedCubicTrajectorySmoother::smoothSegment(std::vector<trajectory_msgs::JointTrajectoryPoint>& wpts) const
{
  int length = wpts.size();
  int num_joints = wpts[0].positions.size();
  if (length < 3)
    return true;

  std::vector<double> intervals(length-1);

  // generate time intervals:
  for (int i=0; i<length-1; i++)
    intervals[i] = (wpts[i+1].time_from_start - wpts[i].time_from_start).toSec();

  // arrays for tridiagonal matrix
  std::vector<double> a(length-2);
  std::vector<double> b(length-2);
  std::vector<double> c(length-2);
  std::vector<double> d(length-2);
  std::vector<double> x(length-2);

  // for each joint:
  for (int j=0; j<num_joints; j++)
  {
    a[0] = 0.0;
    c[length-3] = 0.0;
    for (int i=0; i<length-2; i++)
    {
      c[i] = intervals[i];
      if (i<length-3)
        a[i+1] = intervals[i+2];
      b[i] = 2.0*(intervals[i] + intervals[i+1]);
      d[i] = (3.0/(intervals[i]*intervals[i+1]))*
          ((intervals[i]*intervals[i])*(wpts[i+2].positions[j]-wpts[i+1].positions[j]) +
              (intervals[i+1]*intervals[i+1])*(wpts[i+1].positions[j]-wpts[i].positions[j]));
    }
    d[0] -= wpts[0].velocities[j]*intervals[1];
    d[length-3] -= wpts[length-1].velocities[j]*intervals[length-3];

    tridiagonalSolve(a, b, c, d, x);
    for (int i=0; i<length-2; i++)
      wpts[i+1].velocities[j] = x[i];
  }
  return true;
}

}

