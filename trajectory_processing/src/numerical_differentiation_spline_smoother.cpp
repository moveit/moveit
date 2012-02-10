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

#include <trajectory_processing/trajectory_smoother.h>
#include <trajectory_processing/trajectory_processing_utils.h>
#include <trajectory_processing/numerical_differentiation_spline_smoother.h>

namespace trajectory_processing
{

bool NumericalDifferentiationSplineSmoother::smooth(const trajectory_msgs::JointTrajectory& trajectory_in,
                                                    trajectory_msgs::JointTrajectory& trajectory_out,
                                                    const std::vector<moveit_msgs::JointLimits>& limits)
{
  bool success = true;
  int size = trajectory_in.points.size();
  int num_traj = trajectory_in.joint_names.size();
  trajectory_out = trajectory_in;

  if (!checkTrajectoryConsistency(trajectory_out))
    return false;

  // keep the first and last velocities intact

  // for every point in time:
  for (int i=1; i<size-1; ++i)
  {
    double dt1 = (trajectory_in.points[i].time_from_start - trajectory_in.points[i-1].time_from_start).toSec();
    double dt2 = (trajectory_in.points[i+1].time_from_start - trajectory_in.points[i].time_from_start).toSec();

    // for every (joint) trajectory
    for (int j=0; j<num_traj; ++j)
    {
      double dx1 = trajectory_in.points[i].positions[j] - trajectory_in.points[i-1].positions[j];
      double dx2 = trajectory_in.points[i+1].positions[j] - trajectory_in.points[i].positions[j];

      double v1 = dx1/dt1;
      double v2 = dx2/dt2;

      trajectory_out.points[i].velocities[j] = 0.5*(v1 + v2);
    }
  }

  // all accelerations are 0 for now:
  for (int i=0; i<size; i++)
    for (int j=0; j<num_traj; j++)
      trajectory_out.points[i].accelerations[j] = 0.0;

  return success;
}

}
