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

#include <trajectory_processing/linear_spline_velocity_scaler.h>
#include <trajectory_processing/trajectory_processing_utils.h>

namespace trajectory_processing
{

bool LinearSplineVelocityScaler::smooth(const trajectory_msgs::JointTrajectory& trajectory_in,
                                        trajectory_msgs::JointTrajectory& trajectory_out,
                                        const std::vector<moveit_msgs::JointLimits>& limits)
{
  trajectory_processing::LinearTrajectory traj;
  spline_msgs::SplineTrajectory spline;
  bool success = traj.parameterize(trajectory_in,limits,spline);
  if(!success)
    return false;

  trajectory_out = trajectory_in;
  if (!checkTrajectoryConsistency(trajectory_out))
    return false;

  std::vector<double> times;
  times.resize(spline.segments.size()+1);
  times[0] = 0.0;
  for(int i=0; i< (int) spline.segments.size(); i++)
    times[i+1] = times[i] + spline.segments[i].duration.toSec(); 

  trajectory_msgs::JointTrajectory joint_traj;
  trajectory_processing::sampleSplineTrajectory(spline,times,joint_traj);
  trajectory_out = joint_traj;
  trajectory_out.joint_names = trajectory_in.joint_names;

  return success;
}


}
