/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2017, Ken Anderson
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
 *********************************************************************/

/* Author: Ken Anderson */

#ifndef MOVEIT_TRAJECTORY_PROCESSING_ITERATIVE_SPLINE_PARAMETERIZATION__
#define MOVEIT_TRAJECTORY_PROCESSING_ITERATIVE_SPLINE_PARAMETERIZATION__

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{
/// \brief This class  modifies the timestamps of a trajectory to respect
/// velocity and acceleration constraints.
/// The second and second-last point locations need to move to allow 
/// velocity and acceleration matching.  
/// By default, two points are added to leave the original trajectory unaffected.
class IterativeSplineParameterization
{
public:
  IterativeSplineParameterization(double max_time_change_per_it = .01, bool add_points = true);
  ~IterativeSplineParameterization();

  bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
                         const double max_acceleration_scaling_factor = 1.0) const;

private:
  double max_time_change_per_it_;  /// @brief maximum allowed time change per iteration in seconds
  bool add_points_;  /// @brief if true, add two points to trajectory (first and last segments).  If false, move the 2nd and 2nd last points.
};
}

#endif
