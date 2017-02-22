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
/// \brief This class sets the timestamps of a trajectory
/// to enforce velocity, acceleration, and optionally jerk constraints.
/// Initial/final velocities and accelerations may be specified in the trajectory.
/// Velocity and acceleration limits are specified in the model.
/// A jerk constraint may be passed into the contructor, which may be
/// overridden when the model supports this in the future.
///
/// This algorithm repeatedly fits a cubic spline, adjusts the timing intervals,
/// and repeats until all constraints are satisfied.
/// When finished, each trajectory waypoint will have the time set,
/// as well as the velocities and accelerations for each joint.
/// Since we fit to a cubic spline, the position, velocity, and
/// acceleration will be continuous and within bounds.
/// The jerk will be discontinuous, but within bounds.
///
/// If jerk is disabled, then initial/final acceleration matching
/// will be disabled.
/// You would use this if you want to allow bang-bang style control
/// where the controller can go from min acceleration to max acceleration
/// instantaneously.
/// In other words, discontinuity in the acceleration curve allowed.
/// Since the acceleration can change instantly, matching the initial and final
/// acceleration is unneccessary.
/// This will get the shortest paths and should be comparable to 'optimal'
/// methods that have a trapazoidal velocity curve.
///
/// However, jerk is enabled by default.
/// To match the velocity and acceleration at the endpoints,
/// the second and second-last point locations need to move.
/// By default, two extra points are added to leave the original trajectory unaffected.
/// If points are not added, the trajectory could potentially be faster,
/// but the 2nd and 2nd-last points should be re-checked for collisions.
/// The downside of enforcing jerk is that if the limits are low,
/// there will be visible oscilation
/// around what you may expect as the 'optimal' velocity curve.
/// Increasing the jerk limits will reduce oscilation and speed up execution.
///
/// Migration notes:  If migrating from Iterative Parabolic Time Parameterization,
/// be aware that the velocity and acceleration limits are more strictly enforced
/// using this technique.
/// This means that time-parameterizing the same trajectory with the same
/// velocity and acceleration limits, will result in a longer trajectory.
/// Similarly, enforcing jerk will cause the trajectory to be longer.
/// If this is a problem, try retuning (increasing) the limits.
///
class IterativeSplineParameterization
{
public:
  IterativeSplineParameterization(bool enable_jerk = true, double max_jerk = 50.0, bool add_points = true,
                                  double max_time_change_per_it = .01);
  ~IterativeSplineParameterization();

  bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
                         const double max_acceleration_scaling_factor = 1.0,
                         const double max_jerk_scaling_factor = 1.0) const;

private:
  bool jerk_enabled_;          /// @brief If true, enable jerk and initial/final acceleration matching
  double max_jerk_;            /// @brief The maximum jerk limit.
                               /// Only active if jerk is enabled.
  bool add_points_;            /// @brief If true, add two points to trajectory (first and last segments).
                               /// If false, move the 2nd and 2nd-last points.
                               /// Only active if jerk is enabled.
  double time_change_factor_;  /// @brief multiplicative factor to change time intervals each iteration (>1.0)
};
}

#endif
