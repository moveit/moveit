/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Delft University of Technology
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
 *   * Neither the name of Delft University of Technology nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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

/* Author: Ruben Burger (r.b.burger@tudelft.nl)
 *
 * Based on iterative_time_parameterization.cpp
 */

#include <moveit/trajectory_processing/spline_parameterization.h>
#include <moveit/trajectory_processing/spline.h>
#include <moveit_msgs/JointLimits.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>

namespace trajectory_processing
{
static const double DEFAULT_VEL_MAX = 1.0;
static const double DEFAULT_ACCEL_MAX = 1.0;
static const double ROUNDING_THRESHOLD = 0.01;

SplineParameterization::SplineParameterization(unsigned int max_iterations, double max_time_change_per_it)
  : max_iterations_(max_iterations), max_time_change_per_it_(max_time_change_per_it)
{
}

SplineParameterization::~SplineParameterization()
{
}

namespace
{
void printPoint(const trajectory_msgs::JointTrajectoryPoint& point, std::size_t i)
{
  logDebug(" time   [%i]= %f", i, point.time_from_start.toSec());
  if (point.positions.size() >= 7)
  {
    logDebug(" pos_   [%i]= %f %f %f %f %f %f %f", i, point.positions[0], point.positions[1], point.positions[2],
             point.positions[3], point.positions[4], point.positions[5], point.positions[6]);
  }
  if (point.velocities.size() >= 7)
  {
    logDebug("  vel_  [%i]= %f %f %f %f %f %f %f", i, point.velocities[0], point.velocities[1], point.velocities[2],
             point.velocities[3], point.velocities[4], point.velocities[5], point.velocities[6]);
  }
  if (point.accelerations.size() >= 7)
  {
    logDebug("   acc_ [%i]= %f %f %f %f %f %f %f", i, point.accelerations[0], point.accelerations[1],
             point.accelerations[2], point.accelerations[3], point.accelerations[4], point.accelerations[5],
             point.accelerations[6]);
  }
}

void printStats(const trajectory_msgs::JointTrajectory& trajectory, const std::vector<moveit_msgs::JointLimits>& limits)
{
  logDebug("jointNames= %s %s %s %s %s %s %s", limits[0].joint_name.c_str(), limits[1].joint_name.c_str(),
           limits[2].joint_name.c_str(), limits[3].joint_name.c_str(), limits[4].joint_name.c_str(),
           limits[5].joint_name.c_str(), limits[6].joint_name.c_str());
  logDebug("maxVelocities= %f %f %f %f %f %f %f", limits[0].max_velocity, limits[1].max_velocity,
           limits[2].max_velocity, limits[3].max_velocity, limits[4].max_velocity, limits[5].max_velocity,
           limits[6].max_velocity);
  logDebug("maxAccelerations= %f %f %f %f %f %f %f", limits[0].max_acceleration, limits[1].max_acceleration,
           limits[2].max_acceleration, limits[3].max_acceleration, limits[4].max_acceleration,
           limits[5].max_acceleration, limits[6].max_acceleration);
  // for every point in time:
  for (std::size_t i = 0; i < trajectory.points.size(); ++i)
    printPoint(trajectory.points[i], i);
}
}

// Applies velocity
void SplineParameterization::applyVelocityConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                                      std::vector<double>& time_diff,
                                                      const double max_velocity_scaling_factor) const
{
  const robot_model::JointModelGroup* group = rob_trajectory.getGroup();
  const std::vector<std::string>& vars = group->getVariableNames();
  const std::vector<int>& idx = group->getVariableIndexList();
  const robot_model::RobotModel& rmodel = group->getParentModel();
  const int num_points = rob_trajectory.getWayPointCount();

  double velocity_scaling_factor = 1.0;

  if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
    velocity_scaling_factor = max_velocity_scaling_factor;
  else if (max_velocity_scaling_factor == 0.0)
    logDebug("A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.", velocity_scaling_factor);
  else
    logWarn("Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.", max_velocity_scaling_factor,
            velocity_scaling_factor);

  for (int i = 0; i < num_points - 1; ++i)
  {
    const robot_state::RobotStatePtr& curr_waypoint = rob_trajectory.getWayPointPtr(i);
    const robot_state::RobotStatePtr& next_waypoint = rob_trajectory.getWayPointPtr(i + 1);

    for (std::size_t j = 0; j < vars.size(); ++j)
    {
      double v_max = DEFAULT_VEL_MAX;
      const robot_model::VariableBounds& b = rmodel.getVariableBounds(vars[j]);
      if (b.velocity_bounded_)
        v_max =
            std::min(fabs(b.max_velocity_ * velocity_scaling_factor), fabs(b.min_velocity_ * velocity_scaling_factor));
      const double dq1 = curr_waypoint->getVariablePosition(idx[j]);
      const double dq2 = next_waypoint->getVariablePosition(idx[j]);
      const double t_min = std::abs(dq2 - dq1) / v_max;
      if (t_min > time_diff[i])
        time_diff[i] = t_min;
    }
  }
}

// Iteratively expand dt1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, use the CubicTrajectory::quadSolve() function in cubic_trajectory.h
double SplineParameterization::findT1(const double dq1, const double dq2, double dt1, const double dt2,
                                      const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1) / dt1;
  double v2 = (dq2) / dt2;
  double a = 2.0 * (v2 - v1) / (dt1 + dt2);

  while (std::abs(a) > a_max)
  {
    v1 = (dq1) / dt1;
    v2 = (dq2) / dt2;
    a = 2.0 * (v2 - v1) / (dt1 + dt2);
    dt1 *= mult_factor;
  }

  return dt1;
}

double SplineParameterization::findT2(const double dq1, const double dq2, const double dt1, double dt2,
                                      const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (dq1) / dt1;
  double v2 = (dq2) / dt2;
  double a = 2.0 * (v2 - v1) / (dt1 + dt2);

  while (std::abs(a) > a_max)
  {
    v1 = (dq1) / dt1;
    v2 = (dq2) / dt2;
    a = 2.0 * (v2 - v1) / (dt1 + dt2);
    dt2 *= mult_factor;
  }

  return dt2;
}

namespace
{
// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.
void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory, const std::vector<double>& time_diff)
{
  // Error check
  if (time_diff.empty())
    return;

  robot_state::RobotStatePtr curr_waypoint;

  const robot_model::JointModelGroup* group = rob_trajectory.getGroup();
  const std::vector<std::string>& vars = group->getVariableNames();
  const std::vector<int>& idx = group->getVariableIndexList();

  int num_points = rob_trajectory.getWayPointCount();

  // Adding a 1ns offset for prefixing a 0-acceleration state
  rob_trajectory.setWayPointDurationFromPrevious(0, 1e-9);

  // Times
  for (int i = 1; i < num_points; ++i)
    // Update the time between the waypoints in the robot_trajectory.
    rob_trajectory.setWayPointDurationFromPrevious(i, time_diff[i - 1]);

  // Return if there is only one point in the trajectory!
  if (num_points <= 1)
    return;

  // Fitting a cubic spline
  for (std::size_t j = 0; j < vars.size(); ++j)
  {
    std::vector<double> x, y;
    for (int i = 0; i < num_points; ++i)
    {
      curr_waypoint = rob_trajectory.getWayPointPtr(i);
      y.push_back(curr_waypoint->getVariablePosition(idx[j]));
      x.push_back(rob_trajectory.getWaypointDurationFromStart(i));
    }

    tk::spline s;
    s.set_boundary(tk::spline::first_deriv, 0.0, tk::spline::first_deriv, 0.0, false);
    s.set_points(x, y);

    for (int i = 0; i < num_points; ++i)
    {
      curr_waypoint = rob_trajectory.getWayPointPtr(i);
      curr_waypoint->setVariableVelocity(idx[j], s.deriv(1, x[i]));
      curr_waypoint->setVariableAcceleration(idx[j], s.deriv(2, x[i]));
    }
  }

  // Add prefix waypoint at 50ms to ensure it is not in the past
  rob_trajectory.addPrefixWayPoint(rob_trajectory.getFirstWayPoint(), 0.05);

  // Set accelerations to 0 on first waypoint
  std::vector<double> zero_vector(vars.size(), 0.0);
  rob_trajectory.getWayPointPtr(0)->setVariableAccelerations(zero_vector);
}
}

// Applies Acceleration constraints
void SplineParameterization::applyAccelerationConstraints(robot_trajectory::RobotTrajectory& rob_trajectory,
                                                          std::vector<double>& time_diff,
                                                          const double max_acceleration_scaling_factor) const
{
  robot_state::RobotStatePtr prev_waypoint;
  robot_state::RobotStatePtr curr_waypoint;
  robot_state::RobotStatePtr next_waypoint;

  const robot_model::JointModelGroup* group = rob_trajectory.getGroup();
  const std::vector<std::string>& vars = group->getVariableNames();
  const std::vector<int>& idx = group->getVariableIndexList();
  const robot_model::RobotModel& rmodel = group->getParentModel();

  const int num_points = rob_trajectory.getWayPointCount();
  const unsigned int num_joints = group->getVariableCount();
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

  double acceleration_scaling_factor = 1.0;

  if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
    acceleration_scaling_factor = max_acceleration_scaling_factor;
  else if (max_acceleration_scaling_factor == 0.0)
    logDebug("A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.",
             acceleration_scaling_factor);
  else
    logWarn("Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.",
            max_acceleration_scaling_factor, acceleration_scaling_factor);

  do
  {
    num_updates = 0;
    iteration++;

    // In this case we iterate through the joints on the outer loop.
    // This is so that any time interval increases have a chance to get propogated through the trajectory
    for (unsigned int j = 0; j < num_joints; ++j)
    {
      // Loop forwards, then backwards
      for (int count = 0; count < 2; ++count)
      {
        for (int i = 0; i < num_points - 1; ++i)
        {
          int index = backwards ? (num_points - 1) - i : i;

          curr_waypoint = rob_trajectory.getWayPointPtr(index);

          if (index > 0)
            prev_waypoint = rob_trajectory.getWayPointPtr(index - 1);

          if (index < num_points - 1)
            next_waypoint = rob_trajectory.getWayPointPtr(index + 1);

          // Get acceleration limits
          double a_max = DEFAULT_ACCEL_MAX;
          const robot_model::VariableBounds& b = rmodel.getVariableBounds(vars[j]);
          if (b.acceleration_bounded_)
            a_max = std::min(fabs(b.max_acceleration_ * acceleration_scaling_factor),
                             fabs(b.min_acceleration_ * acceleration_scaling_factor));

          if (index == 0)
          {
            // First point
            q1 = next_waypoint->getVariablePosition(idx[j]);
            q2 = curr_waypoint->getVariablePosition(idx[j]);
            q3 = next_waypoint->getVariablePosition(idx[j]);

            dt1 = dt2 = time_diff[index];
            assert(!backwards);
          }
          else if (index < num_points - 1)
          {
            // middle points
            q1 = prev_waypoint->getVariablePosition(idx[j]);
            q2 = curr_waypoint->getVariablePosition(idx[j]);
            q3 = next_waypoint->getVariablePosition(idx[j]);

            dt1 = time_diff[index - 1];
            dt2 = time_diff[index];
          }
          else
          {
            // last point - careful, there are only numpoints-1 time intervals
            q1 = prev_waypoint->getVariablePosition(idx[j]);
            q2 = curr_waypoint->getVariablePosition(idx[j]);
            q3 = prev_waypoint->getVariablePosition(idx[j]);

            dt1 = dt2 = time_diff[index - 1];
            assert(backwards);
          }

          if (dt1 == 0.0 || dt2 == 0.0)
          {
            v1 = 0.0;
            v2 = 0.0;
            a = 0.0;
          }
          else
          {
            bool start_velocity = false;
            if (index == 0)
            {
              if (curr_waypoint->hasVelocities())
              {
                start_velocity = true;
                v1 = curr_waypoint->getVariableVelocity(idx[j]);
              }
            }
            v1 = start_velocity ? v1 : (q2 - q1) / dt1;
            v2 = (q3 - q2) / dt2;
            a = 2.0 * (v2 - v1) / (dt1 + dt2);
          }

          if (fabs(a) > a_max + ROUNDING_THRESHOLD)
          {
            if (!backwards)
            {
              dt2 = std::min(dt2 + max_time_change_per_it_, findT2(q2 - q1, q3 - q2, dt1, dt2, a_max));
              time_diff[index] = dt2;
            }
            else
            {
              dt1 = std::min(dt1 + max_time_change_per_it_, findT1(q2 - q1, q3 - q2, dt1, dt2, a_max));
              time_diff[index - 1] = dt1;
            }
            num_updates++;

            if (dt1 == 0.0 || dt2 == 0.0)
            {
              v1 = 0.0;
              v2 = 0.0;
              a = 0.0;
            }
            else
            {
              v1 = (q2 - q1) / dt1;
              v2 = (q3 - q2) / dt2;
              a = 2 * (v2 - v1) / (dt1 + dt2);
            }
          }
        }
        backwards = !backwards;
      }
    }
    // logDebug("applyAcceleration: num_updates=%i", num_updates);
  } while (num_updates > 0 && iteration < static_cast<int>(max_iterations_));
}

bool SplineParameterization::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                               const double max_velocity_scaling_factor,
                                               const double max_acceleration_scaling_factor) const
{
  if (trajectory.empty())
    return true;

  const robot_model::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    logError("It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  // this lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  const int num_points = trajectory.getWayPointCount();
  std::vector<double> time_diff(num_points - 1, 0.0);  // the time difference between adjacent points

  applyVelocityConstraints(trajectory, time_diff, max_velocity_scaling_factor);
  applyAccelerationConstraints(trajectory, time_diff, max_acceleration_scaling_factor);

  updateTrajectory(trajectory, time_diff);
  return true;
}
}
