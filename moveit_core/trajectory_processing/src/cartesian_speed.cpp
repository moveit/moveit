/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Benjamin Scholz
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

/* Author: Benjamin Scholz */

#include <moveit/trajectory_processing/cartesian_speed.h>

namespace trajectory_processing
{
bool setMaxCartesianEndEffectorSpeed(robot_trajectory::RobotTrajectory& trajectory, const double speed,
                                     std::string end_effector)
{
  if (speed <= 0.0)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_processing.cartesian_speed", "End effector speed must be greater than 0.");
    return false;
  }
  if (end_effector.empty())
  {
    std::vector<std::string> tips;
    trajectory.getGroup()->getEndEffectorTips(tips);
    if (tips.empty())
    {
      ROS_ERROR_STREAM_NAMED("trajectory_processing.cartesian_speed",
                             "No end effector defined for group attached to trajectory.");
      return false;
    }
    end_effector = tips[0];
  }

  size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints == 0)
    return false;

  robot_state::RobotStatePtr kinematic_state = trajectory.getFirstWayPointPtr();

  // do forward kinematics to get cartesian positions of end effector for current waypoint
  Eigen::Isometry3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
  Eigen::Isometry3d next_end_effector_state;
  double euclidean_distance, new_time_diff, old_time_diff;
  std::vector<double> time_diff(num_waypoints - 1, 0.0);

  for (size_t i = 0; i < num_waypoints - 1; i++)
  {
    // get state for next waypoint
    kinematic_state = trajectory.getWayPointPtr(i + 1);

    // do forward kinematics to get cartesian positions of end effector for next waypoint
    next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);

    // get euclidean distance between the two waypoints
    euclidean_distance =
        pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) +
                pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) +
                pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2),
            0.5);

    new_time_diff = (euclidean_distance / speed);
    old_time_diff = trajectory.getWayPointDurationFromPrevious(i + 1);

    // if constraints allow save the new time difference between waypoints
    if (new_time_diff > old_time_diff)
    {
      time_diff[i] = new_time_diff;
    }
    else
    {
      ROS_WARN_ONCE_NAMED("trajectory_processing.cartesian_speed",
                          "Desired cartesian end-effector speed is not reached because of joint velocity constraints.");
      time_diff[i] = old_time_diff;
    }
    // update the current_end_effector_state for next iteration
    current_end_effector_state = next_end_effector_state;
  }
  // update time stamps, velocities and accelerations of the trajectory
  updateTrajectory(trajectory, time_diff);
  return true;
}
}  // namespace trajectory_processing
