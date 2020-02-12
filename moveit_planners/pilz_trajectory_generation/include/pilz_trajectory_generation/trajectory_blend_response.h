/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRAJECTORY_BLEND_RESPONSE_H
#define TRAJECTORY_BLEND_RESPONSE_H

#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace pilz
{

struct TrajectoryBlendResponse
{
  // The name of the group of joints on which this blender is operating
  std::string group_name;

  // Resulted robot trajectories after blending
  robot_trajectory::RobotTrajectoryPtr first_trajectory;
  robot_trajectory::RobotTrajectoryPtr blend_trajectory;
  robot_trajectory::RobotTrajectoryPtr second_trajectory;

  // Error code
  moveit_msgs::MoveItErrorCodes error_code;
};

}

#endif // TRAJECTORY_BLEND_RESPONSE_H
