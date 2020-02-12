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

#ifndef CARTESIAN_TRAJECTORY_POINT_H
#define CARTESIAN_TRAJECTORY_POINT_H

#include <ros/duration.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>

namespace pilz
{

struct CartesianTrajectoryPoint
{
  geometry_msgs::Pose pose;
  geometry_msgs::Twist velocity;
  geometry_msgs::Twist acceleartion;
  ros::Duration time_from_start;
};

}

#endif // CARTESIAN_TRAJECTORY_POINT_H
