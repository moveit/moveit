/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROBOTSTATEMSGCONVERTIBLE_H
#define ROBOTSTATEMSGCONVERTIBLE_H

#include <moveit_msgs/RobotState.h>
#include <moveit/robot_model/robot_model.h>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Interface class to express that a derived class can be converted
 * into a moveit_msgs::RobotState.
 */
class RobotStateMsgConvertible
{
public:
  virtual moveit_msgs::RobotState toMoveitMsgsRobotState() const = 0;
};
}

#endif  // ROBOTSTATEMSGCONVERTIBLE_H
