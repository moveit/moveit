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

#include "pilz_industrial_motion_planner_testutils/robotconfiguration.h"

#include <stdexcept>

namespace pilz_industrial_motion_planner_testutils
{
RobotConfiguration::RobotConfiguration() : RobotStateMsgConvertible(), GoalConstraintMsgConvertible()
{
}

RobotConfiguration::RobotConfiguration(const std::string& group_name)
  : RobotStateMsgConvertible(), GoalConstraintMsgConvertible(), group_name_(group_name)
{
}

RobotConfiguration::RobotConfiguration(const std::string& group_name,
                                       const moveit::core::RobotModelConstPtr& robot_model)
  : RobotStateMsgConvertible(), GoalConstraintMsgConvertible(), group_name_(group_name), robot_model_(robot_model)
{
  if (robot_model && (!robot_model_->hasJointModelGroup(group_name_)))
  {
    std::string msg{ "Specified robot model does not contain specified group \"" };
    msg.append(group_name).append("\"");
    throw std::invalid_argument(msg);
  }
}

}  // namespace pilz_industrial_motion_planner_testutils
