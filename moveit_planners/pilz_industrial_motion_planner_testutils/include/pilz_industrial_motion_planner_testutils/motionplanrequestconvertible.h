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

#ifndef MOTIONPLANREQUESTCONVERTIBLE_H
#define MOTIONPLANREQUESTCONVERTIBLE_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Interface class to express that a derived class can be converted
 * into a planning_interface::MotionPlanRequest.
 */
class MotionPlanRequestConvertible
{
public:
  virtual planning_interface::MotionPlanRequest toRequest() const = 0;
};
}

#endif  // MOTIONPLANREQUESTCONVERTIBLE_H
