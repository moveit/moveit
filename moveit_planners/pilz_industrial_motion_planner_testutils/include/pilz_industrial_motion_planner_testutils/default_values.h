/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef DEFAULT_VALUES_H
#define DEFAULT_VALUES_H

/*
 * @brief This file contains all default values needed for testing.
 */
namespace pilz_industrial_motion_planner_testutils
{
static constexpr double DEFAULT_VEL{ 0.01 };
static constexpr double DEFAULT_ACC{ 0.01 };
static constexpr double DEFAULT_BLEND_RADIUS{ 0.01 };

static constexpr double DEFAULT_VEL_GRIPPER{ 0.5 };
static constexpr double DEFAULT_ACC_GRIPPER{ 0.8 };
}

#endif  // DEFAULT_VALUES_H
