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

#ifndef PTP_H
#define PTP_H

#include <stdexcept>

#include "basecmd.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Data class storing all information regarding a Ptp command.
 */
template <class StartType, class GoalType>
class Ptp : public BaseCmd<StartType, GoalType>
{
public:
  Ptp() : BaseCmd<StartType, GoalType>()
  {
  }

private:
  virtual std::string getPlannerId() const override;
};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class GoalType>
inline std::string Ptp<StartType, GoalType>::getPlannerId() const
{
  return "PTP";
}
}

#endif  // PTP_H
