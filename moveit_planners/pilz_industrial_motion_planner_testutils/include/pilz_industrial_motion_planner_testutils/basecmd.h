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

#ifndef BASECMD_H
#define BASECMD_H

#include <string>

#include "motioncmd.h"

namespace pilz_industrial_motion_planner_testutils
{
template <class StartType, class GoalType>
class BaseCmd : public MotionCmd
{
public:
  BaseCmd() : MotionCmd()
  {
  }

  virtual ~BaseCmd() = default;

public:
  virtual planning_interface::MotionPlanRequest toRequest() const override;

  void setStartConfiguration(StartType start);
  void setGoalConfiguration(GoalType goal);

  StartType& getStartConfiguration();
  const StartType& getStartConfiguration() const;

  GoalType& getGoalConfiguration();
  const GoalType& getGoalConfiguration() const;

private:
  virtual std::string getPlannerId() const = 0;

protected:
  GoalType goal_;
  StartType start_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setStartConfiguration(StartType start)
{
  start_ = start;
}

template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setGoalConfiguration(GoalType goal)
{
  goal_ = goal;
}

template <class StartType, class GoalType>
inline StartType& BaseCmd<StartType, GoalType>::getStartConfiguration()
{
  return start_;
}

template <class StartType, class GoalType>
inline const StartType& BaseCmd<StartType, GoalType>::getStartConfiguration() const
{
  return start_;
}

template <class StartType, class GoalType>
inline GoalType& BaseCmd<StartType, GoalType>::getGoalConfiguration()
{
  return goal_;
}

template <class StartType, class GoalType>
inline const GoalType& BaseCmd<StartType, GoalType>::getGoalConfiguration() const
{
  return goal_;
}

template <class StartType, class GoalType>
planning_interface::MotionPlanRequest BaseCmd<StartType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = getPlannerId();
  req.group_name = this->planning_group_;

  req.max_velocity_scaling_factor = this->vel_scale_;
  req.max_acceleration_scaling_factor = this->acc_scale_;

  req.start_state = this->start_.toMoveitMsgsRobotState();
  req.goal_constraints.push_back(this->goal_.toGoalConstraints());

  return req;
}
}

#endif  // BASECMD_H
