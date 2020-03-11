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

#ifndef CIRC_H
#define CIRC_H

#include <memory>

#include "basecmd.h"
#include "circauxiliary.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Data class storing all information regarding a Circ command.
 */
template <class StartType, class AuxiliaryType, class GoalType>
class Circ : public BaseCmd<StartType, GoalType>
{
public:
  Circ() : BaseCmd<StartType, GoalType>()
  {
  }

public:
  void setAuxiliaryConfiguration(AuxiliaryType auxiliary);
  AuxiliaryType& getAuxiliaryConfiguration();
  const AuxiliaryType& getAuxiliaryConfiguration() const;

public:
  planning_interface::MotionPlanRequest toRequest() const override;

private:
  virtual std::string getPlannerId() const override;

private:
  AuxiliaryType auxiliary_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class AuxiliaryType, class GoalType>
inline void Circ<StartType, AuxiliaryType, GoalType>::setAuxiliaryConfiguration(AuxiliaryType auxiliary)
{
  auxiliary_ = auxiliary;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline std::string Circ<StartType, AuxiliaryType, GoalType>::getPlannerId() const
{
  return "CIRC";
}

template <class StartType, class AuxiliaryType, class GoalType>
inline planning_interface::MotionPlanRequest Circ<StartType, AuxiliaryType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req{ BaseCmd<StartType, GoalType>::toRequest() };
  req.path_constraints = auxiliary_.toPathConstraints();

  return req;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline AuxiliaryType& Circ<StartType, AuxiliaryType, GoalType>::getAuxiliaryConfiguration()
{
  return auxiliary_;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline const AuxiliaryType& Circ<StartType, AuxiliaryType, GoalType>::getAuxiliaryConfiguration() const
{
  return auxiliary_;
}
}

#endif  // CIRC_H
