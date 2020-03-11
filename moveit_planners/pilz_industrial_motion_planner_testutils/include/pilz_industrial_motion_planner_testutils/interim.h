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

#ifndef INTERIMAXILIARY_H
#define INTERIMAXILIARY_H

#include "circauxiliary.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Class to define a point on the circle on which the robot is supposed
 * to move via circ command.
 */
template <class ConfigType, class BuilderType>
class Interim : public CircAuxiliary<ConfigType, BuilderType>
{
private:
  virtual std::string getConstraintName() const override;
};

template <class ConfigType, class BuilderType>
std::string Interim<ConfigType, BuilderType>::getConstraintName() const
{
  return "interim";
}
}

#endif  // INTERIMAXILIARY_H
