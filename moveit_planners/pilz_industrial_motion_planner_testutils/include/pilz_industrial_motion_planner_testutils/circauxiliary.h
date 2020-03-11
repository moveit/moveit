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

#ifndef CIRCAUXILIARY_H
#define CIRCAUXILIARY_H

#include <string>

#include <moveit_msgs/Constraints.h>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Base class to define an auxiliary point needed to specify
 * circ commands.
 */
template <class ConfigType, class BuilderType>
class CircAuxiliary
{
public:
  void setConfiguration(const ConfigType& auxiliary_config);
  ConfigType& getConfiguration();
  const ConfigType& getConfiguration() const;

public:
  moveit_msgs::Constraints toPathConstraints() const;

private:
  virtual std::string getConstraintName() const = 0;

protected:
  ConfigType auxiliary_config_;
};

template <class ConfigType, class BuilderType>
void CircAuxiliary<ConfigType, BuilderType>::setConfiguration(const ConfigType& auxiliary_config)
{
  auxiliary_config_ = auxiliary_config;
}

template <class ConfigType, class BuilderType>
inline ConfigType& CircAuxiliary<ConfigType, BuilderType>::getConfiguration()
{
  return auxiliary_config_;
}

template <class ConfigType, class BuilderType>
inline const ConfigType& CircAuxiliary<ConfigType, BuilderType>::getConfiguration() const
{
  return auxiliary_config_;
}

template <class ConfigType, class BuilderType>
inline moveit_msgs::Constraints CircAuxiliary<ConfigType, BuilderType>::toPathConstraints() const
{
  return BuilderType().setConstraintName(getConstraintName()).setConfiguration(getConfiguration()).toPathConstraints();
}
}

#endif  // CIRCAUXILIARY_H
