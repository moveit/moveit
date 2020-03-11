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

#ifndef CARTESIANPATHCONSTRAINTSBUILDER_H
#define CARTESIANPATHCONSTRAINTSBUILDER_H

#include <string>

#include <moveit_msgs/Constraints.h>

#include "cartesianconfiguration.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Helper class to build moveit_msgs::Constraints from a
 * given configuration.
 */
class CartesianPathConstraintsBuilder
{
public:
  CartesianPathConstraintsBuilder& setConstraintName(const std::string& constraint_name);
  CartesianPathConstraintsBuilder& setConfiguration(const CartesianConfiguration& configuration);

  moveit_msgs::Constraints toPathConstraints() const;

private:
  std::string constraint_name_;
  CartesianConfiguration configuration_;
};

inline CartesianPathConstraintsBuilder&
CartesianPathConstraintsBuilder::setConstraintName(const std::string& constraint_name)
{
  constraint_name_ = constraint_name;
  return *this;
}

inline CartesianPathConstraintsBuilder&
CartesianPathConstraintsBuilder::setConfiguration(const CartesianConfiguration& configuration)
{
  configuration_ = configuration;
  return *this;
}

inline moveit_msgs::Constraints CartesianPathConstraintsBuilder::toPathConstraints() const
{
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.link_name = configuration_.getLinkName();
  pos_constraint.constraint_region.primitive_poses.push_back(configuration_.getPose());

  moveit_msgs::Constraints path_constraints;
  path_constraints.name = constraint_name_;
  path_constraints.position_constraints.push_back(pos_constraint);
  return path_constraints;
}
}

#endif  // CARTESIANPATHCONSTRAINTSBUILDER_H
