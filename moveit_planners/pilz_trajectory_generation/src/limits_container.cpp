/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pilz_trajectory_generation/limits_container.h"

pilz::LimitsContainer::LimitsContainer():
  has_joint_limits_(false),
  has_cartesian_limits_(false)
{

}

bool pilz::LimitsContainer::hasJointLimits() const
{
  return has_joint_limits_;
}

void pilz::LimitsContainer::setJointLimits(pilz::JointLimitsContainer &joint_limits)
{
  has_joint_limits_ = true;
  joint_limits_  = joint_limits;
}

const pilz::JointLimitsContainer& pilz::LimitsContainer::getJointLimitContainer() const
{
  return joint_limits_;
}

bool pilz::LimitsContainer::hasFullCartesianLimits() const
{
  return (has_cartesian_limits_ &&
          cartesian_limit_.hasMaxTranslationalVelocity() &&
          cartesian_limit_.hasMaxTranslationalAcceleration() &&
          cartesian_limit_.hasMaxTranslationalDeceleration() &&
          cartesian_limit_.hasMaxRotationalVelocity() );
}

void pilz::LimitsContainer::setCartesianLimits(pilz::CartesianLimit &cartesian_limit)
{
  has_cartesian_limits_ = true;
  cartesian_limit_ = cartesian_limit;
}

const pilz::CartesianLimit& pilz::LimitsContainer::getCartesianLimits() const
{
  return cartesian_limit_;
}
