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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>


#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_extensions/joint_limits_interface_extension.h"

#include "pilz_trajectory_generation/joint_limits_validator.h"

bool pilz::JointLimitsValidator::validateAllPositionLimitsEqual(const pilz::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz::JointLimitsValidator::positionEqual, joint_limits);
}

bool pilz::JointLimitsValidator::validateAllVelocityLimitsEqual(const pilz::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz::JointLimitsValidator::velocityEqual, joint_limits);
}

bool pilz::JointLimitsValidator::validateAllAccelerationLimitsEqual(const pilz::JointLimitsContainer &joint_limits)
{
  return validateWithEqualFunc(&pilz::JointLimitsValidator::accelerationEqual, joint_limits);
}

bool pilz::JointLimitsValidator::validateAllDecelerationLimitsEqual(const pilz::JointLimitsContainer &joint_limits)
{
  return validateWithEqualFunc(&pilz::JointLimitsValidator::decelerationEqual, joint_limits);
}

bool pilz::JointLimitsValidator::validateWithEqualFunc
  (bool (*eq_func)(const pilz_extensions::JointLimit&, const pilz_extensions::JointLimit&),
   const pilz::JointLimitsContainer& joint_limits)
{
  // If there are no joint_limits it is considered equal
  if(joint_limits.empty())
  {
    return true;
  }

  pilz_extensions::JointLimit reference = joint_limits.begin()->second;

  for(auto it = std::next(joint_limits.begin()); it != joint_limits.end(); ++it)
  {
      if(!eq_func(reference, it->second))
      {
        return false;
      }
  }

  return true;
}

bool pilz::JointLimitsValidator::positionEqual(const pilz_extensions::JointLimit &lhs,
                                               const pilz_extensions::JointLimit &rhs)
{
  // Return false if .has_velocity_limits differs
  if(lhs.has_position_limits != rhs.has_position_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if(lhs.has_position_limits && ((lhs.max_position != rhs.max_position) || (lhs.min_position != rhs.min_position))){
    return false;
  }
  return true;
}

bool pilz::JointLimitsValidator::velocityEqual(const pilz_extensions::JointLimit &lhs,
                                               const pilz_extensions::JointLimit &rhs)
{
  // Return false if .has_velocity_limits differs
  if(lhs.has_velocity_limits != rhs.has_velocity_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if(lhs.has_velocity_limits && (lhs.max_velocity != rhs.max_velocity)){
    return false;
  }

  return true;
}

bool pilz::JointLimitsValidator::accelerationEqual(const pilz_extensions::JointLimit &lhs,
                                                   const pilz_extensions::JointLimit &rhs)
{
  // Return false if .has_acceleration_limits differs
  if(lhs.has_acceleration_limits != rhs.has_acceleration_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if(lhs.has_acceleration_limits && (lhs.max_acceleration != rhs.max_acceleration)){
    return false;
  }

  return true;
}

bool pilz::JointLimitsValidator::decelerationEqual(const pilz_extensions::JointLimit &lhs,
                                                   const pilz_extensions::JointLimit &rhs)
{
  // Return false if .has_acceleration_limits differs
  if(lhs.has_deceleration_limits != rhs.has_deceleration_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if(lhs.has_deceleration_limits && (lhs.max_deceleration != rhs.max_deceleration)){
    return false;
  }

  return true;
}

