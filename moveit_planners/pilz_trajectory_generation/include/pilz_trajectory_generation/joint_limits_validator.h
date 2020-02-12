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

#ifndef JOINT_LIMITS_VALIDATOR_H
#define JOINT_LIMITS_VALIDATOR_H

#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_trajectory_generation/joint_limits_container.h"

namespace pilz {

/**
 * @brief Validates the equality of all limits inside a container
 */
class JointLimitsValidator{
  public:
    /**
     * @brief Validates that the position limits of all limits are equal
     * @param joint_limits the joint limits
     * @return true if all are equal
     * @note always returns true if has_position_limits=false for all limits, or if the size of joint_limits is 0 or 1
     */
    static bool validateAllPositionLimitsEqual(const pilz::JointLimitsContainer& joint_limits);

    /**
     * @brief Validates that the velocity of all limits is equal
     * @param joint_limits the joint limits
     * @return true if all are equal
     * @note always returns true if has_velocity_limits=false for all limits, or if the size of joint_limits is 0 or 1
     */
    static bool validateAllVelocityLimitsEqual(const pilz::JointLimitsContainer& joint_limits);

    /**
     * @brief Validates that the acceleration of all limits is equal
     * @param joint_limits the joint limits
     * @return true if all are equal
     * @note always returns true if has_acceleration_limits=false for all limits, or if size of joint_limits is 0 or 1
     */
    static bool validateAllAccelerationLimitsEqual(const pilz::JointLimitsContainer& joint_limits);

    /**
     * @brief Validates that the deceleration of all limits is equal
     * @param joint_limits the joint limits
     * @return true if all are equal
     * @note always returns true if has_acceleration_limits=false for all limits, or if size of joint_limits is 0 or 1
     */
    static bool validateAllDecelerationLimitsEqual(const pilz::JointLimitsContainer& joint_limits);

  private:
    static bool validateWithEqualFunc(bool (*eq_func)(const pilz_extensions::JointLimit&,
                                                      const pilz_extensions::JointLimit&),
                                                      const pilz::JointLimitsContainer& joint_limits);

    static bool positionEqual(const pilz_extensions::JointLimit& lhs, const pilz_extensions::JointLimit& rhs);

    static bool velocityEqual(const pilz_extensions::JointLimit& lhs, const pilz_extensions::JointLimit& rhs);

    static bool accelerationEqual(const pilz_extensions::JointLimit& lhs, const pilz_extensions::JointLimit& rhs);

    static bool decelerationEqual(const pilz_extensions::JointLimit& lhs, const pilz_extensions::JointLimit& rhs);

};

/**
 * @class ValidationException
 * @brief A base class for all validations exceptions inheriting from std::runtime_exception
 */
class ValidationException: public std::runtime_error
{
  public:
    ValidationException(const std::string error_desc) : std::runtime_error(error_desc) {}
};

/**
 * @class ValidationJointMissingException
 * @brief Thrown the limits for a joint are defined in the urdf but not on the parameter server (loaded from yaml)
 *
 */
class ValidationJointMissingException: public ValidationException
{
  public:
    ValidationJointMissingException(const std::string error_desc) : ValidationException(error_desc) {}
};


/**
 * @class ValidationDifferentLimitsException
 * @brief Thrown when the limits differ
 *
 */
class ValidationDifferentLimitsException: public ValidationException
{
  public:
    ValidationDifferentLimitsException(const std::string error_desc) : ValidationException(error_desc) {}
};

/**
 * @class ValidationBoundsViolationException
 * @brief Thrown when the limits from the param server are weaker than the ones obtained from the urdf
 *
 */
class ValidationBoundsViolationException: public ValidationException
{
  public:
    ValidationBoundsViolationException(const std::string error_desc) : ValidationException(error_desc) {}
};

}

#endif // JOINT_LIMITS_VALIDATOR_H
