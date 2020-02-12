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

#ifndef JOINT_LIMITS_CONTAINER_H
#define JOINT_LIMITS_CONTAINER_H

#include "pilz_extensions/joint_limits_extension.h"

#include <map>
#include <vector>

namespace pilz
{
/**
 * @brief Container for JointLimits, essentially a map with convenience functions.
 * Adds the ability to as for limits and get a common limit that unifies all given limits
 */
class JointLimitsContainer
{
public:
  /**
   * @brief Add a limit
   * @param joint_name  Name if the joint this limit belongs to
   * @param joint_limit Limit of the joint
   * @return true if the limit was added, false
   *         if joint_limit.has_deceleration_limit && joint_limit.max_deceleration >= 0
   */
  bool addLimit(const std::string& joint_name, pilz_extensions::JointLimit joint_limit);

  /**
   * @brief Check if there is a limit for a joint with the given name in this container
   * @param joint_name Name of the joint
   */
  bool hasLimit(const std::string& joint_name) const;

  /**
   * @brief Get Number of limits in the container
   * @return Number of limits in the container
   */
  size_t getCount() const;

  /**
   * @brief Returns wether the container is empty
   * @return true if empty, false otherwise
   */
  bool empty() const;

  /**
   * @brief Returns joint limit fusion of all(position, velocity, acceleration, deceleration) limits for all joint.
   * There are cases where the most strict limit of all limits is needed.
   * If there are no matching limits, the flag has_[position|velocity|...]_limits is set to false.
   *
   * @return joint limit
   */
  pilz_extensions::JointLimit getCommonLimit() const;

  /**
   * @brief Returns joint limit fusion of all(position, velocity, acceleration, deceleration) limits for given joints.
   * There are cases where the most strict limit of all limits is needed.
   * If there are no matching limits, the flag has_[position|velocity|...]_limits is set to false.
   *
   * @param joint_names
   * @return joint limit
   * @throws std::out_of_range if a joint limit with this name does not exist
   */
  pilz_extensions::JointLimit getCommonLimit(const std::vector<std::string> &joint_names) const;

  /**
   * @brief getLimit get the limit for the given joint name
   * @param joint_name
   * @return joint limit
   * @throws std::out_of_range if a joint limit with this name does not exist
   */
  pilz_extensions::JointLimit getLimit(const std::string& joint_name) const;

  /**
   * @brief ConstIterator to the underlying data structure
   * @return
   */
  std::map<std::string, pilz_extensions::JointLimit>::const_iterator begin() const;


  /**
   * @brief ConstIterator to the underlying data structure
   * @return
   */
  std::map<std::string, pilz_extensions::JointLimit>::const_iterator end() const;

  /**
   * @brief verify position limit of single joint
   * @param joint_name
   * @param joint_position
   * @return
   */
  bool verifyVelocityLimit(const std::string& joint_name,
                           const double& joint_velocity) const;

  /**
   * @brief verify position limit of single joint
   * @param joint_name
   * @param joint_position
   * @return
   */
  bool verifyPositionLimit(const std::string& joint_name,
                           const double& joint_position) const;

  /**
   * @brief verify position limits of multiple joints
   * @param joint_names
   * @param joint_positions
   * @return
   */
  bool verifyPositionLimits(const std::vector<std::string> &joint_names,
                            const std::vector<double> &joint_positions) const;

private:
  /**
   * @brief update the most strict limit with given joint limit
   * @param joint_limit
   * @param common_limit the current most strict limit
   */
  static void updateCommonLimit(const pilz_extensions::JointLimit& joint_limit,
                                pilz_extensions::JointLimit& common_limit);

protected:
  /// Actual container object containing the data
  std::map<std::string, pilz_extensions::JointLimit> container_;
};
}

#endif // JOINT_LIMITS_CONTAINER_H
