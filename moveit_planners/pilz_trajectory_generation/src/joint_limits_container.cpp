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

#include "pilz_trajectory_generation/joint_limits_container.h"

#include "ros/ros.h"
#include <stdexcept>

namespace pilz
{

bool JointLimitsContainer::addLimit(const std::string &joint_name, pilz_extensions::JointLimit joint_limit)
{
  if(joint_limit.has_deceleration_limits && joint_limit.max_deceleration >= 0)
  {
    ROS_ERROR_STREAM("joint_limit.max_deceleration MUST be negative!");
    return false;
  }
  const auto& insertion_result { container_.insert(std::pair<std::string, pilz_extensions::JointLimit>(joint_name,
                                                                                                       joint_limit)) };
  if (!insertion_result.second)
  {
    ROS_ERROR_STREAM("joint_limit for joint " << joint_name << " already contained.");
    return false;
  }
  return true;
}

bool JointLimitsContainer::hasLimit(const std::string &joint_name) const
{
  return container_.find(joint_name) != container_.end();
}

size_t JointLimitsContainer::getCount() const
{
  return container_.size();
}

bool JointLimitsContainer::empty() const
{
  return container_.empty();
}

pilz_extensions::JointLimit JointLimitsContainer::getCommonLimit() const
{
  pilz_extensions::JointLimit common_limit;
  for (const auto & limit : container_)
  {
    updateCommonLimit(limit.second, common_limit);
  }
  return common_limit;
}

pilz_extensions::JointLimit JointLimitsContainer::getCommonLimit(const std::vector<std::string> &joint_names) const
{
  pilz_extensions::JointLimit common_limit;
  for(const auto& joint_name : joint_names)
  {
    updateCommonLimit(container_.at(joint_name), common_limit);
  }
  return common_limit;
}

pilz_extensions::JointLimit JointLimitsContainer::getLimit(const std::string &joint_name) const
{
  return container_.at(joint_name);
}

std::map<std::string, pilz_extensions::JointLimit>::const_iterator JointLimitsContainer::begin() const
{
  return container_.begin();
}

std::map<std::string, pilz_extensions::JointLimit>::const_iterator JointLimitsContainer::end() const
{
  return container_.end();
}

bool JointLimitsContainer::verifyVelocityLimit(const std::string &joint_name,
                                                     const double &joint_velocity) const
{
  return (!(hasLimit(joint_name)
          && getLimit(joint_name).has_velocity_limits
          && fabs(joint_velocity) > getLimit(joint_name).max_velocity));
}


bool JointLimitsContainer::verifyPositionLimit(const std::string &joint_name,
                                                     const double &joint_position) const
{
  return (!( hasLimit(joint_name)
             && getLimit(joint_name).has_position_limits
             && (joint_position < getLimit(joint_name).min_position
                || joint_position > getLimit(joint_name).max_position) ) );
}


bool JointLimitsContainer::verifyPositionLimits(const std::vector<std::string> &joint_names,
                                                    const std::vector<double> &joint_positions) const
{
  if(joint_names.size() != joint_positions.size())
  {
    throw std::out_of_range("joint_names vector has a different size than joint_positions vector.");
  }

  for(std::size_t i=0; i<joint_names.size(); ++i)
  {
    if(!verifyPositionLimit(joint_names.at(i), joint_positions.at(i)))
    {
      return false;
    }
  }

  return true;
}

void JointLimitsContainer::updateCommonLimit(const pilz_extensions::JointLimit& joint_limit,
                                                   pilz_extensions::JointLimit& common_limit)
{
  // check position limits
  if(joint_limit.has_position_limits)
  {
    double min_position = joint_limit.min_position;
    double max_position = joint_limit.max_position;

    common_limit.min_position = (!common_limit.has_position_limits) ? min_position
                                                                    : std::max(common_limit.min_position, min_position);
    common_limit.max_position = (!common_limit.has_position_limits) ? max_position
                                                                    : std::min(common_limit.max_position, max_position);
    common_limit.has_position_limits = true;
  }

  // check velocity limits
  if(joint_limit.has_velocity_limits)
  {
    double max_velocity = joint_limit.max_velocity;
    common_limit.max_velocity = (!common_limit.has_velocity_limits) ? max_velocity
                                                                    : std::min(common_limit.max_velocity, max_velocity);
    common_limit.has_velocity_limits = true;
  }

  // check acceleration limits
  if(joint_limit.has_acceleration_limits)
  {
    double max_acc = joint_limit.max_acceleration;
    common_limit.max_acceleration = (!common_limit.has_acceleration_limits) ? max_acc
                                                                            : std::min(common_limit.max_acceleration,
                                                                                       max_acc);
    common_limit.has_acceleration_limits = true;
  }

  // check deceleration limits
  if(joint_limit.has_deceleration_limits)
  {
    double max_dec = joint_limit.max_deceleration;
    common_limit.max_deceleration = (!common_limit.has_deceleration_limits) ? max_dec
                                                                            : std::max(common_limit.max_deceleration,
                                                                                       max_dec);
    common_limit.has_deceleration_limits = true;
  }
}

}  // namespace pilz
