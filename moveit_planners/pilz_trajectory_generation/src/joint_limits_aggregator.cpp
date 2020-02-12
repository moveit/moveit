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

#include "pilz_trajectory_generation/joint_limits_aggregator.h"

#include "pilz_extensions/joint_limits_interface_extension.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <vector>

using namespace pilz_extensions;

pilz::JointLimitsContainer pilz::JointLimitsAggregator::getAggregatedLimits(const ros::NodeHandle& nh,
                                                             const std::vector<const moveit::core::JointModel*>& joint_models)
{
  JointLimitsContainer container;

  ROS_INFO_STREAM("Reading limits from namespace " << nh.getNamespace());

  // Iterate over all joint models and generate the map
  for(auto joint_model : joint_models)
  {
    JointLimit joint_limit;

    // If there is something defined for the joint on the parameter server
    if(pilz_extensions::joint_limits_interface::getJointLimits(joint_model->getName(), nh, joint_limit))
    {
      if(joint_limit.has_position_limits)
      {
        checkPositionBoundsThrowing(joint_model, joint_limit);
      }
      else
      {
        updatePositionLimitFromJointModel(joint_model, joint_limit);
      }


      if(joint_limit.has_velocity_limits)
      {
        checkVelocityBoundsThrowing(joint_model, joint_limit);
      }
      else
      {
        updateVelocityLimitFromJointModel(joint_model, joint_limit);
      }
    }
    else
    {
      // If there is nothing defined for this joint on the parameter server just update the values by the values of
      // the urdf

      updatePositionLimitFromJointModel(joint_model, joint_limit);
      updateVelocityLimitFromJointModel(joint_model, joint_limit);
    }

    // Update max_deceleration if no max_acceleration has been set
    if(joint_limit.has_acceleration_limits && !joint_limit.has_deceleration_limits){
      joint_limit.max_deceleration = -joint_limit.max_acceleration;
      joint_limit.has_deceleration_limits = true;
    }

    // Insert the joint limit into the map
    container.addLimit(joint_model->getName(), joint_limit);
  }

  return container;
}

void pilz::JointLimitsAggregator::updatePositionLimitFromJointModel(const moveit::core::JointModel* joint_model,
                                                                    JointLimit& joint_limit)
{
  switch(joint_model->getVariableBounds().size())
  {
  // LCOV_EXCL_START
  case 0:  
    ROS_ERROR_STREAM("no bounds set for joint " << joint_model->getName());
    break;
  // LCOV_EXCL_STOP
  case 1:
    joint_limit.has_position_limits = joint_model->getVariableBounds()[0].position_bounded_;
    joint_limit.min_position = joint_model->getVariableBounds()[0].min_position_;
    joint_limit.max_position = joint_model->getVariableBounds()[0].max_position_;
    break;
  // LCOV_EXCL_START
  default:
    ROS_ERROR_STREAM("Multi-DOF-Joints not supported. The robot won't move.");
    joint_limit.has_position_limits = true;
    joint_limit.min_position = 0;
    joint_limit.max_position = 0;
    break;
  // LCOV_EXCL_STOP
  }

  ROS_DEBUG_STREAM("Limit(" << joint_model->getName() << " min:" << joint_limit.min_position << " max:" << joint_limit.max_position);
}

void pilz::JointLimitsAggregator::updateVelocityLimitFromJointModel(const moveit::core::JointModel* joint_model,
                                                                    JointLimit& joint_limit)
{
  switch(joint_model->getVariableBounds().size())
  {
  // LCOV_EXCL_START
  case 0:
    ROS_ERROR_STREAM("no bounds set for joint " << joint_model->getName());
    break;
  // LCOV_EXCL_STOP
  case 1:
    joint_limit.has_velocity_limits = joint_model->getVariableBounds()[0].velocity_bounded_;
    joint_limit.max_velocity = joint_model->getVariableBounds()[0].max_velocity_;
    break;
  // LCOV_EXCL_START
  default:
    ROS_ERROR_STREAM("Multi-DOF-Joints not supported. The robot won't move.");
    joint_limit.has_velocity_limits = true;
    joint_limit.max_velocity = 0;
    break;
  // LCOV_EXCL_STOP
  }
}

void pilz::JointLimitsAggregator::checkPositionBoundsThrowing(const moveit::core::JointModel* joint_model,
                                                              const pilz_extensions::JointLimit& joint_limit)
{
  // Check min position
  if(!joint_model->satisfiesPositionBounds(&joint_limit.min_position))
  {
    throw AggregationBoundsViolationException("min_position of " + joint_model->getName()
                                              + " violates min limit from URDF");
  }

  // Check max position
  if(!joint_model->satisfiesPositionBounds(&joint_limit.max_position))
  {
    throw AggregationBoundsViolationException("max_position of " + joint_model->getName()
                                              + " violates max limit from URDF");
  }
}

void pilz::JointLimitsAggregator::checkVelocityBoundsThrowing(const moveit::core::JointModel* joint_model,
                                                              const JointLimit &joint_limit)
{
  // Check min position
  if(!joint_model->satisfiesVelocityBounds(&joint_limit.max_velocity))
  {
    throw AggregationBoundsViolationException("max_velocity of " + joint_model->getName()
                                              + " violates velocity limit from URDF");
  }
}
