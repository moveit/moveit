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

#ifndef TRAJECTORY_GENERATOR_LIN_H
#define TRAJECTORY_GENERATOR_LIN_H

#include <eigen3/Eigen/Eigen>
#include <kdl/rotational_interpolation_sa.hpp>

#include "pilz_trajectory_generation/trajectory_generator.h"
#include "pilz_trajectory_generation/velocity_profile_atrap.h"

using namespace pilz_trajectory_generation;

namespace pilz
{

//TODO date type of units

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinTrajectoryConversionFailure, moveit_msgs::MoveItErrorCodes::FAILURE);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(JointNumberMismatch, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinJointMissingInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinInverseForGoalIncalculable, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a linear trajectory generator in Cartesian space.
 * The Cartesian trajetory are based on trapezoid velocity profile.
 */
class TrajectoryGeneratorLIN : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of LIN Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: robot model
   * @param planner_limits: limits in joint and Cartesian spaces
   */
  TrajectoryGeneratorLIN(const robot_model::RobotModelConstPtr& robot_model,
                         const pilz::LimitsContainer& planner_limits);

  virtual ~TrajectoryGeneratorLIN() = default;

private:

  virtual void extractMotionPlanInfo(const planning_interface::MotionPlanRequest& req,
                                     MotionPlanInfo& info) const final override;

  virtual void plan(const planning_interface::MotionPlanRequest &req,
                    const MotionPlanInfo& plan_info,
                    const double& sampling_time,
                    trajectory_msgs::JointTrajectory& joint_trajectory) override;

  /**
   * @brief construct a KDL::Path object for a Cartesian straight line
   * @return a unique pointer of the path object. null_ptr in case of an error.
   */
  std::unique_ptr<KDL::Path> setPathLIN(const Eigen::Affine3d& start_pose,
                                        const Eigen::Affine3d& goal_pose) const;


};

}

#endif // TRAJECTORY_GENERATOR_LIN_H
