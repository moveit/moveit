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

#ifndef TRAJECTORY_GENERATOR_PTP_H
#define TRAJECTORY_GENERATOR_PTP_H

#include "eigen3/Eigen/Eigen"
#include "pilz_trajectory_generation/trajectory_generator.h"
#include "pilz_trajectory_generation/velocity_profile_atrap.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

using namespace pilz_trajectory_generation;

namespace pilz
{

//TODO date type of units

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PtpVelocityProfileSyncFailed, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PtpNoIkSolutionForGoalPose, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a point-to-point trajectory generator based on
 * VelocityProfile_ATrap.
 */
class TrajectoryGeneratorPTP : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of PTP Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: a map of joint limits information
   */
  TrajectoryGeneratorPTP(const robot_model::RobotModelConstPtr& robot_model,
                         const pilz::LimitsContainer& planner_limits);

  virtual ~TrajectoryGeneratorPTP() = default;

private:

  virtual void extractMotionPlanInfo(const planning_interface::MotionPlanRequest& req,
                                     MotionPlanInfo& info) const override;

  /**
   * @brief plan ptp joint trajectory with zero start velocity
   * @param start_pos
   * @param goal_pos
   * @param joint_trajectory
   * @param group_name
   * @param velocity_scaling_factor
   * @param acceleration_scaling_factor
   * @param sampling_time
   */
  void planPTP(const std::map<std::string, double>& start_pos,
               const std::map<std::string, double>& goal_pos,
               trajectory_msgs::JointTrajectory& joint_trajectory,
               const std::string &group_name,
               const double& velocity_scaling_factor,
               const double& acceleration_scaling_factor,
               const double& sampling_time);

  virtual void plan(const planning_interface::MotionPlanRequest &req,
                    const MotionPlanInfo& plan_info,
                    const double& sampling_time,
                    trajectory_msgs::JointTrajectory& joint_trajectory) override;

private:
  const double MIN_MOVEMENT = 0.001;
  pilz::JointLimitsContainer joint_limits_;
  // most strict joint limits for each group
  std::map<std::string, pilz_extensions::JointLimit> most_strict_limits_;
};

}

#endif // TRAJECTORY_GENERATOR_PTP_H
