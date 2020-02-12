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

#ifndef TRAJECTORY_GENERATOR_CIRC_H
#define TRAJECTORY_GENERATOR_CIRC_H

#include <eigen3/Eigen/Eigen>
#include <kdl/path.hpp>
#include <kdl/velocityprofile.hpp>

#include "pilz_trajectory_generation/trajectory_generator.h"

using namespace pilz_trajectory_generation;

namespace pilz
{

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircleNoPlane, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircleToSmall, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CenterPointDifferentRadius, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircTrajectoryConversionFailure, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(UnknownPathConstraintName, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoPositionConstraints, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoPrimitivePose, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(UnknownLinkNameOfAuxiliaryPoint, moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NumberOfConstraintsMismatch, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircJointMissingInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircInverseForGoalIncalculable, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a trajectory generator of arcs in Cartesian space.
 * The arc is specified by a start pose, a goal pose and a interim point on the arc,
 * or a point as the center of the circle which forms the arc. Complete circle is not
 * covered by this generator.
 */
class TrajectoryGeneratorCIRC : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of CIRC Trajectory Generator.
   *
   * @param planner_limits Limits in joint and Cartesian spaces
   *
   * @throw TrajectoryGeneratorInvalidLimitsException
   *
   */
  TrajectoryGeneratorCIRC(const robot_model::RobotModelConstPtr& robot_model,
                          const pilz::LimitsContainer& planner_limits);

  virtual ~TrajectoryGeneratorCIRC() = default;

private:
  virtual void cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest &req) const override;

  virtual void extractMotionPlanInfo(const planning_interface::MotionPlanRequest &req,
                                     MotionPlanInfo &info) const final override;

  virtual void plan(const planning_interface::MotionPlanRequest &req,
                    const MotionPlanInfo& plan_info,
                    const double& sampling_time,
                    trajectory_msgs::JointTrajectory& joint_trajectory) override;

  /**
   * @brief Construct a KDL::Path object for a Cartesian path of an arc.
   *
   * @return A unique pointer of the path object, null_ptr in case of an error.
   *
   * @throws CircleNoPlane if the plane in which the circle resides,
   * could not be determined.
   *
   * @throws CircleToSmall if the specified circ radius is to small.
   *
   * @throws CenterPointDifferentRadius if the distances between start-center
   * and goal-center are different.
   */
  std::unique_ptr<KDL::Path> setPathCIRC(const MotionPlanInfo &info) const;


};

}

#endif // TRAJECTORY_GENERATOR_CIRC_H
