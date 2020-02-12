/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PLANCOMPONENTSBUILDER_H
#define PLANCOMPONENTSBUILDER_H

#include <string>
#include <memory>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "pilz_trajectory_generation/trajectory_functions.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/trajectory_blender.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

using TipFrameFunc_t = std::function<const std::string&(const std::string&)>;

// List of exceptions which can be thrown by the PlanComponentsBuilder class.
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoBlenderSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoTipFrameFunctionSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoRobotModelSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(BlendingFailedException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @brief Helper class to encapsulate the merge and blend process of
 * trajectories.
 */
class PlanComponentsBuilder
{
public:
  /**
   * @brief Sets the blender used to blend two trajectories.
   */
  void setBlender(std::unique_ptr<pilz::TrajectoryBlender> blender);

  /**
   * @brief Sets the robot model needed to create new trajectory elements.
   */
  void setModel(const moveit::core::RobotModelConstPtr &model);

  /**
   * @brief Appends the specified trajectory to the trajectory container
   * under construction.
   *
   * The appending complies to the following rules:
   * - A trajectory is simply added/attached to the previous trajectory:
   *      - if they are from the same group and
   *      - if the specified blend_radius is zero.
   * - A trajectory is blended together with the previous trajectory:
   *      - if they are from the same group and
   *      - if the specified blend_radius is GREATER than zero.
   * - A new trajectory element is created and the given trajectory is
   * appended/attached to the newly created empty trajectory:
   *      - if the given and previous trajectory are from different groups.
   *
   * @param other Trajectories which has to be added to the trajectory container
   * under construction.
   *
   * @param blend_radius The blending radius between the previous and the
   * specified trajectory.
   */
  void append(const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius);

  /**
   * @brief Clears the trajectory container under construction.
   */
  void reset();

  /**
   * @return The final trajectory container which results from the append calls.
   */
  std::vector<robot_trajectory::RobotTrajectoryPtr> build() const;

private:
  void blend(const robot_trajectory::RobotTrajectoryPtr& other,
             const double blend_radius);

private:
  /**
   * @brief Appends a trajectory to a result trajectory leaving out the
   * first point, if it matches the last point of the result trajectory.
   *
   * @note Controllers, so far at least the
   * ros_controllers::JointTrajectoryController require a timewise strictly
   * increasing trajectory. If through appending the last point of the
   * original trajectory gets repeated, it is removed here.
   */
  static void appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory &result,
                                           const robot_trajectory::RobotTrajectory &source);

private:
  //! Blender used to blend two trajectories.
  std::unique_ptr<pilz::TrajectoryBlender> blender_;

  //! Robot model needed to create new trajectory container elements.
  moveit::core::RobotModelConstPtr model_;

  //! The previously added trajectory.
  robot_trajectory::RobotTrajectoryPtr traj_tail_;

  //! The trajectory container under construction.
  std::vector<robot_trajectory::RobotTrajectoryPtr> traj_cont_;

private:
  //! Constant to check for equality of variables of two RobotState instances.
  static constexpr double ROBOT_STATE_EQUALITY_EPSILON = 1e-4;
};

inline void PlanComponentsBuilder::setBlender(std::unique_ptr<pilz::TrajectoryBlender> blender)
{
  blender_ = std::move(blender);
}

inline void PlanComponentsBuilder::setModel(const moveit::core::RobotModelConstPtr &model)
{
  model_ = model;
}

inline void PlanComponentsBuilder::reset()
{
  traj_tail_ = nullptr;
  traj_cont_.clear();
}


}

#endif // PLANCOMPONENTSBUILDER_H
