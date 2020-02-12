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
#ifndef COMMAND_LIST_MANAGER_H
#define COMMAND_LIST_MANAGER_H

#include <string>

#include <boost/optional.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "pilz_msgs/MotionSequenceRequest.h"
#include "pilz_trajectory_generation/trajectory_blender.h"
#include "pilz_trajectory_generation/plan_components_builder.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

using RobotTrajCont = std::vector<robot_trajectory::RobotTrajectoryPtr>;

// List of exceptions which can be thrown by the CommandListManager class.
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NegativeBlendRadiusException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LastBlendRadiusNotZeroException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(StartStateSetException, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(OverlappingBlendRadiiException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PlanningPipelineException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @brief This class orchestrates the planning of single commands and
 * command lists.
 */
class CommandListManager
{
public:
  CommandListManager(const ros::NodeHandle& nh, const robot_model::RobotModelConstPtr& model);

  /**
   * @brief Generates trajectories for the specified list of motion commands.
   *
   * The folloing rules apply:
   * - If two consecutive trajectories are from the same group, they are
   * simply attached to each other, given that the blend_radius is zero.
   * - If two consecutive trajectories are from the same group, they are
   * blended together, given that the blend_radius is GREATER than zero.
   * - If two consecutive trajectories are from different groups, then
   * the second trajectory is added as new element to the result container.
   * All following trajectories are then attached to the new trajectory
   * element (until all requests are processed or until the next group change).
   *
   * @param planning_scene The planning scene to be used for trajectory generation.
   * @param req_list List of motion requests containing: PTP, LIN, CIRC
   * and/or gripper commands.
   * Please note: A request is only valid if:
   *    - All blending radii are non negative.
   *    - The blending radius of the last request is 0.
   *    - Only the first request of each group has a start state.
   *    - Non of the blending radii overlapp each other.
   *
   * Please note:
   * Starts states do not need to state the joints of all groups.
   * It is sufficient if a start state states only the joints of the group
   * which it belongs to. Starts states can even be incomplete. In this case
   * default values are set for the unset joints.
   *
   * @return Contains the calculated/generated trajectories.
   */
  RobotTrajCont solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const planning_pipeline::PlanningPipelinePtr& planning_pipeline,
                      const pilz_msgs::MotionSequenceRequest& req_list);

private:
  using MotionResponseCont = std::vector<planning_interface::MotionPlanResponse>;
  using RobotState_OptRef = boost::optional<const robot_state::RobotState& >;
  using RadiiCont = std::vector<double>;
  using GroupNamesCont = std::vector<std::string>;

private:
  /**
   * @brief Validates that two consecutive blending radii do not overlap.
   *
   * @param motion_plan_responses Container of calculated/generated trajectories.
   * @param radii Container stating the blend radii.
   */
  void checkForOverlappingRadii(const MotionResponseCont& resp_cont,
                                const RadiiCont &radii) const;

  /**
   * @brief Solve each sequence item individually.
   *
   * @param planning_scene The planning_scene to be used for trajectory generation.
   * @param req_list Container of requests for calculation/generation.
   *
   * @return Container of generated trajectories.
   */
  MotionResponseCont solveSequenceItems(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const planning_pipeline::PlanningPipelinePtr& planning_pipeline,
                                        const pilz_msgs::MotionSequenceRequest &req_list) const;

  /**
   * @return TRUE if the blending radii of specified trajectories overlap,
   * otherwise FALSE. The functions returns FALSE if both trajectories are from
   * different groups or if both trajectories are end-effector groups.
   */
  bool checkRadiiForOverlap(const robot_trajectory::RobotTrajectory& traj_A,
                            const double radii_A,
                            const robot_trajectory::RobotTrajectory& traj_B,
                            const double radii_B) const;

private:
  /**
   * @return The last RobotState of the specified group which can
   * be found in the specified vector.
   */
  static RobotState_OptRef getPreviousEndState(const MotionResponseCont &motion_plan_responses,
                                               const std::string &group_name);

  /**
   * @brief Set start state to end state of previous calculated trajectory
   * from group.
   */
  static void setStartState(const MotionResponseCont &motion_plan_responses,
                            const std::string &group_name,
                            moveit_msgs::RobotState& start_state);

  /**
   * @return Container of radii extracted from the specified request list.
   *
   * Please note:
   * This functions sets invalid blend radii to zero. Invalid blend radii are:
   * - blend radii between end-effectors and
   * - blend raddi between different groups.
   */
  static RadiiCont extractBlendRadii(const moveit::core::RobotModel &model,
                                     const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @return True in case of an invalid blend radii between specified
   * command A and B, otherwise False. Invalid blend radii are:
   * - blend radii between end-effectors and
   * - blend raddi between different groups.
   */
  static bool isInvalidBlendRadii(const moveit::core::RobotModel &model,
                                  const pilz_msgs::MotionSequenceItem& item_A,
                                  const pilz_msgs::MotionSequenceItem& item_B);

  /**
   * @brief Checks that all blend radii are greater or equal to zero.
   */
  static void checkForNegativeRadii(const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @brief Checks that last blend radius is zero.
   */
  static void checkLastBlendRadiusZero(const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @brief Checks that only the first request of the specified group has
   * a start state in the specified request list.
   */
  static void checkStartStatesOfGroup(const pilz_msgs::MotionSequenceRequest &req_list,
                                      const std::string& group_name);

  /**
   * @brief Checks that each group in the specified request list has only
   * one start state.
   */
  static void checkStartStates(const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @return Returns all group names which are present in the specified
   * request.
   */
  static GroupNamesCont getGroupNames(const pilz_msgs::MotionSequenceRequest &req_list);

private:
  //! Node handle
  ros::NodeHandle nh_;

  //! Robot model
  moveit::core::RobotModelConstPtr model_;

  //! @brief Builder to construct the container containing the final
  //! trajectories.
  PlanComponentsBuilder plan_comp_builder_;
};

inline void CommandListManager::checkLastBlendRadiusZero(const pilz_msgs::MotionSequenceRequest &req_list)
{
  if(req_list.items.back().blend_radius != 0.0)
  {
    throw LastBlendRadiusNotZeroException("The last blending radius must be zero");
  }
}

}

#endif // COMMAND_LIST_MANAGER_H
