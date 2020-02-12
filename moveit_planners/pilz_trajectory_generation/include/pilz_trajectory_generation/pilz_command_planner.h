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

#ifndef PILZ_COMMAND_PLANNER_H
#define PILZ_COMMAND_PLANNER_H

#include <ros/ros.h>

#include "pilz_trajectory_generation/planning_context_loader.h"
#include "pilz_extensions/joint_limits_extension.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/macros/class_forward.h>

#include <pluginlib/class_loader.h>

// Boost includes
#include <boost/scoped_ptr.hpp>

namespace pilz {

/**
 * @brief Moveit Plugin for Planning with Standart Robot Commands
 * This planner is dedicated to return a instance of PlanningContext that corresponds to the requested motion command
 * set as planner_id in the MotionPlanRequest).
 * It can be easily extended with additional commands by creating a class inherting from PlanningContextLoader.
 */
class CommandPlanner : public planning_interface::PlannerManager
{
public:
  virtual ~CommandPlanner(){}

  /**
   * @brief Initializes the planner
   * Upon initialization this planner will look for plugins implementing pilz::PlanningContextLoader.
   * @param model The robot model
   * @param ns The namespace
   * @return true on success, false otherwise
   */
  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;

  /// Description of the planner
  virtual std::string getDescription() const override;

  /**
   * @brief Returns the available planning commands
   * @param list with the planning algorithms
   * @note behined each command is a pilz::PlanningContextLoader loaded as plugin
   */
  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  /**
   * @brief Returns a PlanningContext that can be used to solve(calculate) the trajectory that corresponds to command
   * given in motion request as planner_id.
   * @param planning_scene
   * @param req
   * @param error_code
   * @return
   */
  virtual planning_interface::PlanningContextPtr getPlanningContext(
                                                        const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                        const planning_interface::MotionPlanRequest& req,
                                                        moveit_msgs::MoveItErrorCodes& error_code) const override;

  /**
   * @brief Checks if the request can be handled
   * @param motion request containing the planning_id that corresponds to the motion command
   * @return true if the request can be handled
   */
  virtual bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

  /**
   * @brief Register a PlanningContextLoader to be used by the CommandPlanner
   * @param planning_context_loader
   * @throw ContextLoaderRegistrationException if a loader with the same algorithm name is already registered
   */
  void registerContextLoader(const pilz::PlanningContextLoaderPtr& planning_context_loader);

private:

  /// Plugin loader
  boost::scoped_ptr<pluginlib::ClassLoader<PlanningContextLoader> > planner_context_loader;

  /// Mapping from command to loader
  std::map<std::string, pilz::PlanningContextLoaderPtr> context_loader_map_;

  /// Robot model obtained at initialize
  moveit::core::RobotModelConstPtr model_;

  /// Namespace where the parameters are stored, obtained at initialize
  std::string namespace_;

  /// aggregated limits of the active joints
  pilz::JointLimitsContainer aggregated_limit_active_joints_;

  /// cartesian limit
  pilz::CartesianLimit cartesian_limit_;
};

MOVEIT_CLASS_FORWARD(CommandPlanner)

} // namespace

#endif // PILZ_COMMAND_PLANNER_H
