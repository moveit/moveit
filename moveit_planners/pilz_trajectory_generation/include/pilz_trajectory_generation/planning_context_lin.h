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

#ifndef PLANNINGCONTEXTLIN_H
#define PLANNINGCONTEXTLIN_H

#include "pilz_trajectory_generation/limits_container.h"

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>

#include <atomic>
#include <thread>

#include "pilz_trajectory_generation/planning_context_base.h"
#include "pilz_trajectory_generation/trajectory_generator_lin.h"


namespace pilz {

MOVEIT_CLASS_FORWARD(PlanningContext)

/**
 * @brief PlanningContext for obtaining LIN trajectories
 */
class PlanningContextLIN : public pilz::PlanningContextBase<TrajectoryGeneratorLIN>
{
  public:
    PlanningContextLIN(const std::string& name,
                       const std::string& group,
                       const moveit::core::RobotModelConstPtr& model,
                       const pilz::LimitsContainer& limits):
    pilz::PlanningContextBase<TrajectoryGeneratorLIN>(name, group, model, limits){}
};

} // namespace

#endif // PlanningContextLIN_H
