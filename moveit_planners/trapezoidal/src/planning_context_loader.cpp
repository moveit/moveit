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
#include <ros/ros.h>
#include "trapezoidal_trajectory_generation/planning_context_loader.h"

trapezoidal::PlanningContextLoader::PlanningContextLoader():
  limits_set_(false),
  model_set_(false)
{

}

trapezoidal::PlanningContextLoader::~PlanningContextLoader(){}

bool trapezoidal::PlanningContextLoader::setModel(const moveit::core::RobotModelConstPtr &model)
{
  model_ = model;
  model_set_ = true;
  return true;
}

bool trapezoidal::PlanningContextLoader::setLimits(const trapezoidal::LimitsContainer &limits)
{
  limits_ = limits;
  limits_set_ = true;
  return true;
}

std::string trapezoidal::PlanningContextLoader::getAlgorithm() const
{
  return alg_;
}
