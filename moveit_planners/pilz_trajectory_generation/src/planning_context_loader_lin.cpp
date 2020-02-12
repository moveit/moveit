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

#include "pilz_trajectory_generation/planning_context_lin.h"
#include "pilz_trajectory_generation/planning_context_base.h"
#include "pilz_trajectory_generation/planning_context_loader_lin.h"
#include "moveit/planning_scene/planning_scene.h"

#include <pluginlib/class_list_macros.h>

pilz::PlanningContextLoaderLIN::PlanningContextLoaderLIN()
{
  alg_ = "LIN";
}

pilz::PlanningContextLoaderLIN::~PlanningContextLoaderLIN()
{

}

bool pilz::PlanningContextLoaderLIN::loadContext(planning_interface::PlanningContextPtr& planning_context,
                                                 const std::string& name,
                                                 const std::string& group) const
{
  if(limits_set_ && model_set_) {
    planning_context.reset(new PlanningContextLIN(name, group, model_, limits_));
    return true;
  }
  else
  {
    if(!limits_set_)
    {
      ROS_ERROR_STREAM("Limits are not defined. Cannot load planning context. Call setLimits loadContext");
    }
    if(!model_set_)
    {
      ROS_ERROR_STREAM("Robot model was not set");
    }
    return false;
  }
}

PLUGINLIB_EXPORT_CLASS(pilz::PlanningContextLoaderLIN, pilz::PlanningContextLoader)
