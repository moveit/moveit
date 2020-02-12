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

#include "pilz_trajectory_generation/pilz_command_planner.h"

#include "pilz_trajectory_generation/planning_context_loader.h"
#include "pilz_trajectory_generation/planning_context_loader_ptp.h"
#include "pilz_trajectory_generation/planning_exceptions.h"

#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "pilz_trajectory_generation/cartesian_limits_aggregator.h"

// Boost includes
#include <boost/scoped_ptr.hpp>

#include <pluginlib/class_list_macros.h>

#include <pluginlib/class_loader.h>

namespace pilz {

static const std::string PARAM_NAMESPACE_LIMTS = "robot_description_planning";

bool CommandPlanner::initialize(const moveit::core::RobotModelConstPtr &model, const std::string &ns)
{
  // Call parent class initialize
  planning_interface::PlannerManager::initialize(model, ns);

  // Store the model and the namespace
  model_ = model;
  namespace_ = ns;

  // Obtain the aggregated joint limits
  aggregated_limit_active_joints_ = pilz::JointLimitsAggregator::getAggregatedLimits(
        ros::NodeHandle(PARAM_NAMESPACE_LIMTS),model->getActiveJointModels());

  // Obtain cartesian limits
  cartesian_limit_ = pilz::CartesianLimitsAggregator::getAggregatedLimits(ros::NodeHandle(PARAM_NAMESPACE_LIMTS));

  // Load the planning context loader
  planner_context_loader.reset(new pluginlib::ClassLoader<PlanningContextLoader>("pilz_trajectory_generation",
                                                                                    "pilz::PlanningContextLoader"));

  // List available plugins
  const std::vector<std::string> &factories = planner_context_loader->getDeclaredClasses();
  std::stringstream ss;
  for (const auto& factory : factories)
  {
    ss << factory << " ";
  }

  ROS_INFO_STREAM("Available plugins: " << ss.str());

  // Load each factory
  for (const auto& factory : factories)
  {

    ROS_INFO_STREAM("About to load: " << factory);
    PlanningContextLoaderPtr loader_pointer(planner_context_loader->createInstance(factory));

    pilz::LimitsContainer limits;
    limits.setJointLimits(aggregated_limit_active_joints_);
    limits.setCartesianLimits(cartesian_limit_);

    loader_pointer->setLimits(limits);
    loader_pointer->setModel(model_);

    registerContextLoader(loader_pointer);

  }

  return true;
}

std::string CommandPlanner::getDescription() const
{
  return "Simple Command Planner";
}

void CommandPlanner::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();

  for(const auto& context_loader : context_loader_map_)
  {
    algs.push_back(context_loader.first);
  }
}

planning_interface::PlanningContextPtr CommandPlanner::getPlanningContext(
                                                      const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                      const moveit_msgs::MotionPlanRequest& req,
                                                      moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_DEBUG_STREAM("Loading PlanningContext for request\n<request>\n" << req << "\n</request>");

  // Check that a loaded for this request exists
  if(!canServiceRequest(req))
  {
    ROS_ERROR_STREAM("No ContextLoader for planner_id " << req.planner_id.c_str() << " found. Planning not possible.");
    return nullptr;
  }

  planning_interface::PlanningContextPtr planning_context;

  if(context_loader_map_.at(req.planner_id)->loadContext(planning_context, req.planner_id, req.group_name))
  {
    ROS_DEBUG_STREAM("Found planning context loader for " << req.planner_id << " group:" << req.group_name);
    planning_context->setMotionPlanRequest(req);
    planning_context->setPlanningScene(planning_scene);
    return planning_context;
  }
  else {
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return nullptr;
  }

}

bool CommandPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  return context_loader_map_.find(req.planner_id) != context_loader_map_.end();
}

void CommandPlanner::registerContextLoader(const pilz::PlanningContextLoaderPtr& planning_context_loader)
{
  // Only add if command is not already in list, throw exception if not
  if(context_loader_map_.find(planning_context_loader->getAlgorithm()) == context_loader_map_.end())
  {
    context_loader_map_[planning_context_loader->getAlgorithm()] = planning_context_loader;
    ROS_INFO_STREAM("Registered Algorithm [" << planning_context_loader->getAlgorithm() << "]");
  }
  else
  {
    throw ContextLoaderRegistrationException("The command [" + planning_context_loader->getAlgorithm()
                                                      + "] is already registered");
  }
}

} // namespace pilz

PLUGINLIB_EXPORT_CLASS(pilz::CommandPlanner, planning_interface::PlannerManager)
