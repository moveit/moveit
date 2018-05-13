/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <pluginlib/class_loader.h>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <set>

namespace ompl_interface
{
// class PlanningContextManager::LastPlanningContext
// {
// public:
//   OMPLPlanningContextPtr getContext()
//   {
//     boost::mutex::scoped_lock slock(lock_);
//     return last_planning_context_solve_;
//   }

//   void setContext(const OMPLPlanningContextPtr& context)
//   {
//     boost::mutex::scoped_lock slock(lock_);
//     last_planning_context_solve_ = context;
//   }

//   void clear()
//   {
//     boost::mutex::scoped_lock slock(lock_);
//     last_planning_context_solve_.reset();
//   }

// private:
//   /* The planning group for which solve() was called last */
//   OMPLPlanningContextPtr last_planning_context_solve_;
//   boost::mutex lock_;
// };

// struct PlanningContextManager::CachedContexts
// {
//   std::map<std::pair<std::string, std::string>, std::vector<OMPLPlanningContextPtr> > contexts_;
//   boost::mutex lock_;
// };

}  // namespace ompl_interface

ompl_interface::PlanningContextManager::PlanningContextManager(
    const robot_model::RobotModelConstPtr& kmodel, const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  : kmodel_(kmodel), constraint_sampler_manager_(csm)
{
}

bool ompl_interface::PlanningContextManager::initialize()
{
  // last_planning_context_.reset(new LastPlanningContext());
  // cached_contexts_.reset(new CachedContexts());

  // Initialize planning context plugin loader
  try
  {
    ompl_context_loader_.reset(
        new pluginlib::ClassLoader<OMPLPlanningContext>("moveit_planners_ompl", "ompl_interface::OMPLPlanningContext"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning context plugin loader " << ex.what());
    return false;
  }

  return true;
}

void ompl_interface::PlanningContextManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  planner_configs_ = pconfig;
}

ompl_interface::OMPLPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED("planning_context_manager", "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return OMPLPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("planning_context_manager", "No planning scene supplied as input");
    return OMPLPlanningContextPtr();
  }

  // identify the correct planning configuration
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN_NAMED("planning_context_manager",
                     "Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
                     req.group_name.c_str(), req.planner_id.c_str());
  }

  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_NAMED("planning_context_manager", "Cannot find planning configuration for group '%s'",
                      req.group_name.c_str());
      return OMPLPlanningContextPtr();
    }
  }

  // Check for a cached planning context
  OMPLPlanningContextPtr context;
  const planning_interface::PlannerConfigurationSettings& config = pc->second;

  // {
  //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
  //   std::map<std::pair<std::string, std::string>, std::vector<OMPLPlanningContextPtr> >::const_iterator cc =
  //       cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));
  //   if (cc != cached_contexts_->contexts_.end())
  //   {
  //     for (std::size_t i = 0; i < cc->second.size(); ++i)
  //       if (cc->second[i].unique())
  //       {
  //         ROS_DEBUG_NAMED("planning_context_manager", "Reusing cached planning context");
  //         context = cc->second[i];
  //         break;
  //       }
  //   }
  // }

  // Create a new planning context
  if (!context)
  {
    OMPLPlanningContextSpecification context_spec;
    context_spec.config_ = config;
    context_spec.csm_ = constraint_sampler_manager_;
    context_spec.req_ = req;
    context_spec.robot_model_ = kmodel_;
    context_spec.jmg_ = kmodel_->getJointModelGroup(config.group);

    ROS_DEBUG_NAMED("planning_context_manager", "Creating new planning context");

    auto config_it = config.config.find("plugin");

    boost::shared_ptr<OMPLPlanningContext> ptr;
    if (config_it != config.config.end())
    {
      std::string plugin = config_it->second;
      if (!ompl_context_loader_->isClassAvailable(plugin))
      {
        ROS_ERROR("Plugin '%s' does not exist. Assuming default plugin.", plugin.c_str());
        plugin = DEFAULT_OMPL_PLUGIN;
      }

      ptr = ompl_context_loader_->createInstance(plugin);
    }
    else
    {
      ROS_WARN("No plugin specified for planner configuration '%s'.  Assuming default plugin.", config.name.c_str());
      ptr = ompl_context_loader_->createInstance(DEFAULT_OMPL_PLUGIN);
    }

    context = std::shared_ptr<OMPLPlanningContext>(ptr.get(), [ptr](OMPLPlanningContext*) mutable { ptr.reset(); });

    context->initialize(context_spec);
    // {
    //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    //   cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    // }
  }

  // last_planning_context_->setContext(context);

  if (context)
  {
    context->clear();

    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return OMPLPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return OMPLPlanningContextPtr();

    try
    {
      ROS_DEBUG_NAMED("planning_context_manager", "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED("planning_context_manager", "OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}

// ompl_interface::OMPLPlanningContextPtr ompl_interface::PlanningContextManager::getLastPlanningContext() const
// {
//   return last_planning_context_->getContext();
// }
