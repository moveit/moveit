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

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <set>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>

namespace ompl_interface
{
// class PlanningContextManager::LastPlanningContext
// {
// public:
//   ModelBasedPlanningContextPtr getContext()
//   {
//     boost::mutex::scoped_lock slock(lock_);
//     return last_planning_context_solve_;
//   }

//   void setContext(const ModelBasedPlanningContextPtr& context)
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
//   ModelBasedPlanningContextPtr last_planning_context_solve_;
//   boost::mutex lock_;
// };

// struct PlanningContextManager::CachedContexts
// {
//   std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> > contexts_;
//   boost::mutex lock_;
// };

}  // namespace ompl_interface

ompl_interface::PlanningContextManager::PlanningContextManager(
    const robot_model::RobotModelConstPtr& kmodel, const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  : kmodel_(kmodel), constraint_sampler_manager_(csm)
{
  // last_planning_context_.reset(new LastPlanningContext());
  // cached_contexts_.reset(new CachedContexts());
  registerDefaultStateSpaces();
}

void ompl_interface::PlanningContextManager::registerDefaultStateSpaces()
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
}

void ompl_interface::PlanningContextManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  planner_configs_ = pconfig;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_interface::PlannerConfigurationSettings& config,
    const StateSpaceFactoryTypeSelector& factory_selector, const moveit_msgs::MotionPlanRequest& req) const
{
  const ompl_interface::ModelBasedStateSpaceFactoryPtr& factory = factory_selector(config.group);

  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  // {
  //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
  //   std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> >::const_iterator cc =
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
    ModelBasedStateSpaceSpecification space_spec(kmodel_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = factory->getNewStateSpace(space_spec);

    bool state_validity_cache = true;
    ROS_DEBUG_NAMED("planning_context_manager", "Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec));
    context->useStateValidityCache(state_validity_cache);

    // {
    //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    //   cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    // }
  }

  context->setSpecificationConfig(config.config);

  // last_planning_context_->setContext(context);
  return context;
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory1(
    const std::string& /* dummy */, const std::string& factory_type) const
{
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f =
      factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory2(
    const std::string& group, const moveit_msgs::MotionPlanRequest& req) const
{
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin();
       it != state_space_factories_.end(); ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, kmodel_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR_NAMED("planning_context_manager", "There are no known state spaces that can represent the given planning "
                                                "problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    ROS_DEBUG_NAMED("planning_context_manager", "Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED("planning_context_manager", "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("planning_context_manager", "No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
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
      return ModelBasedPlanningContextPtr();
    }
  }

  // Check if sampling in JointModelStateSpace is enforced for this group by user.
  // This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
  // However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
  // leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
  // in JointModelStateSpace.
  StateSpaceFactoryTypeSelector factory_selector;
  std::map<std::string, std::string>::const_iterator it = pc->second.config.find("enforce_joint_model_state_space");

  if (it != pc->second.config.end() && boost::lexical_cast<bool>(it->second))
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1,
                                   JointModelStateSpace::PARAMETERIZATION_TYPE);
  else
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory2, this, _1, req);

  ModelBasedPlanningContextPtr context = getPlanningContext(pc->second, factory_selector, req);

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
      return ModelBasedPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    try
    {
      context->configure();
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

// ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getLastPlanningContext() const
// {
//   return last_planning_context_->getContext();
// }
