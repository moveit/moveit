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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan  */

#include "ompl_interface/planning_context_manager.h"

#include <planning_models/conversions.h>
#include <algorithm>
#include <set>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/msRRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/tools/debug/Profiler.h>

#include "ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h"
#include "ompl_interface/parameterization/work_space/pose_model_state_space_factory.h"

namespace ompl_interface
{
class PlanningContextManager::LastPlanningContext
{
public:
  
  ModelBasedPlanningContextPtr getContext(void)
  {
    boost::mutex::scoped_lock slock(lock_);
    return last_planning_context_solve_;
  }
  
  void setContext(const ModelBasedPlanningContextPtr &context)
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_ = context;
  }

  void clear(void)
  {    
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_.reset();
  }
  
private:
  /* The planning group for which solve() was called last */
  ModelBasedPlanningContextPtr last_planning_context_solve_;
  boost::mutex                 lock_;  
};

struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>,
           std::vector<ModelBasedPlanningContextPtr> > contexts_;
  boost::mutex                                         lock_;
};

}

ompl_interface::PlanningContextManager::PlanningContextManager(const planning_models::KinematicModelConstPtr &kmodel) :
  kmodel_(kmodel), constraint_sampler_manager_(NULL), max_goal_samples_(10), max_state_sampling_attempts_(10), max_goal_sampling_attempts_(1000),
  max_planning_threads_(4), max_velocity_(10), max_acceleration_(2.0), max_solution_segment_length_(0.0)
{
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultStateSpaces();
}

ompl_interface::PlanningContextManager::~PlanningContextManager(void)
{
}

namespace ompl_interface
{

template<typename T>
static ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si)
{
  return ob::PlannerPtr(new T(si));
}
}

ompl::base::PlannerPtr ompl_interface::PlanningContextManager::plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                                                const std::string &name, const std::map<std::string, std::string> &config) const
{
  std::map<std::string, ob::PlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
  {
    ob::PlannerPtr p = it->second(si);
    if (!name.empty())
      p->setName(name);
    p->params().setParams(config, true);
    p->setup();
    return p;
  }
  else
  {
    ROS_ERROR("Unknown planner: '%s'", planner.c_str());
    return ob::PlannerPtr();
  }
}
  
ompl_interface::ConfiguredPlannerAllocator ompl_interface::PlanningContextManager::getPlannerAllocator(void) const
{
  return boost::bind(&PlanningContextManager::plannerAllocator, this, _1, _2, _3, _4);
}

void ompl_interface::PlanningContextManager::registerDefaultPlanners(void)
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1));
  //  registerPlannerAllocator("geometric::msRRTConnect", boost::bind(&allocatePlanner<og::msRRTConnect>, _1));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1));
}

void ompl_interface::PlanningContextManager::registerDefaultStateSpaces(void)
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
}

void ompl_interface::PlanningContextManager::setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig)
{
  planner_configs_.clear();
  for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
    planner_configs_[pconfig[i].name] = pconfig[i];

  // construct default configurations
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& groups = kmodel_->getJointModelGroupMap();
  for (std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
    if (planner_configs_.find(it->first) == planner_configs_.end())
    {
      PlanningConfigurationSettings empty;
      empty.name = empty.group = it->first;
      planner_configs_[empty.name] = empty;
    }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(const std::string &config, const std::string& factory_type) const
{
  std::map<std::string, PlanningConfigurationSettings>::const_iterator pc = planner_configs_.find(config);
  if (pc != planner_configs_.end())
  {
    std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f = 
      factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
    if (f != state_space_factories_.end())
      return getPlanningContext(pc->second, f->second.get());
    else
      ROS_ERROR("Factory of type '%s' was not found", factory_type.c_str());    
  }
  else
    ROS_ERROR("Planning configuration '%s' was not found", config.c_str());
  return ModelBasedPlanningContextPtr();
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(const PlanningConfigurationSettings &config,
                                                                                                        const ModelBasedStateSpaceFactory *factory) const
{
  ModelBasedPlanningContextPtr context;
  {
    boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> >::const_iterator cc =
      cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));
    if (cc != cached_contexts_->contexts_.end())
    {
      for (std::size_t i = 0 ; i < cc->second.size() ; ++i)
        if (cc->second[i].unique())
        {
          ROS_DEBUG("Reusing cached planning context");
          context = cc->second[i];
          break;
        }
    }
  }
  
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(kmodel_, config.group);
    planning_scene::KinematicsAllocators::const_iterator it = kinematics_allocators_.find(space_spec.joint_model_group_);
    if (it != kinematics_allocators_.end())
    {
      space_spec.kinematics_allocator_ = it->second.first;
      space_spec.kinematics_subgroup_allocators_ = it->second.second;
    }
    
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_allocator_ = getPlannerAllocator();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    ROS_DEBUG("Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, factory->getNewStateSpace(space_spec), context_spec));
    {
      boost::mutex::scoped_lock slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  context->setMaximumVelocity(max_velocity_);
  context->setMaximumAcceleration(max_acceleration_);
  if (max_solution_segment_length_ <= std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(context->getOMPLSimpleSetup().getStateSpace()->getMaximumExtent() / 100.0);
  else
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  
  last_planning_context_->setContext(context);
  return context;  
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(const moveit_msgs::MotionPlanRequest &req) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
    return ModelBasedPlanningContextPtr();
  }
  
  // find the problem representation to use
  const ModelBasedStateSpaceFactory *factory = NULL;
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin() ; it != state_space_factories_.end() ; ++it)
  {
    int priority = it->second->canRepresentProblem(req, kmodel_, kinematics_allocators_);
    if (priority >= 0)
      if (!factory || priority > prev_priority)
      {
        factory = it->second.get();
        prev_priority = priority;
      }
  }
  
  if (!factory)
  {
    ROS_ERROR("There are no known state spaces that can represent the given planning problem");
    return ModelBasedPlanningContextPtr();
  }
  else
    ROS_DEBUG("Using '%s' parameterization for solving problem", factory->getType().c_str());
  
  // identify the correct planning configuration
  std::map<std::string, PlanningConfigurationSettings>::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.group_name + "[" + req.planner_id + "]");
    if (pc == planner_configs_.end())
      ROS_WARN_STREAM("Cannot find planning configuration for group '" << req.group_name
		      << "' using planner '" << req.planner_id << "'. Will use defaults instead.");
  }
  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_STREAM("Cannot find planning configuration for group '" << req.group_name << "'");
      return ModelBasedPlanningContextPtr();
    }
  }
  
  return getPlanningContext(pc->second, factory);
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getLastPlanningContext(void) const
{
  return last_planning_context_->getContext();
}
