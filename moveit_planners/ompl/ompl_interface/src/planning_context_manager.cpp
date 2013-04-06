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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <algorithm>
#include <set>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
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

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>

namespace ompl_interface
{
class PlanningContextManager::LastPlanningContext
{
public:
  
  ModelBasedPlanningContextPtr getContext()
  {
    boost::mutex::scoped_lock slock(lock_);
    return last_planning_context_solve_;
  }
  
  void setContext(const ModelBasedPlanningContextPtr &context)
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_ = context;
  }

  void clear()
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

ompl_interface::PlanningContextManager::PlanningContextManager(const robot_model::RobotModelConstPtr &kmodel, const constraint_samplers::ConstraintSamplerManagerPtr &csm) :
  kmodel_(kmodel), constraint_sampler_manager_(csm),
  max_goal_samples_(10), max_state_sampling_attempts_(4), max_goal_sampling_attempts_(1000),
  max_planning_threads_(4), max_solution_segment_length_(0.0)
{
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultStateSpaces();
}

ompl_interface::PlanningContextManager::~PlanningContextManager()
{
}

namespace ompl_interface
{

template<typename T>
inline void auxPlannerConfig(const ob::PlannerPtr &planner, const ModelBasedPlanningContextSpecification &spec)
{
}
/*
template<>
inline void auxPlannerConfig<og::msRRTConnect>(const ob::PlannerPtr &planner, const ModelBasedPlanningContextSpecification &spec)
{
  for (std::size_t i = 0 ; i < spec.subspaces_.size() ; ++i)
    planner->as<og::msRRTConnect>()->addExplorationSubspace(spec.subspaces_[i]);
}
*/
template<typename T>
static ompl::base::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si, const std::string &new_name, const ModelBasedPlanningContextSpecification &spec)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);  
  auxPlannerConfig<T>(planner, spec);
  planner->setup();
  return planner;
}

}

ompl_interface::ConfiguredPlannerAllocator ompl_interface::PlanningContextManager::plannerSelector(const std::string &planner) const
{
  std::map<std::string, ConfiguredPlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    logError("Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}
  

void ompl_interface::PlanningContextManager::registerDefaultPlanners()
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
  //  registerPlannerAllocator("geometric::msRRTConnect", boost::bind(&allocatePlanner<og::msRRTConnect>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3)); 
  registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
}

void ompl_interface::PlanningContextManager::registerDefaultStateSpaces()
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
}

ompl_interface::ConfiguredPlannerSelector ompl_interface::PlanningContextManager::getPlannerSelector() const
{
  return boost::bind(&PlanningContextManager::plannerSelector, this, _1);
}

void ompl_interface::PlanningContextManager::setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig)
{
  planner_configs_.clear();
  for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
    planner_configs_[pconfig[i].name] = pconfig[i];

  // construct default configurations
  const std::map<std::string, robot_model::JointModelGroup*>& groups = kmodel_->getJointModelGroupMap();
  for (std::map<std::string, robot_model::JointModelGroup*>::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
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
    return getPlanningContext(pc->second, boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1, factory_type));
  else
  {
    logError("Planning configuration '%s' was not found", config.c_str());
    return ModelBasedPlanningContextPtr();
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(const PlanningConfigurationSettings &config,
                                                                                                        const FactoryTypeSelector &factory_selector) const
{
  const ompl_interface::ModelBasedStateSpaceFactoryPtr &factory = factory_selector(config.group);
  
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
          logDebug("Reusing cached planning context");
          context = cc->second[i];
          break;
        }
    }
  }
  
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(kmodel_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_selector_ = getPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = factory->getNewStateSpace(space_spec);
    context_spec.use_state_validity_cache_ = true;
    if (config.config.find("subspaces") != config.config.end())
    {
      context_spec.config_.erase("subspaces");
      // if the planner operates at subspace level the cache may be unsafe
      context_spec.use_state_validity_cache_ = false;
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(config.config.at("subspaces"), sep);
      for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
      {
        const ompl_interface::ModelBasedStateSpaceFactoryPtr &sub_fact = factory_selector(*beg);
        if (sub_fact)
        {
          ModelBasedStateSpaceSpecification sub_space_spec(kmodel_, *beg);
          context_spec.subspaces_.push_back(sub_fact->getNewStateSpace(sub_space_spec));
        }
      }
    }
    
    logDebug("Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec));
    {
      boost::mutex::scoped_lock slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ <= std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(context->getOMPLSimpleSetup().getStateSpace()->getMaximumExtent() / 100.0);
  else
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  
  last_planning_context_->setContext(context);
  return context;  
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory1(const std::string & /* dummy */, const std::string &factory_type) const
{
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f = 
    factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    logError("Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory2(const std::string &group, const moveit_msgs::MotionPlanRequest &req) const
{
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin() ; it != state_space_factories_.end() ; ++it)
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
    logError("There are no known state spaces that can represent the given planning problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    logDebug("Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(const moveit_msgs::MotionPlanRequest &req) const
{
  if (req.group_name.empty())
  {
    logError("No group specified to plan for");
    return ModelBasedPlanningContextPtr();
  }
  
  // identify the correct planning configuration
  std::map<std::string, PlanningConfigurationSettings>::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ? req.group_name + "[" + req.planner_id + "]" : req.planner_id);
    if (pc == planner_configs_.end())
      logWarn("Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.", 
              req.group_name.c_str(), req.planner_id.c_str());
  }
  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      logError("Cannot find planning configuration for group '%s'", req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }
  
  return getPlanningContext(pc->second, boost::bind(&PlanningContextManager::getStateSpaceFactory2, this, _1, req));
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getLastPlanningContext() const
{
  return last_planning_context_->getContext();
}
