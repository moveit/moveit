/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface/ompl_interface.h"

#include <planning_models/conversions.h>
#include <algorithm>
#include <fstream>
#include <set>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

#include "ompl_interface/parameterization/joint_space/joint_model_planning_context_factory.h"
#include "ompl_interface/parameterization/work_space/pose_model_planning_context_factory.h"

namespace ompl_interface
{
class OMPLInterface::LastPlanningContext
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

struct OMPLInterface::CachedContexts
{
  std::map<std::pair<std::string, std::string>,
           std::vector<ModelBasedPlanningContextPtr> > contexts_;
  boost::mutex                                         lock_;
};

}

ompl_interface::OMPLInterface::OMPLInterface(const planning_models::KinematicModelConstPtr &kmodel) :
  kmodel_(kmodel), max_goal_samples_(10), max_sampling_attempts_(10), max_planning_threads_(4),
  max_velocity_(10), max_acceleration_(2.0), max_solution_segment_length_(0.0)
{
  constraints_.reset(new std::vector<ConstraintApproximation>());
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultPlanningContexts();
}

ompl_interface::OMPLInterface::~OMPLInterface(void)
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

ompl::base::PlannerPtr ompl_interface::OMPLInterface::plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
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

void ompl_interface::OMPLInterface::specifyIKSolvers(const std::map<std::string, kc::KinematicsAllocator> &kinematics_allocators)
{
  kinematics_allocators_.clear();
  const std::map<std::string, pm::KinematicModel::JointModelGroup*>& groups = kmodel_->getJointModelGroupMap();
  for (std::map<std::string, pm::KinematicModel::JointModelGroup*>::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
  {
    const pm::KinematicModel::JointModelGroup *jmg = it->second;
    std::pair<kc::KinematicsAllocator, kc::KinematicsSubgroupAllocator> result;
    
    std::map<std::string, kc::KinematicsAllocator>::const_iterator jt = kinematics_allocators.find(jmg->getName());
    if (jt == kinematics_allocators.end())
    {
      // if an IK allocator is NOT available for this group, we try to see if we can use subgroups for IK
      std::set<const pm::KinematicModel::JointModel*> joints;
      joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());
      
      std::vector<const pm::KinematicModel::JointModelGroup*> subs;
      
      // go through the groups that we know have IK allocators and see if they are included in the group that does not; fi so, put that group in sub
      for (std::map<std::string, kc::KinematicsAllocator>::const_iterator kt = kinematics_allocators.begin() ;
           kt != kinematics_allocators.end() ; ++kt)
      {
        const pm::KinematicModel::JointModelGroup *sub = jmg->getParentModel()->getJointModelGroup(kt->first);
        std::set<const pm::KinematicModel::JointModel*> sub_joints;
        sub_joints.insert(sub->getJointModels().begin(), sub->getJointModels().end());
        
        if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
        {
          std::set<const pm::KinematicModel::JointModel*> result;
          std::set_difference(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end(),
                              std::inserter(result, result.end()));
          subs.push_back(sub);
          joints = result;
        }
      }
      
      // if we found subgroups, pass that information to the planning group
      if (!subs.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0 ; i < subs.size() ; ++i)
        {
          ss << subs[i]->getName() << " ";
          result.second[subs[i]] = kinematics_allocators.find(subs[i]->getName())->second;
        }
        ROS_DEBUG("Added sub-group IK allocators for group '%s': [ %s]", jmg->getName().c_str(), ss.str().c_str());
      }
    }
    else
      // if the IK allocator is for this group, we use it
      result.first = jt->second;

    kinematics_allocators_[jmg] = result;
  }  
}
  
ompl_interface::ConfiguredPlannerAllocator ompl_interface::OMPLInterface::getPlannerAllocator(void) const
{
  return boost::bind(&OMPLInterface::plannerAllocator, this, _1, _2, _3, _4);
}

void ompl_interface::OMPLInterface::registerDefaultPlanners(void)
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1));
}

void ompl_interface::OMPLInterface::registerDefaultPlanningContexts(void)
{
  registerPlanningContextFactory(ModelBasedPlanningContextFactoryPtr(new JointModelPlanningContextFactory()));
  registerPlanningContextFactory(ModelBasedPlanningContextFactoryPtr(new PoseModelPlanningContextFactory()));
}

void ompl_interface::OMPLInterface::setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig)
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

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(const std::string &config, const std::string& factory_type) const
{
  std::map<std::string, PlanningConfigurationSettings>::const_iterator pc = planner_configs_.find(config);
  if (pc != planner_configs_.end())
  {
    std::map<std::string, ModelBasedPlanningContextFactoryPtr>::const_iterator f = 
      factory_type.empty() ? planning_context_factories_.begin() : planning_context_factories_.find(factory_type);
    if (f != planning_context_factories_.end())
      return getPlanningContext(f->second.get(), pc->second);
    else
      ROS_ERROR("Factory of type '%s' was not found", factory_type.c_str());    
  }
  else
    ROS_ERROR("Planning configuration '%s' was not found", config.c_str());
  return ModelBasedPlanningContextPtr();
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(const ModelBasedPlanningContextFactory *factory, 
                                                                                               const PlanningConfigurationSettings &config) const
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
    AvailableKinematicsSolvers::const_iterator it = kinematics_allocators_.find(space_spec.joint_model_group_);
    if (it != kinematics_allocators_.end())
    {
      space_spec.kinematics_allocator_ = it->second.first;
      space_spec.kinematics_subgroup_allocators_ = it->second.second;
    }
    
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_allocator_ = getPlannerAllocator();
    context_spec.constraints_ = constraints_;
    ROS_DEBUG("Creating new planning context");
    context = factory->getNewPlanningContext(config.name, space_spec, context_spec); 
    {
      boost::mutex::scoped_lock slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }
  
  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumSamplingAttempts(max_sampling_attempts_);
  context->setMaximumVelocity(max_velocity_);
  context->setMaximumAcceleration(max_acceleration_);
  if (max_solution_segment_length_ <= std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(context->getOMPLSimpleSetup().getStateSpace()->getMaximumExtent() / 100.0);
  else
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  
  return context;  
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::prepareForSolve(const moveit_msgs::MotionPlanRequest &req, 
                                                                                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                                            moveit_msgs::MoveItErrorCodes &error_code,
                                                                                            unsigned int &attempts, double &timeout) const
{
  if (req.group_name.empty())
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    ROS_ERROR("No group specified to plan for");
    return ModelBasedPlanningContextPtr();
  }
  
  // find the problem representation to use
  const ModelBasedPlanningContextFactory *factory = NULL;
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedPlanningContextFactoryPtr>::const_iterator it = planning_context_factories_.begin() ; it != planning_context_factories_.end() ; ++it)
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
    ROS_ERROR("There are no known planning contexts that can represent the given planning problem");
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
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      ROS_ERROR_STREAM("Cannot find planning configuration for group '" << req.group_name << "'");
      return ModelBasedPlanningContextPtr();
    }
  }
  
  ModelBasedPlanningContextPtr context = getPlanningContext(factory, pc->second);
  
  timeout = req.allowed_planning_time.toSec();
  if (timeout <= 0.0)
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ALLOWED_PLANNING_TIME;
    ROS_INFO("The timeout for planning must be positive (%lf specified). Assuming one second instead.", timeout);
    timeout = 1.0;
  }
  
  attempts = 1;
  if (req.num_planning_attempts > 0)
    attempts = req.num_planning_attempts;
  else
    if (req.num_planning_attempts < 0)
      ROS_ERROR("The number of desired planning attempts should be positive. Assuming one attempt.");
  
  return context;
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const
{
  ompl::tools::Profiler::ScopedStart pslv;
  ot::Profiler::ScopedBlock sblock("OMPLInterfaceSolve");
  last_planning_context_->clear();
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, res.error_code, attempts, timeout);
  if (!context)
    return false;

  last_planning_context_->setContext(context);
  
  // get the starting state
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
  context->setPlanningScene(planning_scene);
  context->setStartState(start_state);
  context->setPlanningVolume(req.motion_plan_request.workspace_parameters);
  if (context->setPlanningConstraints(req.motion_plan_request.goal_constraints, req.motion_plan_request.path_constraints, &res.error_code))
  {
    ROS_DEBUG("%s: New planning context is set.", context->getName().c_str());
    context->configure();  
    
    if (context->solve(timeout, attempts))
    {
      double ptime = context->getLastPlanTime();
      if (ptime < timeout)
	context->simplifySolution(timeout - ptime);
      context->interpolateSolution();
      
      // fill the response
      ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
                context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
      pm::kinematicStateToRobotState(start_state, res.trajectory_start);
      res.planning_time = ros::Duration(context->getLastPlanTime());
      context->getSolutionPath(res.trajectory);
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
    else
    {
      ROS_INFO("Unable to solve the planning problem");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Unable to set planning context");
    return false;
  }
}

bool ompl_interface::OMPLInterface::benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const moveit_msgs::ComputePlanningBenchmark::Request &req,
                                              moveit_msgs::ComputePlanningBenchmark::Response &res) const
{  
  unsigned int attempts = 1;
  double timeout = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, res.error_code, attempts, timeout);
  if (!context)
    return false;
  
  // get the starting state
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);

  context->setPlanningScene(planning_scene);
  context->setStartState(start_state);
  context->setPlanningConstraints(req.motion_plan_request.goal_constraints, req.motion_plan_request.path_constraints, &res.error_code);
  context->setPlanningVolume(req.motion_plan_request.workspace_parameters);
  context->configure();
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return context->benchmark(timeout, std::max(1u, req.average_count), req.filename);
}

ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints, double timeout,
                                                         const std::string &factory_type) const
{
  moveit_msgs::Constraints empty;
  return solve(planning_scene, config, start_state, goal_constraints, empty, timeout, factory_type);
}

ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints,
                                                         const moveit_msgs::Constraints &path_constraints, double timeout,
                                                         const std::string &factory_type) const
{ 
  ompl::tools::Profiler::ScopedStart pslv;
  last_planning_context_->clear();
  
  ModelBasedPlanningContextPtr context = getPlanningContext(config, factory_type);
  if (!context)
    return ob::PathPtr();
  
  std::vector<moveit_msgs::Constraints> goal_constraints_v(1, goal_constraints);  
  context->setPlanningScene(planning_scene);
  context->setStartState(start_state);
  context->setPlanningConstraints(goal_constraints_v, path_constraints, NULL);
  context->configure();
  
  last_planning_context_->setContext(context);
  
  // solve the planning problem
  if (context->solve(timeout, 1))
  {
    double ptime = context->getLastPlanTime();
    if (ptime < timeout)
      context->simplifySolution(timeout - ptime);
    context->interpolateSolution();
    return ob::PathPtr(new og::PathGeometric(context->getOMPLSimpleSetup().getSolutionPath()));
  }
  
  return ob::PathPtr();  
}

void ompl_interface::OMPLInterface::terminateSolve(void)
{
  const ModelBasedPlanningContextPtr &context = last_planning_context_->getContext();
  if (context)
    context->terminateSolve();
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getLastPlanningContext(void) const
{
  return last_planning_context_->getContext();
}

void ompl_interface::OMPLInterface::addConstraintApproximation(const moveit_msgs::Constraints &constr, const std::string &group, const std::string &factory, unsigned int samples)
{
  addConstraintApproximation(constr, constr, group, factory, samples);
}

void ompl_interface::OMPLInterface::addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard, const std::string &group, const std::string &factory, unsigned int samples)
{  
  const ModelBasedPlanningContextPtr &pc = getPlanningContext(group, factory);
  if (pc)
  {
    ompl::base::StateStoragePtr ss = pc->constructConstraintApproximation(constr_sampling, constr_hard, samples);
    if (ss)
      constraints_->push_back(ConstraintApproximation(kmodel_, group, factory, constr_hard, group + "_" + 
						      boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss));
    else
      ROS_ERROR("Unable to construct constraint approximation for group '%s'", group.c_str());
  }
}

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string &path)
{
  ROS_INFO("Loading constrained space approximations from '%s'", path.c_str());
  
  std::ifstream fin((path + "/manifest").c_str());
  if (!fin.good())
    ROS_DEBUG("Manifest not found in folder '%s'", path.c_str());
  
  while (fin.good() && !fin.eof())
  {
    std::string group, factory, serialization, filename;
    fin >> group;
    if (fin.eof())
      break;
    fin >> factory;
    if (fin.eof())
      break;
    fin >> serialization;    
    if (fin.eof())
      break;
    fin >> filename;
    const ModelBasedPlanningContextPtr &pc = getPlanningContext(group, factory);
    if (pc)
    {
      ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pc->getOMPLSimpleSetup().getStateSpace());
      cass->load((path + "/" + filename).c_str());
      std::size_t sum = 0;
      for (std::size_t i = 0 ; i < cass->size() ; ++i)
      {
        cass->getState(i)->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
        sum += cass->getMetadata(i).size();
      }
      constraints_->push_back(ConstraintApproximation(kmodel_, group, factory, serialization, filename, ompl::base::StateStoragePtr(cass)));
      ROS_INFO("Loaded %lu states and %lu connections (%0.1lf per state) from %s", cass->size(), sum, (double)sum / (double)cass->size(), filename.c_str());
    }
  }
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string &path)
{
  ROS_INFO("Saving %u constrained space approximations to '%s'", (unsigned int)constraints_->size(), path.c_str());
  
  std::ofstream fout((path + "/manifest").c_str());
  for (std::size_t i = 0 ; i < constraints_->size() ; ++i)
  {
    fout << constraints_->at(i).group_ << std::endl;
    fout << constraints_->at(i).factory_ << std::endl;
    fout << constraints_->at(i).serialization_ << std::endl;
    fout << constraints_->at(i).ompldb_filename_ << std::endl;
    if (constraints_->at(i).state_storage_)
      constraints_->at(i).state_storage_->store((path + "/" + constraints_->at(i).ompldb_filename_).c_str());
  }
  fout.close();
}

void ompl_interface::OMPLInterface::clearConstraintApproximations(void)
{
  constraints_->clear();
}

void ompl_interface::OMPLInterface::printConstraintApproximations(std::ostream &out) const
{
  for (std::size_t i = 0 ; i < constraints_->size() ; ++i)
  {
    out << constraints_->at(i).group_ << std::endl;
    out << constraints_->at(i).factory_ << std::endl;
    out << constraints_->at(i).ompldb_filename_ << std::endl;
    out << constraints_->at(i).constraint_msg_ << std::endl;
  }
}

