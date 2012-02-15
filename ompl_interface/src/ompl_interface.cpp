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

ompl_interface::OMPLInterface::OMPLInterface(const planning_models::KinematicModelConstPtr &kmodel) : kmodel_(kmodel)
{   
  //  constraints_.reset(new std::vector<ConstraintApproximation>());
}

ompl_interface::OMPLInterface::~OMPLInterface(void)
{
}

namespace ompl_interface
{
template<typename T>
ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si)
{
  return ob::PlannerPtr(new T(si));
}
}

ompl::base::PlannerPtr ompl_interface::OMPLInterface::plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                                       const std::map<std::string, std::string> &config) const
{
  std::map<std::string, ob::PlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
  {
    ob::PlannerPtr p = it->second(si);
    p->params().setParams(config, true);
    return p;
  }
  else
  {
    ROS_ERROR("Unknown planner: '%s'", planner.c_str());
    return ob::PlannerPtr();
  }
}

ompl_interface::ConfiguredPlannerAllocator ompl_interface::OMPLInterface::getPlannerAllocator(void) const
{
  return boost::bind(&OMPLInterface::plannerAllocator, this, _1, _2, _3);
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

void ompl_interface::OMPLInterface::setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig)
{
  planner_configs_.clear();
  for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
    planner_configs_[pconfig[i].name] = pconfig[i];
}


/*
bool ompl_interface::OMPLInterface::configure(const std::vector<PlanningConfigurationSettings> &pconfig)
{
  // construct specified configurations
  for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
  {
    const planning_models::KinematicModel::JointModelGroup *jmg = kmodel_->getJointModelGroup(pconfig[i].group);
    if (jmg)
    {
      planning_configurations_[pconfig[i].name].reset(new PlanningConfiguration(pconfig[i].name, kmodel_, jmg, constraints_, pconfig[i].config));
      ROS_INFO_STREAM("Added planning configuration '" << pconfig[i].name << "'");
    }
  }
  // construct default configurations
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& groups = kmodel_->getJointModelGroupMap();
  for (std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
    if (planning_configurations_.find(it->first) == planning_configurations_.end())
    {
      static const std::map<std::string, std::string> empty;
      planning_configurations_[it->first].reset(new PlanningConfiguration(it->first, kmodel_, it->second, constraints_, empty));
      ROS_INFO_STREAM("Added planning configuration '" << it->first << "'");
    }
  configured_ = true;
  return true;
}

ompl_interface::PlanningConfigurationPtr ompl_interface::OMPLInterface::allocPlanningConfiguration(const moveit_msgs::MotionPlanRequest &req,
                                                                                                   moveit_msgs::MoveItErrorCodes &error_code,)
{
  if (req.group_name.empty())
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    ROS_ERROR("No group specified to plan for");
    return false;
  }

  
  // decide what type of state spaces we can create;
  std::vector<KinematicModelStateSpaceFactory*> possible;
  for (std::size_t i = 0 ; i < state_space_factories_.size() ; ++i)
    if (state_space_factories_[i]->canRepresentProblem(req))
      possible.push_back(state_space_factories_[i].get());
  if (possible.empty())
  {
    ROS_ERROR("Unable to represent planning problem in any known state space");
    return false;
  }
  KinematicModelStateSpaceFactory *factory = possible[0];
  for (std::size_t i = 1 ; i < possible.size() ; ++i)
    if (factory->getPriority() < possible[i]->getPriority())
      factory = possible[i];

  // construct the state space specs
  KinematicModelStateSpaceSpecification kmsss(kmodel_, kmodel_->getJointModelGroup(req.group_name));
  kmsss.joint_limits_ = req.joint_limits;
  kmsss.constraint_approximations_ = constraint_approximations_;
  kmsss.ik_allocators_ = ik_allocators_;

  // construct the state space
  KinematicModelStateSpacePtr state_space = factory->allocStateSpace(kmsss);
  
  // construct the planning configuration
  PlanningConfigurationPtr pconfig(new PlanningConfiguration(planning_scene, state_space));
  pconfig->solve();
  
  
}
*/

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
  int index = -1;
  for (std::size_t i = 0 ; i < planning_context_factories_.size() ; ++i)
    if (index < 0 || planning_context_factories_[i]->getPriority() > planning_context_factories_[index]->getPriority())
      if (planning_context_factories_[i]->canRepresentProblem(req))
        index = i;
  
  if (index < 0)
  {
    ROS_ERROR("There are no known planning contexts that can represent the given planning problem");
    return ModelBasedPlanningContextPtr();
  }
  
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
  
  const PlanningConfigurationSettings &pc_to_use = pc->second;
  
  KinematicModelStateSpaceSpecification space_spec(kmodel_, pc_to_use.group);
  ModelBasedPlanningContextSpecification context_spec;
  context_spec.planning_scene_ = planning_scene;
  context_spec.config_ = pc_to_use.config;
  context_spec.ik_allocators_ = ik_allocators_;
  context_spec.planner_allocator_ = getPlannerAllocator();
  
  ModelBasedPlanningContextPtr context = planning_context_factories_[index]->getNewPlanningContext(pc_to_use.name, space_spec, context_spec);
  
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
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, res.error_code, attempts, timeout);
  if (!context)
    return false;
  
  // get the starting state
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
  context->setStartState(start_state);
  context->setupConstraints(req.motion_plan_request.goal_constraints, req.motion_plan_request.path_constraints, &res.error_code);
  context->setPlanningVolume(req.motion_plan_request.workspace_parameters);
  
  
  if (context->solve(timeout, attempts))
  {
    double ptime = context->getLastPlanTime();
    if (ptime < timeout)
      context->simplifySolution(timeout - ptime);
    context->interpolateSolution();
    
    // fill the response
    ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
	      context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    pm::kinematicStateToRobotState(start_state, res.robot_state);
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

bool ompl_interface::OMPLInterface::benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const moveit_msgs::ComputePlanningBenchmark::Request &req, moveit_msgs::ComputePlanningBenchmark::Response &res) const
{
  /*
  PlanningConfigurationPtr pc;
  unsigned int attempts = 0;
  double timeout = 0.0;
  if (!prepareForSolve(req.motion_plan_request, res.error_code, pc, attempts, timeout))
    return false;

  // get the starting state
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
  
  // configure the planning configuration
  boost::mutex::scoped_lock slock(pc->lock_);
  
  if (!pc->setupPlanningContext(planning_scene, start_state, req.motion_plan_request.goal_constraints, req.motion_plan_request.path_constraints, &res.error_code))
    return false;
  pc->setPlanningVolume(req.motion_plan_request.workspace_parameters);
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  
  return pc->benchmark(timeout, attempts, req.filename);
  */
}

/*
ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints, double timeout) const
{
  moveit_msgs::Constraints empty;
  return solve(planning_scene, config, start_state, goal_constraints, empty, timeout);
}

ompl::base::PathPtr ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                         const std::string &config, const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints,
                                                         const moveit_msgs::Constraints &path_constraints, double timeout) const
{ 
  ompl::tools::Profiler::ScopedStart pslv;
  
  std::map<std::string, PlanningConfigurationPtr>::const_iterator pc = planning_configurations_.find(config);
  if (pc == planning_configurations_.end())
  {
    ROS_ERROR("Planner configuration '%s' not found", config.c_str()); 
    return ompl::base::PathPtr();  
  }
  
  // configure the planning group
  std::vector<moveit_msgs::Constraints> goal_constraints_v(1, goal_constraints);  
  boost::mutex::scoped_lock slock(pc->second->lock_);
  
  if (!pc->second->setupPlanningContext(planning_scene, start_state, goal_constraints_v, path_constraints))
    return ompl::base::PathPtr();  
  
  last_planning_configuration_solve_ = pc->second;
  
  // solve the planning problem
  if (pc->second->solve(timeout, 1))
  {
    double ptime = pc->second->getLastPlanTime();
    if (ptime < timeout)
      pc->second->simplifySolution(timeout - ptime);
    pc->second->interpolateSolution();
    return ompl::base::PathPtr(new ompl::geometric::PathGeometric(pc->second->getOMPLSimpleSetup().getSolutionPath()));
  }
  
  return ompl::base::PathPtr();  
}


const ompl_interface::PlanningConfigurationPtr& ompl_interface::OMPLInterface::getPlanningConfiguration(const std::string &config) const
{
  std::map<std::string, PlanningConfigurationPtr>::const_iterator pc = planning_configurations_.find(config);
  if (pc == planning_configurations_.end())
  {
    ROS_ERROR("Planner configuration '%s' not found", config.c_str());
    static PlanningConfigurationPtr empty;
    return empty;
  }
  else
    return pc->second;
}

void ompl_interface::OMPLInterface::addConstraintApproximation(const moveit_msgs::Constraints &constr, const std::string &group, unsigned int samples)
{
  addConstraintApproximation(constr, constr, group, samples);
}

void ompl_interface::OMPLInterface::addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard, const std::string &group, unsigned int samples)
{  
  const PlanningConfigurationPtr &pc = getPlanningConfiguration(group);
  if (pc)
  {
    ompl::base::StateStoragePtr ss = pc->constructConstraintApproximation(constr_sampling, constr_hard, samples);
    if (ss)
      constraints_->push_back(ConstraintApproximation(kmodel_, group, constr_hard, group + "_" + 
						      boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss));
    else
      ROS_ERROR("Unable to construct constraint approximation for group '%s'", group.c_str());
  }
}

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string &path)
{
  ROS_INFO("Loading constrained space approximations from '%s'", path.c_str());
  
  std::ifstream fin((path + "/list").c_str());
  if (!fin.good())
    ROS_ERROR("Unable to find 'list' in folder '%s'", path.c_str());
  
  while (fin.good() && !fin.eof())
  {
    std::string group, serialization, filename;
    fin >> group;
    if (fin.eof())
      break;
    fin >> serialization;    
    if (fin.eof())
      break;
    fin >> filename;
    const PlanningConfigurationPtr &pc = getPlanningConfiguration(group);
    if (pc)
    {
      ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pc->getOMPLSimpleSetup().getStateSpace());
      cass->load((path + "/" + filename).c_str());
      std::size_t sum = 0;
      for (std::size_t i = 0 ; i < cass->size() ; ++i)
      {
        cass->getState(i)->as<KMStateSpace::StateType>()->tag = i;
        sum += cass->getMetadata(i).size();
      }
      constraints_->push_back(ConstraintApproximation(kmodel_, group, serialization, filename, ompl::base::StateStoragePtr(cass)));
      ROS_INFO("Loaded %lu states and %lu connextions (%0.1lf per state) from %s", cass->size(), sum, (double)sum / (double)cass->size(), filename.c_str());
    }
  }
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string &path)
{
  ROS_INFO("Saving %u constrained space approximations to '%s'", (unsigned int)constraints_->size(), path.c_str());
  
  std::ofstream fout((path + "/list").c_str());
  for (std::size_t i = 0 ; i < constraints_->size() ; ++i)
  {
    fout << constraints_->at(i).group_ << std::endl;
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
    out << constraints_->at(i).ompldb_filename_ << std::endl;
    out << constraints_->at(i).constraint_msg_ << std::endl;
  }
}
*/
