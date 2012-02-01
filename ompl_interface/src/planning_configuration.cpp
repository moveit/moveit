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

#include "ompl_interface/planning_configuration.h"
#include "ompl_interface/detail/threadsafe_state_storage.h"
#include "ompl_interface/detail/state_validity_checker.h"
#include "ompl_interface/detail/constrained_sampler.h"
#include "ompl_interface/detail/constrained_goal_sampler.h"
#include "ompl_interface/detail/projection_evaluators.h"
#include "ompl_interface/detail/goal_union.h"

#include <kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <planning_models/conversions.h>

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

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <sstream>
#include <fstream>

#include <ompl/util/Profiler.h>

ompl::base::PlannerPtr ompl_interface::PlanningConfiguration::plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                                               const std::map<std::string, std::string> &config) const
{
  ompl::base::Planner *p = NULL;
  if (planner == "geometric::RRT")
    p = new ompl::geometric::RRT(si);
  else if (planner == "geometric::RRTConnect")
    p = new ompl::geometric::RRTConnect(si);
  else if (planner == "geometric::pRRT")
    p = new ompl::geometric::pRRT(si);
  else if (planner == "geometric::LazyRRT")
    p = new ompl::geometric::LazyRRT(si);
  else if (planner == "geometric::EST")
    p = new ompl::geometric::EST(si);
  else if (planner == "geometric::SBL")
    p = new ompl::geometric::SBL(si);
  else if (planner == "geometric::pSBL")
    p = new ompl::geometric::pSBL(si);
  else if (planner == "geometric::KPIECE")
    p = new ompl::geometric::KPIECE1(si);
  else if (planner == "geometric::BKPIECE")
    p = new ompl::geometric::BKPIECE1(si);
  else if (planner == "geometric::LBKPIECE")
    p = new ompl::geometric::LBKPIECE1(si);
  else if (planner == "geometric::RRTStar")
    p = new ompl::geometric::RRTstar(si);
  else if (planner == "geometric::PRM")
    p = new ompl::geometric::PRM(si);
  else
    ROS_WARN("%s: Unknown planner type: %s", name_.c_str(), planner.c_str());
  if (p)
    p->params().setParams(config, true);
  return ompl::base::PlannerPtr(p);
}

ompl_interface::PlanningConfiguration::PlanningConfiguration(const std::string &name, const planning_models::KinematicModelConstPtr &kmodel,
                                                             const planning_models::KinematicModel::JointModelGroup *jmg, 
                                                             const ConstraintApproximationsPtr &constraint_approx,
                                                             const std::map<std::string, std::string> &config) :
  
  name_(name), kmodel_(kmodel), joint_model_group_(jmg), kinematic_model_state_space_(new KMStateSpace(jmg)),
  ompl_state_space_(kinematic_model_state_space_), ompl_simple_setup_(ompl_state_space_), ompl_benchmark_(ompl_simple_setup_),
  parallel_plan_(ompl_simple_setup_.getProblemDefinition()), start_state_(kmodel), 
  constraint_approx_(constraint_approx), last_plan_time_(0.0),
  max_goal_samples_(10), max_sampling_attempts_(10000), max_planning_threads_(4)
  
{
  max_solution_segment_length_ = ompl_simple_setup_.getStateSpace()->getMaximumExtent() / 100.0;
  ompl_simple_setup_.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new StateValidityChecker(this)));
  static_cast<StateValidityChecker*>(ompl_simple_setup_.getStateValidityChecker().get())->useNewStartingState();
  ompl_simple_setup_.getStateSpace()->setStateSamplerAllocator(boost::bind(&PlanningConfiguration::allocPathConstrainedSampler, this, _1));
  useConfig(config);
}

ompl_interface::PlanningConfiguration::~PlanningConfiguration(void)
{
}

void ompl_interface::PlanningConfiguration::useConfig(const std::map<std::string, std::string> &config)
{
  if (config.empty())
    return;
  std::map<std::string, std::string> cfg = config;
  
  // set the projection evaluator
  std::map<std::string, std::string>::iterator it = cfg.find("projection_evaluator");
  if (it != cfg.end())
  {
    setProjectionEvaluator(boost::trim_copy(it->second));
    cfg.erase(it);
  }
  
  if (cfg.empty())
    return;
  
  it = cfg.find("type");
  if (it == cfg.end())
    ROS_WARN("%s: Attribute 'type' not specified in planner configuration", name_.c_str());
  else
  {
    // remove the 'type' parameter; the rest are parameters for the planner itself
    std::string type = it->second;
    cfg.erase(it);
    ompl_simple_setup_.setPlannerAllocator(boost::bind(&PlanningConfiguration::plannerAllocator, this, _1, type, cfg));
    ROS_INFO("Planner configuration '%s' will use planner '%s'. Additional configuration parameters will be set when the planner is constructed.",
	     name_.c_str(), type.c_str());
  }
  
  // call the setParams() after setup()
  ompl_simple_setup_.getSpaceInformation()->setup();
  ompl_simple_setup_.getSpaceInformation()->params().setParams(cfg, true);
}

void ompl_interface::PlanningConfiguration::setProjectionEvaluator(const std::string &peval)
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (kmodel_->hasLinkModel(link_name))
      ompl_simple_setup_.getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name)));
    else
      ROS_ERROR("Attempted to set projection evaluator with respect to position of link '%s', but that link is not known to the kinematic model.", link_name.c_str());
  }
  else
    if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
    {
      std::string joints = peval.substr(7, peval.length() - 8);
      boost::replace_all(joints, ",", " ");
      std::vector<std::pair<std::string, unsigned int> > j;
      std::stringstream ss(joints);
      while (ss.good() && !ss.eof())
      {
	std::string v; ss >> v >> std::ws;
	if (kmodel_->hasJointModel(v))
	{
	  unsigned int vc = kmodel_->getJointModel(v)->getVariableCount();
	  if (vc > 0)
	    j.push_back(std::make_pair(v, vc));
	  else
	    ROS_WARN("%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(), v.c_str());
	}
	else
	  ROS_ERROR("%s: Attempted to set projection evaluator with respect to value of joint '%s', but that joint is not known to the kinematic model.",
		    name_.c_str(), v.c_str());
      }
      if (j.empty())
	ROS_ERROR("%s: No valid joints specified for joint projection", name_.c_str());
      else
	ompl_simple_setup_.getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j)));
    }
    else
      ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
}

namespace ompl_interface
{
class ConstraintApproximationStateSampler : public ompl::base::StateSampler
{
public:
  
  ConstraintApproximationStateSampler(const ompl::base::StateSpace *space, const ConstraintApproximationStateStorage *state_storage) : 
    ompl::base::StateSampler(space), state_storage_(state_storage)
  {
  }
  
  virtual void sampleUniform(ompl::base::State *state)
  { 
    space_->copyState(state, state_storage_->getState(rng_.uniformInt(0, state_storage_->size() - 1)));
  }
  
  virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
  {
    int index = -1;
    int tag = near->as<KMStateSpace::StateType>()->tag;
    if (tag >= 0)
    {
      const std::vector<std::size_t> &md = state_storage_->getMetadata(tag);
      if (!md.empty() && rng_.uniform01() * md.size() > 1.0)
        index = md[rng_.uniformInt(0, md.size() - 1)];
    }
    if (index < 0)
      index = rng_.uniformInt(0, state_storage_->size() - 1);
    double dist = space_->distance(near, state_storage_->getState(index));
    if (dist > distance)
      space_->interpolate(near, state_storage_->getState(index), distance / dist, state);
    else
      space_->copyState(state, state_storage_->getState(index));
  }
  
  virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }
  
protected:
  
  /** \brief The states to sample from */
  const ConstraintApproximationStateStorage *state_storage_;  
};
}

ompl::base::StateSamplerPtr ompl_interface::PlanningConfiguration::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
  if (ompl_state_space_.get() != ss)
    ROS_FATAL("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
  ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());
  
  if (path_kinematic_constraints_set_)
  {
    if (constraint_approx_ && !path_kinematic_constraints_set_->empty())
      for (std::size_t i = 0 ; i < constraint_approx_->size() ; ++i)
        if (path_kinematic_constraints_set_->equal(*constraint_approx_->at(i).kconstraints_set_, 0.5))
          if (constraint_approx_->at(i).state_storage_)
          {
            ROS_DEBUG("Using precomputed state sampler (approximated constraint space)");
            return ompl::base::StateSamplerPtr(new ConstraintApproximationStateSampler(ss, constraint_approx_->at(i).state_storage_));
          }
    const kinematic_constraints::ConstraintSamplerPtr &cs = getConstraintsSampler(path_kinematic_constraints_set_->getAllConstraints());
    if (cs)
    {
      ROS_DEBUG("%s: Allocating specialized state sampler for state space", name_.c_str());
      return ompl::base::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG("%s: Allocating default state sampler for state space", name_.c_str());
  return ss->allocDefaultStateSampler();
}

ompl::base::GoalPtr ompl_interface::PlanningConfiguration::getGoalRepresentation(const kinematic_constraints::KinematicConstraintSetPtr &kset) const
{
  return ompl::base::GoalPtr(new ConstrainedGoalSampler(this, kset, getConstraintsSampler(kset->getAllConstraints())));
}

ompl::base::StateStoragePtr ompl_interface::PlanningConfiguration::constructConstraintApproximation(const moveit_msgs::Constraints &constr, unsigned int samples)
{
  return constructConstraintApproximation(constr, constr, samples);
}

ompl::base::StateStoragePtr ompl_interface::PlanningConfiguration::constructConstraintApproximation(const moveit_msgs::Constraints &constr_sampling,
                                                                                                    const moveit_msgs::Constraints &constr_hard,
                                                                                                    unsigned int samples)
{
  planning_scene_.reset();
  
  // state storage structure
  ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(ompl_simple_setup_.getStateSpace());
  ompl::base::StateStoragePtr sstor(cass);
  
  // construct a sampler for the sampling constraints
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, planning_models::TransformsConstPtr(new planning_models::Transforms(kmodel_->getModelFrame())));
  kset.add(constr_hard);
  
  // default state
  planning_models::KinematicState default_state(kmodel_);
  default_state.setToDefaultValues();
  int nthreads = 0;
  
  // construct the constrained states
#pragma omp parallel
  { 
#pragma omp master
    {
      nthreads = omp_get_num_threads();    
    }
    
    planning_models::KinematicState kstate(default_state);
    
    const ompl::base::SpaceInformationPtr &si = ompl_simple_setup_.getSpaceInformation();
    kinematic_constraints::ConstraintSamplerPtr cs = getConstraintsSampler(constr_sampling);
    ompl::base::StateSamplerPtr ss(cs ? ompl::base::StateSamplerPtr(new ConstrainedSampler(this, cs)) :
				   ompl_simple_setup_.getStateSpace()->allocDefaultStateSampler());
    
    ompl::base::ScopedState<> temp(si);
    unsigned int attempts = 0;
    int done = -1;
    bool slow_warn = false;
    ompl::time::point start = ompl::time::now();
    while (sstor->size() < samples)
    {
      ++attempts;
      if (!slow_warn && attempts > 10 && attempts > sstor->size() * 100)
      {
	slow_warn = true;
	ROS_WARN("Computation of valid state database is very slow...");
      }
      
      if (attempts > samples && sstor->size() == 0)
      {
	ROS_ERROR("Unable to generate any samples");
	break;
      }
      
      ss->sampleUniform(temp.get());
      getKMStateSpace().copyToKinematicState(kstate, temp.get());
      kstate.getJointStateGroup(joint_model_group_->getName())->updateLinkTransforms();
      double distance = 0.0;
      if (kset.decide(kstate, distance))
      {
#pragma omp critical
	{
	  if (sstor->size() < samples)
	    sstor->addState(temp.get());
	}
      }
      
#pragma omp master
      {
	int done_now = 100 * sstor->size() / samples;
	if (done != done_now)
	{
	  done = done_now;
	  ROS_INFO("%d%% complete", done);
	}
      }
    }
#pragma omp master
    {
      ROS_INFO("Generated %u states in %lf seconds", (unsigned int)sstor->size(), ompl::time::seconds(ompl::time::now() - start));
    }
  }
  
  // construct connexions
  const ompl::base::StateSpacePtr &space = ompl_simple_setup_.getStateSpace();
  std::vector<planning_models::KinematicState> kstates(nthreads, default_state);
  const std::vector<const ompl::base::State*> &states = sstor->getStates();
  std::vector<ompl::base::ScopedState<> > temps(nthreads, ompl::base::ScopedState<>(space));
  
  ompl::time::point start = ompl::time::now();
  int good = 0;
  
#pragma omp parallel for schedule(dynamic) 
  for (std::size_t j = 0 ; j < sstor->size() ; ++j)
  {
    int threadid = omp_get_thread_num();
    planning_models::KinematicState &kstate = kstates[threadid];
    planning_models::KinematicState::JointStateGroup *jsg = kstate.getJointStateGroup(joint_model_group_->getName());
    ompl::base::State *temp = temps[threadid].get();
    double distance = 0.0;
    
    for (std::size_t i = j + 1 ; i < sstor->size() ; ++i)
    {
      space->interpolate(states[j], states[i], 0.5, temp);
      getKMStateSpace().copyToKinematicState(kstate, temp);
      jsg->updateLinkTransforms();
      if (kset.decide(kstate, distance))
      {
#pragma omp critical
	{
	  cass->getMetadata(i).push_back(j);
	  cass->getMetadata(j).push_back(i);
	  good++;
	}
      }
    }
  }
  ROS_INFO("Computed possible connexions in %lf seconds. Added %d connexions", ompl::time::seconds(ompl::time::now() - start), good);
  
  return sstor;
}

kinematic_constraints::ConstraintSamplerPtr ompl_interface::PlanningConfiguration::getConstraintsSampler(const moveit_msgs::Constraints &constr) const
{
  if (planning_scene_)
    return kinematic_constraints::constructConstraintsSampler(joint_model_group_, constr, kmodel_, planning_scene_->getTransforms(),
                                                              ik_allocator_, ik_subgroup_allocators_);
  else
    return kinematic_constraints::constructConstraintsSampler(joint_model_group_, constr, kmodel_,
                                                              planning_models::TransformsConstPtr(new planning_models::Transforms(kmodel_->getModelFrame())),
                                                              ik_allocator_, ik_subgroup_allocators_);
}

void ompl_interface::PlanningConfiguration::setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams)
{
  ROS_DEBUG("%s: Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = [%f, %f], z = [%f, %f]", name_.c_str(),
	    wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);
  
  kinematic_model_state_space_->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x,
						  wparams.min_corner.y, wparams.max_corner.y,
						  wparams.min_corner.z, wparams.max_corner.z);
}

bool ompl_interface::PlanningConfiguration::setupPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                 const planning_models::KinematicState &start_state,
                                                                 const std::vector<moveit_msgs::Constraints> &goal_constraints,
                                                                 const moveit_msgs::Constraints &path_constraints,
                                                                 moveit_msgs::MoveItErrorCodes *error)
{
  planning_scene_ = planning_scene;
  
  // ******************* check if the input is correct
  goal_constraints_.clear();
  for (std::size_t i = 0 ; i < goal_constraints.size() ; ++i)
  {
    moveit_msgs::Constraints constr = kinematic_constraints::mergeConstraints(goal_constraints[i], path_constraints);
    kinematic_constraints::KinematicConstraintSetPtr kset(new kinematic_constraints::KinematicConstraintSet(planning_scene_->getKinematicModel(),
													    planning_scene_->getTransforms()));
    kset->add(constr);
    if (!kset->empty())
      goal_constraints_.push_back(kset);
  }
  if (goal_constraints_.empty())
  {
    ROS_WARN("%s: No goal constraints specified. There is no problem to solve.", name_.c_str());
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }
  
  // first we need to identify what kind of planning we will perform
  
  // ******************* set up the starting state for the plannig context
  // set the starting state
  start_state_ = start_state;
  
  // notify the state validity checker about the new starting state
  static_cast<StateValidityChecker*>(ompl_simple_setup_.getStateValidityChecker().get())->useNewStartingState();
  
  // convert the input state to the corresponding OMPL state
  ompl::base::ScopedState<> ompl_start_state(ompl_state_space_);
  kinematic_model_state_space_->copyToOMPLState(ompl_start_state.get(), start_state_);
  ompl_simple_setup_.setStartState(ompl_start_state);
  
  // ******************* set the path constraints to use
  path_kinematic_constraints_set_.reset(new kinematic_constraints::KinematicConstraintSet(kmodel_, planning_scene_->getTransforms()));
  path_kinematic_constraints_set_->clear();
  path_kinematic_constraints_set_->add(path_constraints);
  
  // ******************* set up the goal representation, based on goal constraints
  
  std::vector<ompl::base::GoalPtr> goals;
  for (std::size_t i = 0 ; i < goal_constraints_.size() ; ++i)
  {
    ompl::base::GoalPtr g = getGoalRepresentation(goal_constraints_[i]);
    if (g)
      goals.push_back(g);
  }
  
  if (!goals.empty())
  {
    ompl::base::GoalPtr goal = goals.size() == 1 ? goals[0] : ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));
    ompl_simple_setup_.setGoal(goal);
    ompl_simple_setup_.setup();
  }
  else
    ROS_ERROR("Unable to construct goal representation");
  
  ROS_DEBUG("%s: New planning context is set.", name_.c_str());
  return true;
}

bool ompl_interface::PlanningConfiguration::benchmark(double timeout, unsigned int count, const std::string &filename)
{
  ompl_benchmark_.clearPlanners();
  ompl_benchmark_.addPlanner(ompl_simple_setup_.getPlanner());
  ompl_benchmark_.setExperimentName(planning_scene_->getKinematicModel()->getName() + "_" + joint_model_group_->getName() + "_" +
				    planning_scene_->getName() + "_" + name_);
  
  ompl::Benchmark::Request req;
  req.maxTime = timeout;
  req.runCount = count;
  req.displayProgress = true;
  req.saveConsoleOutput = false;
  ompl_benchmark_.benchmark(req);
  return filename.empty() ? ompl_benchmark_.saveResultsToFile() : ompl_benchmark_.saveResultsToFile(filename.c_str());
}

bool ompl_interface::PlanningConfiguration::solve(double timeout, unsigned int count)
{
  ompl::Profiler::ScopedBlock sblock("PlanningConfiguration::solve");
  
  ompl_simple_setup_.getGoal()->clearSolutionPaths();
  const ompl::base::PlannerPtr planner = ompl_simple_setup_.getPlanner();
  if (planner)
    planner->clear();
  bool gls = ompl_simple_setup_.getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
  // just in case sampling is not started
  if (gls)
    static_cast<ompl::base::GoalLazySamples*>(ompl_simple_setup_.getGoal().get())->startSampling();
  // try to fix invalid input states, if any
  double d = ompl_simple_setup_.getStateSpace()->getMaximumExtent() / 1000.0;
  if (!ompl_simple_setup_.getProblemDefinition()->fixInvalidInputStates(d, d, 1000))
    ompl_simple_setup_.getProblemDefinition()->fixInvalidInputStates(d * 10.0, d * 10.0, 1000);
  
  bool result = false;
  if (count <= 1)
  {
    ROS_DEBUG("%s: Solving the planning problem once...", name_.c_str());
    result = ompl_simple_setup_.solve(timeout);
    last_plan_time_ = ompl_simple_setup_.getLastPlanComputationTime();
  }
  else
  {
    ROS_DEBUG("%s: Solving the planning problem %u times...", name_.c_str(), count);
    parallel_plan_.clearHybridizationPaths();
    if (count <= max_planning_threads_)
    {
      parallel_plan_.clearPlanners();
      if (ompl_simple_setup_.getPlannerAllocator())
	for (unsigned int i = 0 ; i < count ; ++i)
	  parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
      else
	for (unsigned int i = 0 ; i < count ; ++i)
	  parallel_plan_.addPlanner(ompl::geometric::getDefaultPlanner(ompl_simple_setup_.getGoal()));
      ompl::time::point start = ompl::time::now();
      result = parallel_plan_.solve(timeout, 1, count, true);
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
    }
    else
    {
      ompl::time::point start = ompl::time::now();
      int n = count / max_planning_threads_;
      result = true;
      for (int i = 0 ; i < n ; ++i)
      {
	parallel_plan_.clearPlanners();
	if (ompl_simple_setup_.getPlannerAllocator())
	  for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
	    parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
	else
	  for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
	    parallel_plan_.addPlanner(ompl::geometric::getDefaultPlanner(ompl_simple_setup_.getGoal()));
	bool r = parallel_plan_.solve(timeout, 1, max_planning_threads_, true);
	result = result && r;
      }
      n = count % max_planning_threads_;
      if (n)
      {
	parallel_plan_.clearPlanners();
	if (ompl_simple_setup_.getPlannerAllocator())
	  for (int i = 0 ; i < n ; ++i)
	    parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
	else
	  for (int i = 0 ; i < n ; ++i)
	    parallel_plan_.addPlanner(ompl::geometric::getDefaultPlanner(ompl_simple_setup_.getGoal()));
	bool r = parallel_plan_.solve(timeout, 1, n, true);
	result = result && r;
      }
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
    }
  }
  
  if (gls)
    // just in case we need to stop sampling
    static_cast<ompl::base::GoalLazySamples*>(ompl_simple_setup_.getGoal().get())->stopSampling();
  
  if (ompl_simple_setup_.getGoal()->isApproximate())
    ROS_WARN("Computed solution is approximate");
  ompl::Profiler::Status();
  
  return result;
}

void ompl_interface::PlanningConfiguration::simplifySolution(double timeout)
{
  ompl_simple_setup_.simplifySolution(timeout);
}

void ompl_interface::PlanningConfiguration::interpolateSolution(void)
{
  if (ompl_simple_setup_.haveSolutionPath())
  {
    ompl::geometric::PathGeometric &pg = ompl_simple_setup_.getSolutionPath();
    pg.interpolate((std::size_t)floor(0.5 + pg.length() / max_solution_segment_length_));
  }
}

void ompl_interface::PlanningConfiguration::convertPath(const ompl::geometric::PathGeometric &pg, moveit_msgs::RobotTrajectory &traj) const
{
  planning_models::KinematicState ks = start_state_;
  const std::vector<const planning_models::KinematicModel::JointModel*> &jnt = joint_model_group_->getJointModels();
  std::vector<const planning_models::KinematicModel::JointModel*> onedof;
  std::vector<const planning_models::KinematicModel::JointModel*> mdof;
  traj.joint_trajectory.header.frame_id = kmodel_->getModelFrame();
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() == 1)
    {
      traj.joint_trajectory.joint_names.push_back(jnt[i]->getName());
      onedof.push_back(jnt[i]);
    }
    else
    {
      traj.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
      traj.multi_dof_joint_trajectory.frame_ids.push_back(kmodel_->getModelFrame());
      traj.multi_dof_joint_trajectory.child_frame_ids.push_back(jnt[i]->getChildLinkModel()->getName());
      mdof.push_back(jnt[i]);
    }
  if (!onedof.empty())
    traj.joint_trajectory.points.resize(pg.states.size());
  if (!mdof.empty())
    traj.multi_dof_joint_trajectory.points.resize(pg.states.size());
  for (std::size_t i = 0 ; i < pg.states.size() ; ++i)
  {
    kinematic_model_state_space_->copyToKinematicState(ks, pg.states[i]);
    if (!onedof.empty())
    {
      traj.joint_trajectory.points[i].positions.resize(onedof.size());
      for (std::size_t j = 0 ; j < onedof.size() ; ++j)
	traj.joint_trajectory.points[i].positions[j] = ks.getJointState(onedof[j]->getName())->getVariableValues()[0];
      traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    }
    if (!mdof.empty())
    {
      traj.multi_dof_joint_trajectory.points[i].poses.resize(mdof.size());
      for (std::size_t j = 0 ; j < mdof.size() ; ++j)
      {
	planning_models::msgFromPose(ks.getJointState(mdof[j]->getName())->getVariableTransform(),
				     traj.multi_dof_joint_trajectory.points[i].poses[j]);
      }
      traj.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    }
  }
}

bool ompl_interface::PlanningConfiguration::getSolutionPath(moveit_msgs::RobotTrajectory &traj) const
{
  if (!ompl_simple_setup_.haveSolutionPath())
    return false;
  convertPath(ompl_simple_setup_.getSolutionPath(), traj);
  return true;
}

void ompl_interface::PlanningConfiguration::lock(void) const
{
  lock_.lock();
}

void ompl_interface::PlanningConfiguration::unlock(void) const
{
  lock_.unlock();
}
