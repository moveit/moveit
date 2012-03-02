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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface/parameterization/model_based_planning_context.h"
#include "ompl_interface/detail/state_validity_checker.h"
#include "ompl_interface/detail/constrained_sampler.h"
#include "ompl_interface/detail/constrained_goal_sampler.h"
#include "ompl_interface/detail/goal_union.h"
#include "ompl_interface/detail/projection_evaluators.h"
#include <kinematic_constraints/utils.h>

#include <ompl/base/GoalLazySamples.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/debug/Profiler.h>

ompl_interface::ModelBasedPlanningContext::ModelBasedPlanningContext(const std::string &name, const ModelBasedStateSpacePtr &state_space, 
                                                                     const ModelBasedPlanningContextSpecification &spec) :
  spec_(spec), name_(name), ompl_state_space_(state_space), complete_initial_robot_state_(ompl_state_space_->getKinematicModel()),
  ompl_simple_setup_(ompl_state_space_), ompl_benchmark_(ompl_simple_setup_), ompl_parallel_plan_(ompl_simple_setup_.getProblemDefinition()),
  last_plan_time_(0.0), max_goal_samples_(0), max_sampling_attempts_(0), max_planning_threads_(0),
  max_velocity_(0), max_acceleration_(0.0), max_solution_segment_length_(0.0)
{
  ompl_simple_setup_.getStateSpace()->computeSignature(space_signature_);
  ompl_simple_setup_.getStateSpace()->setStateSamplerAllocator(boost::bind(&ModelBasedPlanningContext::allocPathConstrainedSampler, this, _1));
}

void ompl_interface::ModelBasedPlanningContext::setProjectionEvaluator(const std::string &peval)
{
  if (!ompl_state_space_)
  {
    ROS_ERROR("No state space is configured yet");
    return;
  }
  ob::ProjectionEvaluatorPtr pe = getProjectionEvaluator(peval);
  if (pe)
    ompl_state_space_->registerDefaultProjection(pe);
}

ompl::base::ProjectionEvaluatorPtr ompl_interface::ModelBasedPlanningContext::getProjectionEvaluator(const std::string &peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getKinematicModel()->hasLinkModel(link_name))
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
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
	if (getKinematicModel()->hasJointModel(v))
	{
	  unsigned int vc = getKinematicModel()->getJointModel(v)->getVariableCount();
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
	return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
    }
    else
      ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());  
  return ob::ProjectionEvaluatorPtr();
}
namespace ompl_interface
{
class ConstraintApproximationStateSampler : public ob::StateSampler
{
public:
  
  ConstraintApproximationStateSampler(const ob::StateSpace *space, const ConstraintApproximationStateStorage *state_storage) : 
    ob::StateSampler(space), state_storage_(state_storage)
  {
  }
  
  virtual void sampleUniform(ob::State *state)
  { 
    space_->copyState(state, state_storage_->getState(rng_.uniformInt(0, state_storage_->size() - 1)));
  }
  
  virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
  {
    int index = -1;
    int tag = near->as<ModelBasedStateSpace::StateType>()->tag;
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
  
  virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }
  
protected:
  
  /** \brief The states to sample from */
  const ConstraintApproximationStateStorage *state_storage_;  
};
}

ompl::base::StateSamplerPtr ompl_interface::ModelBasedPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
  if (ompl_state_space_.get() != ss)
    ROS_FATAL("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
  ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());
  
  if (path_constraints_)
  {
    if (spec_.constraints_)
    {
      for (std::size_t i = 0 ; i < spec_.constraints_->size() ; ++i)
	if (spec_.constraints_->at(i).group_ == getJointModelGroupName() && spec_.constraints_->at(i).space_signature_ == space_signature_ && 
	    spec_.constraints_->at(i).state_storage_ && spec_.constraints_->at(i).kconstraints_set_->equal(*path_constraints_, 0.1))
	{
	  ROS_DEBUG("Using precomputed state sampler (approximated constraint space)");
	  return ob::StateSamplerPtr(new ConstraintApproximationStateSampler(ss, spec_.constraints_->at(i).state_storage_));
	}
    }
    
    kc::ConstraintSamplerPtr cs = kc::ConstraintSampler::constructFromMessage(getJointModelGroup(), path_constraints_->getAllConstraints(), getKinematicModel(),
									      getPlanningScene()->getTransforms(), ompl_state_space_->getKinematicsAllocator(), 
									      ompl_state_space_->getKinematicsSubgroupAllocators());
    if (cs)
    {
      ROS_DEBUG("%s: Allocating specialized state sampler for state space", name_.c_str());
      return ob::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG("%s: Allocating default state sampler for state space", name_.c_str());
  return ss->allocDefaultStateSampler();
}

void ompl_interface::ModelBasedPlanningContext::configure(void)
{
  if (ompl_simple_setup_.getGoal())
  {
    // convert the input state to the corresponding OMPL state
    ompl::base::ScopedState<> ompl_start_state(ompl_state_space_);
    ompl_state_space_->copyToOMPLState(ompl_start_state.get(), getCompleteInitialRobotState());
    ompl_simple_setup_.setStartState(ompl_start_state);
    ompl_simple_setup_.setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(this)));
    
    useConfig();  
    ompl_simple_setup_.setup();
  }
}

void ompl_interface::ModelBasedPlanningContext::useConfig(void)
{
  const std::map<std::string, std::string> &config = spec_.config_;
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
  
  it = cfg.find("max_velocity");
  if (it != cfg.end())
  {
    try
    {
      max_velocity_ = boost::lexical_cast<double>(boost::trim_copy(it->second));
      ROS_INFO("%s: Maximum velocity set to %lf", name_.c_str(), max_velocity_);
    }
    catch(boost::bad_lexical_cast &e)
    {
      ROS_ERROR("%s: Unable to parse maximum velocity: %s", name_.c_str(), e.what());
    }
    cfg.erase(it);
  }
  
  it = cfg.find("max_acceleration");
  if (it != cfg.end())
  {
    try
    {
      max_velocity_ = boost::lexical_cast<double>(boost::trim_copy(it->second));
      ROS_INFO("%s: Maximum acceleration set to %lf", name_.c_str(), max_velocity_);
    }
    catch(boost::bad_lexical_cast &e)
    {
      ROS_ERROR("%s: Unable to parse maximum acceleration: %s", name_.c_str(), e.what());
    }
    cfg.erase(it);
  }
  
  if (cfg.empty())
    return;
  
  it = cfg.find("type");
  if (it == cfg.end())
  {
    if (name_ != getJointModelGroupName())
      ROS_WARN("%s: Attribute 'type' not specified in planner configuration", name_.c_str());
  }
  else
  {
    // remove the 'type' parameter; the rest are parameters for the planner itself
    std::string type = it->second;
    cfg.erase(it);
    ompl_simple_setup_.setPlannerAllocator(boost::bind(spec_.planner_allocator_, _1, type, 
						       name_ != getJointModelGroupName() ? name_ : "", cfg));
    ROS_INFO("Planner configuration '%s' will use planner '%s'. Additional configuration parameters will be set when the planner is constructed.",
	     name_.c_str(), type.c_str());
  }
  
  // call the setParams() after setup(), so we know what the params are
  ompl_simple_setup_.getSpaceInformation()->setup();
  ompl_simple_setup_.getSpaceInformation()->params().setParams(cfg, true);
  // call setup() again for possibly new param values
  ompl_simple_setup_.getSpaceInformation()->setup();
}

void ompl_interface::ModelBasedPlanningContext::setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams)
{
  if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
      wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
      wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
  {
    ROS_DEBUG("It looks like the planning volume was not specified. Using default values.");
    moveit_msgs::WorkspaceParameters default_wp;
    default_wp.min_corner.x = default_wp.min_corner.y = default_wp.min_corner.z = -1.0;
    default_wp.max_corner.x = default_wp.max_corner.y = default_wp.max_corner.z = 1.0;
    setPlanningVolume(default_wp);    
    return;
  }
  
  ROS_DEBUG("%s: Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = [%f, %f], z = [%f, %f]", name_.c_str(),
	    wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);
  
  ompl_state_space_->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x,
                                       wparams.min_corner.y, wparams.max_corner.y,
                                       wparams.min_corner.z, wparams.max_corner.z);
}

void ompl_interface::ModelBasedPlanningContext::simplifySolution(double timeout)
{
  ompl_simple_setup_.simplifySolution(timeout);
}

void ompl_interface::ModelBasedPlanningContext::interpolateSolution(void)
{
  if (ompl_simple_setup_.haveSolutionPath())
  {
    og::PathGeometric &pg = ompl_simple_setup_.getSolutionPath();
    pg.interpolate((std::size_t)floor(0.5 + pg.length() / max_solution_segment_length_));
  }
}

void ompl_interface::ModelBasedPlanningContext::convertPath(const ompl::geometric::PathGeometric &pg, moveit_msgs::RobotTrajectory &traj) const
{
  planning_models::KinematicState ks = complete_initial_robot_state_;
  const std::vector<const planning_models::KinematicModel::JointModel*> &jnt = getJointModelGroup()->getJointModels();
  std::vector<const planning_models::KinematicModel::JointModel*> onedof;
  std::vector<const planning_models::KinematicModel::JointModel*> mdof;
  traj.joint_trajectory.header.frame_id = getPlanningScene()->getPlanningFrame();
  traj.joint_trajectory.joint_names.clear();
  traj.multi_dof_joint_trajectory.joint_names.clear();
  traj.multi_dof_joint_trajectory.child_frame_ids.clear();
  for (std::size_t i = 0 ; i < jnt.size() ; ++i)
    if (jnt[i]->getVariableCount() == 1)
    {
      traj.joint_trajectory.joint_names.push_back(jnt[i]->getName());
      onedof.push_back(jnt[i]);
    }
    else
    {
      traj.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
      traj.multi_dof_joint_trajectory.frame_ids.push_back(traj.joint_trajectory.header.frame_id);
      traj.multi_dof_joint_trajectory.child_frame_ids.push_back(jnt[i]->getChildLinkModel()->getName());
      mdof.push_back(jnt[i]);
    }
  if (!onedof.empty())
    traj.joint_trajectory.points.resize(pg.getStateCount());
  if (!mdof.empty())
    traj.multi_dof_joint_trajectory.points.resize(pg.getStateCount());
  std::vector<double> times;
  pg.computeFastTimeParametrization(max_velocity_, max_acceleration_, times, 50);
  for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
  {
    ompl_state_space_->copyToKinematicState(ks, pg.getState(i));
    if (!onedof.empty())
    {
      traj.joint_trajectory.points[i].positions.resize(onedof.size());
      for (std::size_t j = 0 ; j < onedof.size() ; ++j)
	traj.joint_trajectory.points[i].positions[j] = ks.getJointState(onedof[j]->getName())->getVariableValues()[0];
      traj.joint_trajectory.points[i].time_from_start = ros::Duration(times[i]);
    }
    if (!mdof.empty())
    {
      traj.multi_dof_joint_trajectory.points[i].poses.resize(mdof.size());
      for (std::size_t j = 0 ; j < mdof.size() ; ++j)
      {
	planning_models::msgFromPose(ks.getJointState(mdof[j]->getName())->getVariableTransform(),
				     traj.multi_dof_joint_trajectory.points[i].poses[j]);
      }
      traj.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(times[i]);
    }
  }
}

bool ompl_interface::ModelBasedPlanningContext::getSolutionPath(moveit_msgs::RobotTrajectory &traj) const
{
  if (!ompl_simple_setup_.haveSolutionPath())
    return false;
  convertPath(ompl_simple_setup_.getSolutionPath(), traj);
  return true;
}

void ompl_interface::ModelBasedPlanningContext::setVerboseStateValidityChecks(bool flag)
{
  if (ompl_simple_setup_.getStateValidityChecker())
    static_cast<StateValidityChecker*>(ompl_simple_setup_.getStateValidityChecker().get())->setVerbose(flag);
}

ompl::base::GoalPtr ompl_interface::ModelBasedPlanningContext::constructGoal(void)
{ 
  // ******************* set up the goal representation, based on goal constraints
  
  std::vector<ob::GoalPtr> goals;
  for (std::size_t i = 0 ; i < goal_constraints_.size() ; ++i)
  {
    kc::ConstraintSamplerPtr cs = kc::ConstraintSampler::constructFromMessage(getJointModelGroup(), goal_constraints_[i]->getAllConstraints(), getKinematicModel(),
									      getPlanningScene()->getTransforms(), ompl_state_space_->getKinematicsAllocator(),
									      ompl_state_space_->getKinematicsSubgroupAllocators());
    ob::GoalPtr g = ob::GoalPtr(new ConstrainedGoalSampler(this, goal_constraints_[i], cs));
    goals.push_back(g);
  }
  
  if (!goals.empty())
    return goals.size() == 1 ? goals[0] : ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));
  else
    ROS_ERROR("Unable to construct goal representation");
  
  return ob::GoalPtr();
}

void ompl_interface::ModelBasedPlanningContext::setPlanningScene(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  clear();
  planning_scene_ = planning_scene;
}

void ompl_interface::ModelBasedPlanningContext::setStartState(const pm::KinematicState &complete_initial_robot_state)
{
  clear();
  complete_initial_robot_state_ = complete_initial_robot_state;
}

void ompl_interface::ModelBasedPlanningContext::clear(void)
{
  ompl_simple_setup_.clear();
  ompl_simple_setup_.clearStartStates();
  ompl_simple_setup_.setGoal(ob::GoalPtr());  
  ompl_simple_setup_.setStateValidityChecker(ob::StateValidityCheckerPtr());
  path_constraints_.reset();
  goal_constraints_.clear();
}

bool ompl_interface::ModelBasedPlanningContext::setPlanningConstraints(const std::vector<moveit_msgs::Constraints> &goal_constraints,
                                                                       const moveit_msgs::Constraints &path_constraints,
                                                                       moveit_msgs::MoveItErrorCodes *error)
{
  
  // ******************* check if the input is correct
  goal_constraints_.clear();
  for (std::size_t i = 0 ; i < goal_constraints.size() ; ++i)
  {
    moveit_msgs::Constraints constr = kc::mergeConstraints(goal_constraints[i], path_constraints);
    kc::KinematicConstraintSetPtr kset(new kc::KinematicConstraintSet(getPlanningScene()->getKinematicModel(), getPlanningScene()->getTransforms()));
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
  
  // ******************* set the path constraints to use
  path_constraints_.reset(new kc::KinematicConstraintSet(getPlanningScene()->getKinematicModel(), getPlanningScene()->getTransforms()));
  path_constraints_->add(path_constraints);

  ob::GoalPtr goal = constructGoal();
  ompl_simple_setup_.setGoal(goal);
  if (goal)
    return true;
  else
    return false;
}

bool ompl_interface::ModelBasedPlanningContext::benchmark(double timeout, unsigned int count, const std::string &filename)
{
  ompl_benchmark_.clearPlanners();
  ompl_simple_setup_.setup();  
  ompl_benchmark_.addPlanner(ompl_simple_setup_.getPlanner());
  ompl_benchmark_.setExperimentName(getKinematicModel()->getName() + "_" + getJointModelGroupName() + "_" +
				    getPlanningScene()->getName() + "_" + name_);
  
  ot::Benchmark::Request req;
  req.maxTime = timeout;
  req.runCount = count;
  req.displayProgress = true;
  req.saveConsoleOutput = false;
  ompl_benchmark_.benchmark(req);
  return filename.empty() ? ompl_benchmark_.saveResultsToFile() : ompl_benchmark_.saveResultsToFile(filename.c_str());
}

bool ompl_interface::ModelBasedPlanningContext::solve(double timeout, unsigned int count)
{
  ot::Profiler::ScopedBlock sblock("PlanningContextSolve");
  
  ompl_simple_setup_.getGoal()->clearSolutionPaths();
  const ob::PlannerPtr planner = ompl_simple_setup_.getPlanner();
  if (planner)
    planner->clear();
  bool gls = ompl_simple_setup_.getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  // just in case sampling is not started
  if (gls)
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_.getGoal().get())->startSampling();

  // try to fix invalid input states, if any
  double d = ompl_simple_setup_.getStateSpace()->getMaximumExtent() / 1000.0;
  if (!ompl_simple_setup_.getProblemDefinition()->fixInvalidInputStates(d, d, 100))
    ompl_simple_setup_.getProblemDefinition()->fixInvalidInputStates(d * 10.0, d * 10.0, 100);
  
  // \todo Fix above code to do interpolation of prefix & suffix paths for fixed states;

  ompl_simple_setup_.getSpaceInformation()->getMotionValidator()->resetMotionCounter();
  bool result = false;
  if (count <= 1)
  {
    ROS_DEBUG("%s: Solving the planning problem once...", name_.c_str());
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout);
    registerTerminationCondition(ptc);
    result = ompl_simple_setup_.solve(ptc);
    last_plan_time_ = ompl_simple_setup_.getLastPlanComputationTime();
    unregisterTerminationCondition();
  }
  else
  {
    ROS_DEBUG("%s: Solving the planning problem %u times...", name_.c_str(), count);
    ompl_parallel_plan_.clearHybridizationPaths();
    if (count <= max_planning_threads_)
    {
      ompl_parallel_plan_.clearPlanners();
      if (ompl_simple_setup_.getPlannerAllocator())
	for (unsigned int i = 0 ; i < count ; ++i)
	  ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
      else
	for (unsigned int i = 0 ; i < count ; ++i)
	  ompl_parallel_plan_.addPlanner(ompl::geometric::getDefaultPlanner(ompl_simple_setup_.getGoal())); 

      ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout);    
      registerTerminationCondition(ptc);
      ompl::time::point start = ompl::time::now();
      result = ompl_parallel_plan_.solve(ptc, 1, count, true);
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
    else
    {
      ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout);    
      registerTerminationCondition(ptc);
      ompl::time::point start = ompl::time::now();
      int n = count / max_planning_threads_;
      result = true;
      for (int i = 0 ; i < n && ptc() == false ; ++i)
      {
	ompl_parallel_plan_.clearPlanners();
	if (ompl_simple_setup_.getPlannerAllocator())
	  for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
	    ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
	else
	  for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
	    ompl_parallel_plan_.addPlanner(og::getDefaultPlanner(ompl_simple_setup_.getGoal()));
	bool r = ompl_parallel_plan_.solve(ptc, 1, max_planning_threads_, true);
	result = result && r; 
      }
      n = count % max_planning_threads_;
      if (n && ptc() == false)
      {
	ompl_parallel_plan_.clearPlanners();
	if (ompl_simple_setup_.getPlannerAllocator())
	  for (int i = 0 ; i < n ; ++i)
	    ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_.getPlannerAllocator());
	else
	  for (int i = 0 ; i < n ; ++i)
	    ompl_parallel_plan_.addPlanner(og::getDefaultPlanner(ompl_simple_setup_.getGoal()));
	bool r = ompl_parallel_plan_.solve(ptc, 1, n, true);
	result = result && r;
      }
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
  }
  
  if (gls)
    // just in case we need to stop sampling
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_.getGoal().get())->stopSampling();

  int v = ompl_simple_setup_.getSpaceInformation()->getMotionValidator()->getValidMotionCount();
  int iv = ompl_simple_setup_.getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
  ROS_DEBUG("There were %d valid motions and %d invalid motions.", v, iv);
  
  if (ompl_simple_setup_.getGoal()->isApproximate())
    ROS_WARN("Computed solution is approximate");
  
  return result;
}

void ompl_interface::ModelBasedPlanningContext::registerTerminationCondition(const ob::PlannerTerminationCondition &ptc)
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = &ptc;
}

void ompl_interface::ModelBasedPlanningContext::unregisterTerminationCondition(void)
{ 
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = NULL;
}

void ompl_interface::ModelBasedPlanningContext::terminateSolve(void)
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  if (ptc_)
    ptc_->terminate();
}

ompl::base::StateStoragePtr ompl_interface::ModelBasedPlanningContext::constructConstraintApproximation(const moveit_msgs::Constraints &constr_sampling,
                                                                                                        const moveit_msgs::Constraints &constr_hard,
                                                                                                        unsigned int samples)
{
  // state storage structure
  ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(ompl_simple_setup_.getStateSpace());
  ob::StateStoragePtr sstor(cass);
  
  // construct a sampler for the sampling constraints
  kc::KinematicConstraintSet kset(getKinematicModel(), pm::TransformsConstPtr(new pm::Transforms(getKinematicModel()->getModelFrame())));
  kset.add(constr_hard);
  
  // default state
  pm::KinematicState default_state(getKinematicModel());
  default_state.setToDefaultValues();
  setStartState(default_state);
  
  int nthreads = 0;
  unsigned int attempts = 0;

  ompl_state_space_->setPlanningVolume(-20.0, 20.0, -20.0, 20.0, -20.0, 20.0);

  // construct the constrained states
#pragma omp parallel
  { 
#pragma omp master
    {
	nthreads = omp_get_num_threads();    
    }
    
    pm::KinematicState kstate(default_state);
    
    const ob::SpaceInformationPtr &si = ompl_simple_setup_.getSpaceInformation();
    kc::ConstraintSamplerPtr cs = kc::ConstraintSampler::constructFromMessage(getJointModelGroup(), constr_sampling, getKinematicModel(),
									      pm::TransformsConstPtr(new pm::Transforms(getKinematicModel()->getModelFrame())),
									      ompl_state_space_->getKinematicsAllocator(),
									      ompl_state_space_->getKinematicsSubgroupAllocators());
    ConstrainedSampler *csmp = cs ? new ConstrainedSampler(this, cs) : NULL;
    ob::StateSamplerPtr ss(csmp ? ob::StateSamplerPtr(csmp) : ompl_simple_setup_.getStateSpace()->allocDefaultStateSampler());
    
    ompl::base::ScopedState<> temp(si);
    int done = -1;
    bool slow_warn = false;
    ompl::time::point start = ompl::time::now();
    while (sstor->size() < samples)
    {
      ++attempts;
#pragma omp master
      {      
	int done_now = 100 * sstor->size() / samples;
	if (done != done_now)
	{
	  done = done_now;
	  ROS_INFO("%d%% complete (kept %0.1lf%% sampled states)", done, 100.0 * (double)sstor->size() / (double)attempts);
	}

	if (!slow_warn && attempts > 10 && attempts > sstor->size() * 100)
	{
	  slow_warn = true;
	  ROS_WARN("Computation of valid state database is very slow...");
	}
      }

      if (attempts > samples && sstor->size() == 0)
      {
	ROS_ERROR("Unable to generate any samples");
	break;
      }
      
      ss->sampleUniform(temp.get());
      ompl_state_space_->copyToKinematicState(kstate, temp.get());
      kstate.getJointStateGroup(getJointModelGroup()->getName())->updateLinkTransforms();
      double distance = 0.0;
      if (kset.decide(kstate, distance))
      {
#pragma omp critical
	{
	  if (sstor->size() < samples)
	  {
	    temp->as<ModelBasedStateSpace::StateType>()->tag = sstor->size();
	    sstor->addState(temp.get());
	  }
	}
      }
    }
    
#pragma omp master
    {
      ROS_INFO("Generated %u states in %lf seconds", (unsigned int)sstor->size(), ompl::time::seconds(ompl::time::now() - start)); 
      if (csmp)
	ROS_INFO("Constrained sampling rate: %lf", csmp->getConstrainedSamplingRate());
    }
  }
  ROS_INFO("Computing graph connections...");
  
  ompl::tools::SelfConfig sc(ompl_simple_setup_.getSpaceInformation());
  double range = 0.0;
  sc.configurePlannerRange(range);
  unsigned int maxC = sstor->size() / 50; // connect to maximum 2% of the state space
  
  // construct connexions
  const ob::StateSpacePtr &space = ompl_simple_setup_.getStateSpace();
  std::vector<planning_models::KinematicState> kstates(nthreads, default_state);
  const std::vector<const ompl::base::State*> &states = sstor->getStates();
  std::vector<ompl::base::ScopedState<> > temps(nthreads, ompl::base::ScopedState<>(space));
  
  ompl::time::point start = ompl::time::now();
  int good = 0;
  int done = -1;
  
#pragma omp parallel for schedule(dynamic) 
  for (std::size_t j = 0 ; j < sstor->size() ; ++j)
  {
    int threadid = omp_get_thread_num();
    pm::KinematicState &kstate = kstates[threadid];
    pm::KinematicState::JointStateGroup *jsg = kstate.getJointStateGroup(getJointModelGroup()->getName());
    ompl::base::State *temp = temps[threadid].get();
    double distance = 0.0;
    int done_now = 100 * j / sstor->size();
    if (done != done_now)
    {
	done = done_now;
	ROS_INFO("%d%% complete", done);
    }
    
    for (std::size_t i = j + 1 ; i < sstor->size() ; ++i)
    {
      if (space->distance(states[j], states[i]) > range)
	continue;
	
      space->interpolate(states[j], states[i], 0.5, temp);
      ompl_state_space_->copyToKinematicState(kstate, temp);
      jsg->updateLinkTransforms();
      if (kset.decide(kstate, distance))
      {
#pragma omp critical
	{
	  cass->getMetadata(i).push_back(j);
	  cass->getMetadata(j).push_back(i);
	  good++;
	}
	if (cass->getMetadata(j).size() >= maxC)
	  break;
      }
    }
  }
  ROS_INFO("Computed possible connexions in %lf seconds. Added %d connexions", ompl::time::seconds(ompl::time::now() - start), good);
  
  return sstor;
}
