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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <ompl/tools/debug/Profiler.h>
#include <fstream>

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr &kmodel) :
  kmodel_(kmodel),
  constraint_sampler_manager_(new  constraint_samplers::ConstraintSamplerManager()),
  context_manager_(kmodel, constraint_sampler_manager_),
  constraints_library_(new ConstraintsLibrary(context_manager_)),
  use_constraints_approximations_(true),
  simplify_solutions_(true)
{
}

ompl_interface::OMPLInterface::~OMPLInterface()
{
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(const planning_interface::MotionPlanRequest &req) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(req);
  if (ctx)
    configureConstraints(ctx);
  return ctx;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(const std::string &config, const std::string &factory_type) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(config, factory_type);
  if (ctx)
    configureConstraints(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureConstraints(const ModelBasedPlanningContextPtr &context) const
{
  if (use_constraints_approximations_)
    context->setConstraintsApproximations(constraints_library_);
  else
    context->setConstraintsApproximations(ConstraintsLibraryPtr());
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::prepareForSolve(const planning_interface::MotionPlanRequest &req, 
                                                                                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                                            moveit_msgs::MoveItErrorCodes *error_code,
                                                                                            unsigned int *attempts, double *timeout) const
{
  ot::Profiler::ScopedBlock sblock("OMPLInterface:PrepareForSolve");

  if (!planning_scene)
  { 
    logError("No planning scene supplied as input"); 
    error_code->val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return ModelBasedPlanningContextPtr();
  }
  
  robot_state::RobotState start_state = planning_scene->getCurrentState();
  robot_state::robotStateMsgToRobotState(*planning_scene->getTransforms(), req.start_state, start_state);

  ModelBasedPlanningContextPtr context = getPlanningContext(req);
  if (!context)
  {
    error_code->val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return context;
  }

  *timeout = req.allowed_planning_time;
  if (*timeout <= 0.0)
  {
    logInform("The timeout for planning must be positive (%lf specified). Assuming one second instead.", *timeout);
    *timeout = 1.0;
  }
  
  *attempts = 1;
  if (req.num_planning_attempts > 0)
    *attempts = req.num_planning_attempts;
  else
    if (req.num_planning_attempts < 0)
      logError("The number of desired planning attempts should be positive. Assuming one attempt.");
  
  context->clear();
  
  // set the planning scene
  context->setPlanningScene(planning_scene);
  context->setCompleteInitialState(start_state);
  
  context->setPlanningVolume(req.workspace_parameters);
  if (!context->setPathConstraints(req.path_constraints, error_code))
    return ModelBasedPlanningContextPtr();

  if (req.trajectory_constraints.constraints.empty())
  {
    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, error_code))
      return ModelBasedPlanningContextPtr(); 
    static const std::vector<ValidConstrainedSamplerPtr> empty;
    context->setFollowSamplers(empty);
  }
  else
  {  
    if (req.goal_constraints.empty())
    {
      // if there are no goal constraints, then the goal has to satisfy the last trajectory constraint only
      std::vector<moveit_msgs::Constraints> goal_constraints(1, req.trajectory_constraints.constraints.back());
      if (!context->setGoalConstraints(goal_constraints, req.path_constraints, error_code))
        return ModelBasedPlanningContextPtr();
    }
    else
    {
      // if there are goal constraints, the goal has to satisfy them as well as the path constraints and the last trajectory constraint
      moveit_msgs::Constraints additional_constraints = kinematic_constraints::mergeConstraints(req.path_constraints, req.trajectory_constraints.constraints.back());
      if (!context->setGoalConstraints(req.goal_constraints, additional_constraints, error_code))
        return ModelBasedPlanningContextPtr();
    }

    // construct the valid state samplers we have to go through
    std::size_t n1 = req.trajectory_constraints.constraints.size() - 1;
    logDebug("%s: Allocating %u constrained samplers.", context->getName().c_str(), (unsigned int)n1);
    std::vector<ValidConstrainedSamplerPtr> samplers(n1);
    for (std::size_t i = 0 ; i < n1 ; ++i)
    {
      constraint_samplers::ConstraintSamplerPtr cs;
      if (constraint_sampler_manager_) 
      {
        moveit_msgs::Constraints combined_constraints = kinematic_constraints::mergeConstraints(req.path_constraints, req.trajectory_constraints.constraints[i]);
        cs = constraint_sampler_manager_->selectSampler(context->getPlanningScene(), context->getJointModelGroupName(), combined_constraints);
      }
      
      kinematic_constraints::KinematicConstraintSetPtr ks(new kinematic_constraints::KinematicConstraintSet(planning_scene->getRobotModel(), planning_scene->getTransforms()));
      ks->add(req.trajectory_constraints.constraints[i]);
      samplers[i] = ValidConstrainedSamplerPtr(new ValidConstrainedSampler(context.get(), ks, cs));
    }
    context->setFollowSamplers(samplers);
  }
  try
  {
    context->configure();
    logDebug("%s: New planning context is set.", context->getName().c_str());
    error_code->val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  catch (ompl::Exception &ex)
  {
    logError("OMPL encountered an error: %s", ex.what());
    context.reset();
  }
  
  return context;
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res) const
{
  ompl::tools::Profiler::ScopedStart pslv;
  ot::Profiler::ScopedBlock sblock("OMPLInterface:Solve");

  unsigned int attempts = 1;
  double timeout = 0.0;
  
  ModelBasedPlanningContextPtr context = prepareForSolve(req, planning_scene, &res.error_code_, &attempts, &timeout);

  if (!context)
    return false;
  
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel_, context->getJointModelGroupName()));
  
  bool follow = !req.trajectory_constraints.constraints.empty();
  if (follow ? context->follow(timeout, attempts) : context->solve(timeout, attempts))
  {
    double ptime = context->getLastPlanTime();
    if (simplify_solutions_ && ptime < timeout && !follow)
    {
      context->simplifySolution(timeout - ptime);
      ptime += context->getLastSimplifyTime();
    }
    context->interpolateSolution();
    
    // fill the response
    logDebug("%s: Returning successful solution with %lu states", context->getName().c_str(),
             context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    
    context->getSolutionPath(*res.trajectory_);
    res.planning_time_ = ptime;
    return true;
  }
  else
  {
    logInform("Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
					  const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanDetailedResponse &res) const
{
  ompl::tools::Profiler::ScopedStart pslv;
  ot::Profiler::ScopedBlock sblock("OMPLInterface:Solve");
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  moveit_msgs::MoveItErrorCodes error_code; // not used
  ModelBasedPlanningContextPtr context = prepareForSolve(req, planning_scene, &error_code, &attempts, &timeout);
  if (!context)
    return false;

  bool follow = !req.trajectory_constraints.constraints.empty();
  if (follow ? context->follow(timeout, attempts) : context->solve(timeout, attempts))
  {
    res.trajectory_.reserve(3);
    
    // add info about planned solution
    double ptime = context->getLastPlanTime();
    res.processing_time_.push_back(ptime);
    res.description_.push_back("plan");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(kmodel_, context->getJointModelGroupName()));
    context->getSolutionPath(*res.trajectory_.back());
    
    // simplify solution if time remains
    if (simplify_solutions_ && ptime < timeout || !follow)
    {
      context->simplifySolution(timeout - ptime);
      res.processing_time_.push_back(context->getLastSimplifyTime());
      res.description_.push_back("simplify");
      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(kmodel_, context->getJointModelGroupName()));
      context->getSolutionPath(*res.trajectory_.back());
    }
    
    ros::WallTime start_interpolate = ros::WallTime::now();
    context->interpolateSolution();
    res.processing_time_.push_back((ros::WallTime::now() - start_interpolate).toSec());
    res.description_.push_back("interpolate");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(kmodel_, context->getJointModelGroupName()));
    context->getSolutionPath(*res.trajectory_.back());
    
    // fill the response
    logDebug("%s: Returning successful solution with %lu states", context->getName().c_str(),
             context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    logInform("Unable to solve the planning problem");
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}
/*
bool ompl_interface::OMPLInterface::benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const moveit_msgs::BenchmarkPluginRequest &req,
                                              moveit_msgs::BenchmarkPluginResponse &res) const
{  
  unsigned int attempts = 1;
  double timeout = 0.0;

  ModelBasedPlanningContextPtr context = prepareForSolve(req.motion_plan_request, planning_scene, &res.error_code, &attempts, &timeout);
  if (!context)
    return false; 
  return context->benchmark(timeout, attempts, req.filename);
}
*/
void ompl_interface::OMPLInterface::terminateSolve()
{
  const ModelBasedPlanningContextPtr &context = getLastPlanningContext();
  if (context)
    context->terminateSolve();
}
