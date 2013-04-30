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
#include <moveit/profiler/profiler.h>
#include <fstream>

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr &kmodel, const ros::NodeHandle &nh) :
  nh_(nh),
  kmodel_(kmodel),
  constraint_sampler_manager_(new  constraint_samplers::ConstraintSamplerManager()),
  context_manager_(kmodel, constraint_sampler_manager_),
  constraints_library_(new ConstraintsLibrary(context_manager_)),
  use_constraints_approximations_(true),
  simplify_solutions_(true)
{  
  loadParams();
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
  moveit::Profiler::ScopedBlock sblock("OMPLInterface:PrepareForSolve");

  if (!planning_scene)
  { 
    ROS_ERROR("No planning scene supplied as input"); 
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
    ROS_INFO("The timeout for planning must be positive (%lf specified). Assuming one second instead.", *timeout);
    *timeout = 1.0;
  }
  
  *attempts = 1;
  if (req.num_planning_attempts > 0)
    *attempts = req.num_planning_attempts;
  else
    if (req.num_planning_attempts < 0)
      ROS_ERROR("The number of desired planning attempts should be positive. Assuming one attempt.");
  
  context->clear();
  
  // set the planning scene
  context->setPlanningScene(planning_scene);
  context->setCompleteInitialState(start_state);
  
  context->setPlanningVolume(req.workspace_parameters);
  if (!context->setPathConstraints(req.path_constraints, error_code))
    return ModelBasedPlanningContextPtr();
  
  if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, error_code))
    return ModelBasedPlanningContextPtr(); 
  
  try
  {
    context->configure();
    ROS_DEBUG("%s: New planning context is set.", context->getName().c_str());
    error_code->val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  catch (ompl::Exception &ex)
  {
    ROS_ERROR("OMPL encountered an error: %s", ex.what());
    context.reset();
  }
  
  return context;
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res) const
{
  moveit::Profiler::ScopedStart pslv;
  moveit::Profiler::ScopedBlock sblock("OMPLInterface:Solve");

  unsigned int attempts = 1;
  double timeout = 0.0;
  
  ModelBasedPlanningContextPtr context = prepareForSolve(req, planning_scene, &res.error_code_, &attempts, &timeout);

  if (!context)
    return false;

  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel_, context->getJointModelGroupName()));
  if (context->solve(timeout, attempts))
  {
    /*
    ompl::base::PlannerData pd(context->getOMPLSimpleSetup().getSpaceInformation());
    context->getOMPLSimpleSetup().getPlannerData(pd);
    for (unsigned int k = 0 ; k < pd.numVertices() ; ++k)
      context->getOMPLSimpleSetup().getSpaceInformation()->printState(pd.getVertex(k).getState());
    */    

    double ptime = context->getLastPlanTime();
    if (simplify_solutions_ && ptime < timeout)
    {
      context->simplifySolution(timeout - ptime);
      ptime += context->getLastSimplifyTime();
    }
    context->interpolateSolution();
    
    // fill the response
    ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
             context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    
    context->getSolutionPath(*res.trajectory_);
    res.planning_time_ = ptime;
    return true;
  }
  else
  {
    ROS_INFO("Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool ompl_interface::OMPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
					  const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanDetailedResponse &res) const
{
  moveit::Profiler::ScopedStart pslv;
  moveit::Profiler::ScopedBlock sblock("OMPLInterface:Solve");
  
  unsigned int attempts = 1;
  double timeout = 0.0;
  moveit_msgs::MoveItErrorCodes error_code; // not used
  ModelBasedPlanningContextPtr context = prepareForSolve(req, planning_scene, &error_code, &attempts, &timeout);
  if (!context)
    return false;

  if (context->solve(timeout, attempts))
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
    if (simplify_solutions_ && ptime < timeout)
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
    ROS_DEBUG("%s: Returning successful solution with %lu states", context->getName().c_str(),
              context->getOMPLSimpleSetup().getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    ROS_INFO("Unable to solve the planning problem");
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

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string &path)
{
  constraints_library_->loadConstraintApproximations(path);   
  std::stringstream ss;
  constraints_library_->printConstraintApproximations(ss);
  ROS_INFO_STREAM(ss.str());
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string &path)
{
  constraints_library_->saveConstraintApproximations(path);
}

bool ompl_interface::OMPLInterface::saveConstraintApproximations()
{
  std::string cpath;
  if (nh_.getParam("constraint_approximations_path", cpath))
  {
    saveConstraintApproximations(cpath);
    return true;
  }
  ROS_WARN("ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
  return false;
}

bool ompl_interface::OMPLInterface::loadConstraintApproximations()
{
  std::string cpath;
  if (nh_.getParam("constraint_approximations_path", cpath))
  {
    loadConstraintApproximations(cpath);
    return true;
  }
  return false;
}

void ompl_interface::OMPLInterface::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));
}

void ompl_interface::OMPLInterface::loadParams()
{ 
  ROS_INFO("Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintApproximations();
  loadConstraintSamplers();
}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  const std::vector<std::string> &group_names = kmodel_->getJointModelGroupNames();  
  std::vector<ompl_interface::PlanningConfigurationSettings> pconfig;
  // read the planning configuration for each group
  pconfig.clear();
  for (std::size_t i = 0 ; i < group_names.size() ; ++i)
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = {
      "projection_evaluator", "longest_valid_segment_fraction"
    };
    
    // get parameters specific for the group
    std::map<std::string, std::string> specific_group_params;
    for (std::size_t k = 0 ; k < sizeof(KNOWN_GROUP_PARAMS) / sizeof(std::string) ; ++k)
    {
      if (nh_.hasParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k]))
      {
	std::string value;
	if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value))
        {
          if (!value.empty())
            specific_group_params[KNOWN_GROUP_PARAMS[k]] = value;
        }
        else
        {
          double value_d;
          if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_d))
            specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_d);
          else
          {
            int value_i;
            if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_d))
              specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_i);
            else
            {
              bool value_b;
              if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_b))
                specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_b);
            }
          }
        }
      }
    }
    
    // set the parameters (if any) for the default group configuration;
    if (!specific_group_params.empty())
    {
      ompl_interface::PlanningConfigurationSettings pc;
      pc.name = group_names[i];
      pc.group = group_names[i];
      pc.config = specific_group_params;
      pconfig.push_back(pc);
    }
    
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_names[i] + "/planner_configs", config_names))
    {
      if (config_names.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
	for (int32_t j = 0; j < config_names.size() ; ++j)
	  if (config_names[j].getType() == XmlRpc::XmlRpcValue::TypeString)
	  {
	    std::string planner_config = static_cast<std::string>(config_names[j]);
	    XmlRpc::XmlRpcValue xml_config;
	    if (nh_.getParam("planner_configs/" + planner_config, xml_config))
	    {
	      if (xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
	      {
		ompl_interface::PlanningConfigurationSettings pc;
		pc.name = group_names[i] + "[" + planner_config + "]";
		pc.group = group_names[i];
		// inherit parameters from the group (which can be overriden)
		pc.config = specific_group_params;
		
		// read parameters specific for this configuration
		for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin() ; it != xml_config.end() ; ++it)
		  if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
		    pc.config[it->first] = static_cast<std::string>(it->second);
		  else
		    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
		      pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<double>(it->second));
		    else
		      if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
			pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<int>(it->second));
		      else
			if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
			  pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<bool>(it->second));
		pconfig.push_back(pc);
	      }
	      else
		ROS_ERROR("A planning configuration should be of type XmlRpc Struct type (for configuration '%s')", planner_config.c_str());
	    }
	    else
	      ROS_ERROR("Could not find the planner configuration '%s' on the param server", planner_config.c_str());
	  }
	  else
	    ROS_ERROR("Planner configuration names must be of type string (for group '%s')", group_names[i].c_str());
      }
      else
	ROS_ERROR("The planner_configs argument of a group configuration should be an array of strings (for group '%s')", group_names[i].c_str());
    }
  }
  
  for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
  {
    ROS_DEBUG_STREAM("Parameters for configuration '"<< pconfig[i].name << "'");
    for (std::map<std::string, std::string>::const_iterator it = pconfig[i].config.begin() ; it != pconfig[i].config.end() ; ++it)
      ROS_DEBUG_STREAM(it->first << " = " << it->second);
  }
  setPlanningConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  ROS_INFO("OMPL ROS interface is running.");
}
