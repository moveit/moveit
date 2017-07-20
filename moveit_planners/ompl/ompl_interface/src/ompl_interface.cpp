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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/profiler/profiler.h>
#include <fstream>

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& kmodel, const ros::NodeHandle& nh)
  : nh_(nh)
  , kmodel_(kmodel)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(kmodel, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  ROS_INFO("Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintApproximations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& kmodel,
                                             const planning_interface::PlannerConfigurationMap& pconfig,
                                             const ros::NodeHandle& nh)
  : nh_(nh)
  , kmodel_(kmodel)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(kmodel, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  ROS_INFO("Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintApproximations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::~OMPLInterface()
{
}

void ompl_interface::OMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  const std::vector<const robot_model::JointModelGroup*>& groups = kmodel_->getJointModelGroups();
  for (std::size_t i = 0; i < groups.size(); ++i)
  {
    if (pconfig.find(groups[i]->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = groups[i]->getName();
      pconfig2[empty.name] = empty;
    }
  }

  context_manager_.setPlannerConfigurations(pconfig2);
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req) const
{
  moveit_msgs::MoveItErrorCodes dummy;
  return getPlanningContext(planning_scene, req, dummy);
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(planning_scene, req, error_code);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const std::string& config, const std::string& factory_type) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(config, factory_type);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  if (use_constraints_approximations_)
    context->setConstraintsApproximations(constraints_library_);
  else
    context->setConstraintsApproximations(ConstraintsLibraryPtr());
  context->simplifySolutions(simplify_solutions_);
}

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string& path)
{
  constraints_library_->loadConstraintApproximations(path);
  std::stringstream ss;
  constraints_library_->printConstraintApproximations(ss);
  ROS_INFO_STREAM(ss.str());
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string& path)
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
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));
}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  const std::vector<std::string>& group_names = kmodel_->getJointModelGroupNames();
  planning_interface::PlannerConfigurationMap pconfig;

  // read the planning configuration for each group
  pconfig.clear();
  for (std::size_t i = 0; i < group_names.size(); ++i)
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space" };

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (std::size_t k = 0; k < sizeof(KNOWN_GROUP_PARAMS) / sizeof(std::string); ++k)
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
      planning_interface::PlannerConfigurationSettings pc;
      pc.name = group_names[i];
      pc.group = group_names[i];
      pc.config = specific_group_params;
      pconfig[pc.name] = pc;
    }

    // get parameters specific to each planner type
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_names[i] + "/planner_configs", config_names))
    {
      if (config_names.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int32_t j = 0; j < config_names.size(); ++j)
          if (config_names[j].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string planner_config = static_cast<std::string>(config_names[j]);
            XmlRpc::XmlRpcValue xml_config;
            if (nh_.getParam("planner_configs/" + planner_config, xml_config))
            {
              if (xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
              {
                planning_interface::PlannerConfigurationSettings pc;
                pc.name = group_names[i] + "[" + planner_config + "]";
                pc.group = group_names[i];
                // inherit parameters from the group (which can be overriden)
                pc.config = specific_group_params;

                // read parameters specific for this configuration
                for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin(); it != xml_config.end(); ++it)
                  if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    pc.config[it->first] = static_cast<std::string>(it->second);
                  else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<double>(it->second));
                  else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<int>(it->second));
                  else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<bool>(it->second));
                pconfig[pc.name] = pc;
              }
              else
                ROS_ERROR("A planning configuration should be of type XmlRpc Struct type (for configuration '%s')",
                          planner_config.c_str());
            }
            else
              ROS_ERROR("Could not find the planner configuration '%s' on the param server", planner_config.c_str());
          }
          else
            ROS_ERROR("Planner configuration names must be of type string (for group '%s')", group_names[i].c_str());
      }
      else
        ROS_ERROR("The planner_configs argument of a group configuration should be an array of strings (for group "
                  "'%s')",
                  group_names[i].c_str());
    }
  }

  for (planning_interface::PlannerConfigurationMap::iterator it = pconfig.begin(); it != pconfig.end(); ++it)
  {
    ROS_DEBUG_STREAM_NAMED("parameters", "Parameters for configuration '" << it->first << "'");
    for (std::map<std::string, std::string>::const_iterator config_it = it->second.config.begin();
         config_it != it->second.config.end(); ++config_it)
      ROS_DEBUG_STREAM_NAMED("parameters", " - " << config_it->first << " = " << config_it->second);
  }
  setPlannerConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  ROS_INFO("OMPL ROS interface is running.");
}
