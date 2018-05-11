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

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.hpp>
#include <class_loader/class_loader.h>

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <string>
#include <map>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "moveit_planners_ompl/OMPLDynamicReconfigureConfig.h"

#include <moveit/ompl_interface/ompl_planner_manager.h>

<<<<<<< HEAD
#include <ompl/util/Console.h>

#include <memory>

#define OMPL_ROS_LOG(ros_log_level)                                                                                    \
  {                                                                                                                    \
    ROSCONSOLE_DEFINE_LOCATION(true, ros_log_level, ROSCONSOLE_NAME_PREFIX ".ompl");                                   \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled))                                                           \
      ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_,    \
                            filename, line, __ROSCONSOLE_FUNCTION__, "%s", text.c_str());                              \
  }

ompl_interface::OMPLPlannerManager::OMPLPlannerManager()
  : planning_interface::PlannerManager(), nh_("~")
{
  constraint_sampler_manager_.reset(new constraint_samplers::ConstraintSamplerManager());
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));

  class OutputHandler : public ompl::msg::OutputHandler
  {
  public:
    void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line) override
    {
      switch (level)
      {
        case ompl::msg::LOG_DEV2:
        case ompl::msg::LOG_DEV1:
        case ompl::msg::LOG_DEBUG:
          OMPL_ROS_LOG(::ros::console::levels::Debug);
          break;
        case ompl::msg::LOG_INFO:
          OMPL_ROS_LOG(::ros::console::levels::Info);
          break;
        case ompl::msg::LOG_WARN:
          OMPL_ROS_LOG(::ros::console::levels::Warn);
          break;
        case ompl::msg::LOG_ERROR:
          OMPL_ROS_LOG(::ros::console::levels::Error);
          break;
        case ompl::msg::LOG_NONE:
        default:
          /* ignore */
          break;
      }
    }
  };

  output_handler_.reset(new OutputHandler());
  ompl::msg::useOutputHandler(output_handler_.get());
}

bool ompl_interface::OMPLPlannerManager::initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
{
  if (!ns.empty())
    nh_ = ros::NodeHandle(ns);

  kmodel_ = model;

  std::string ompl_ns = ns.empty() ? "ompl" : ns + "/ompl";
  dynamic_reconfigure_server_.reset(
      new dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>(ros::NodeHandle(nh_, ompl_ns)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&OMPLPlannerManager::dynamicReconfigureCallback, this, _1, _2));

  context_manager_.reset(new PlanningContextManager(kmodel_, constraint_sampler_manager_));

  loadPlannerConfigurations();
  // loadConstraintApproximations();
  loadConstraintSamplers();

  return true;
}
>>>>>>> c0eb73821... Cleaned up and split ompl_planner_manager into header and source file

bool ompl_interface::OMPLPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  return req.trajectory_constraints.constraints.empty();
}

std::string ompl_interface::OMPLPlannerManager::getDescription() const
{
  return "OMPL";
}

void ompl_interface::OMPLPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  const planning_interface::PlannerConfigurationMap& pconfig = getPlannerConfigurations();
  algs.clear();
  algs.reserve(pconfig.size());
  for (planning_interface::PlannerConfigurationMap::const_iterator it = pconfig.begin(); it != pconfig.end(); ++it)
    algs.push_back(it->first);
}

void ompl_interface::OMPLPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  // this call can add a few more configs than we pass in (adds defaults)
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

  context_manager_->setPlannerConfigurations(pconfig2);
  // so we read the configs instead of just setting pconfig
  PlannerManager::setPlannerConfigurations(getPlannerConfigurations());
}

planning_interface::PlanningContextPtr ompl_interface::OMPLPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_->getPlanningContext(planning_scene, req, error_code);
  if (ctx)
  {
    ctx->simplifySolutions(config_.simplify_solutions);
  }

  return ctx;
}

// bool ompl_interface::OMPLPlannerManager::saveConstraintApproximations()
// {
//   std::string cpath;
//   if (nh_.getParam("constraint_approximations_path", cpath))
//   {
//     constraints_library_->saveConstraintApproximations(cpath);
//     return true;
//   }
//   ROS_WARN("ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
//   return false;
// }

// bool ompl_interface::OMPLPlannerManager::loadConstraintApproximations()
// {
//   std::string cpath;
//   if (nh_.getParam("constraint_approximations_path", cpath))
//   {
//     constraints_library_->loadConstraintApproximations(cpath);
//     std::stringstream ss;
//     constraints_library_->printConstraintApproximations(ss);
//     ROS_INFO_STREAM(ss.str());
//     return true;
//   }
//   return false;
// }

void ompl_interface::OMPLPlannerManager::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));
}

bool ompl_interface::OMPLPlannerManager::loadPlannerConfiguration(
    const std::string& group_name, const std::string& planner_id,
    const std::map<std::string, std::string>& group_params,
    planning_interface::PlannerConfigurationSettings& planner_config)
{
  XmlRpc::XmlRpcValue xml_config;
  if (!nh_.getParam("planner_configs/" + planner_id, xml_config))
  {
    ROS_ERROR("Could not find the planner configuration '%s' on the param server", planner_id.c_str());
    return false;
  }

  if (xml_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("A planning configuration should be of type XmlRpc Struct type (for configuration '%s')",
              planner_id.c_str());
    return false;
  }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
  for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin(); it != xml_config.end(); ++it)
  {
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
      planner_config.config[it->first] = static_cast<std::string>(it->second);
    else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      planner_config.config[it->first] = boost::lexical_cast<std::string>(static_cast<double>(it->second));
    else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
      planner_config.config[it->first] = boost::lexical_cast<std::string>(static_cast<int>(it->second));
    else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      planner_config.config[it->first] = boost::lexical_cast<std::string>(static_cast<bool>(it->second));
  }

  return true;
}

void ompl_interface::OMPLPlannerManager::loadPlannerConfigurations()
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
          continue;
        }

        double value_d;
        if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_d))
        {
          specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_d);
          continue;
        }

        int value_i;
        if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_d))
        {
          specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_i);
          continue;
        }

        bool value_b;
        if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_b))
        {
          specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_b);
          continue;
        }
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (nh_.getParam(group_names[i] + "/default_planner_config", default_planner_id))
    {
      if (!loadPlannerConfiguration(group_names[i], default_planner_id, specific_group_params, default_pc))
        default_planner_id = "";
    }
    if (default_planner_id.empty())
    {
      default_pc.group = group_names[i];
      default_pc.config = specific_group_params;
      default_pc.config["type"] = "geometric::RRTConnect";
    }
    default_pc.name = group_names[i];  // this is the name of the default config
    pconfig[default_pc.name] = default_pc;

    // get parameters specific to each planner type
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_names[i] + "/planner_configs", config_names))
    {
      if (config_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("The planner_configs argument of a group configuration "
                  "should be an array of strings (for group '%s')",
                  group_names[i].c_str());
        continue;
      }

      for (std::size_t j = 0; j < config_names.size(); ++j)
      {
        if (config_names[j].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR("Planner configuration names must be of type string (for group '%s')", group_names[i].c_str());
          continue;
        }
        std::string planner_id = static_cast<std::string>(config_names[j]);

        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_names[i], planner_id, specific_group_params, pc))
          pconfig[pc.name] = pc;
      }
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

CLASS_LOADER_REGISTER_CLASS(ompl_interface::OMPLPlannerManager, planning_interface::PlannerManager);
