/**
 * @file stomp_optimization_task.cpp
 * @brief This defines stomp's optimization task
 *
 * @author Jorge Nicho
 * @date March 23, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdexcept>
#include "stomp_moveit/stomp_optimization_task.h"

using PluginConfigs = std::vector<std::pair<std::string, XmlRpc::XmlRpcValue> >;

static const std::string COST_FUNCTIONS_FIELD = "cost_functions";
static const std::string NOISY_FILTERS_FIELD = "noisy_filters";
static const std::string UPDATE_FILTERS_FIELD = "update_filters";
static const std::string NOISE_GENERATOR_FIELD = "noise_generator";

/**
 * @brief Convenience method to load an array of STOMP plugins
 * @param config      The parameter value
 * @param param_name  The parameter name that contains the plugin
 * @param plugins     An array of found plugins and their corresponding parameter value
 * @return true if succeeded, false otherwise.
 */
bool parsePluginConfigs(XmlRpc::XmlRpcValue config, std::string param_name, PluginConfigs& plugins)
{
  if (config.hasMember(param_name) && (config[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray))
  {
    XmlRpc::XmlRpcValue& plugin_list = config[param_name];
    std::string class_name;

    // look for the 'class' entry
    for (auto i = 0u; i < plugin_list.size(); i++)
    {
      XmlRpc::XmlRpcValue& plugin_config = plugin_list[i];
      if (plugin_config.hasMember("class") && (plugin_config["class"].getType() == XmlRpc::XmlRpcValue::TypeString))
      {
        class_name = static_cast<std::string>(plugin_config["class"]);
        plugins.push_back(std::make_pair(class_name, plugin_config));
      }
      else
      {
        ROS_ERROR("Element in the '%s' array parameter did not contain a 'class' entry", param_name.c_str());
        return false;
      }
    }
  }
  else
  {
    ROS_WARN("Plugin under entry '%s' was not found in ros parameter.", param_name.c_str());
    ROS_DEBUG("Failed to find plugin under entry '%s' in ros parameter %s", param_name.c_str(), config.toXml().c_str());
    return false;
  }

  return !plugins.empty();
}

/**
 * @struct PluginData
 * @brief   Packs plugin information into a single struct
 * @param config          The configuration data in XmlRPC format
 * @param param_key       The lookup key which should have an entry in "config"
 * @param critical        Whether this plugin is required
 * @param single_instance Whether only one plugin is expected
 * @param plugin_desc     A brief description of the plugin
 * @param robot_model     A pointer  to the robot mdel
 * @param group_name      The specified group name
 */
struct PluginData
{
  XmlRpc::XmlRpcValue config;
  std::string param_key;
  bool critical;
  bool single_instance;
  std::string plugin_desc;
  moveit::core::RobotModelConstPtr robot_model;
  std::string group_name;
};

template <typename PluginPtr, typename ClassLoaderPtr>
bool loadPlugins(const PluginData plugin_data, ClassLoaderPtr class_loader, std::vector<PluginPtr>& plugin_array)
{
  PluginConfigs plugins;

  if (parsePluginConfigs(plugin_data.config, plugin_data.param_key, plugins))
  {
    for (auto& entry : plugins)
    {
      // instantiating
      PluginPtr plugin;
      try
      {
        plugin.reset(class_loader->createUnmanagedInstance(entry.first));
      }
      catch (pluginlib::PluginlibException& ex)
      {
        if (plugin_data.critical)
        {
          ROS_ERROR("%s plugin '%s' could not be created", plugin_data.plugin_desc.c_str(), entry.first.c_str());
          return false;
        }
        else
        {
          ROS_WARN("%s plugin '%s' could not be created", plugin_data.plugin_desc.c_str(), entry.first.c_str());
          continue;
        }
      }

      // initializing
      if (plugin->initialize(plugin_data.robot_model, plugin_data.group_name, entry.second))
      {
        plugin_array.push_back(plugin);
        ROS_INFO_STREAM("Stomp Optimization Task loaded " << plugin_data.plugin_desc << " '" << plugin->getName()
                                                          << "' plugin");
      }
      else
      {
        if (plugin_data.critical)
        {
          ROS_ERROR("%s plugin '%s' failed to initialize", plugin_data.plugin_desc.c_str(), entry.first.c_str());
          return false;
        }
        else
        {
          ROS_WARN("%s plugin '%s' failed to initialize", plugin_data.plugin_desc.c_str(), entry.first.c_str());
          continue;
        }
      }

      if (plugin_data.single_instance)
      {
        break;
      }
    }

    std::stringstream ss;
    ss << "[";
    auto arrayToString = [&ss](PluginConfigs::value_type& p) { ss << p.first << " "; };

    std::for_each(plugins.begin(), plugins.end(), arrayToString);
    ss << "]";

    ROS_DEBUG("Loaded %s plugins: %s", plugin_data.plugin_desc.c_str(), ss.str().c_str());
  }
  else
  {
    return false;
  }

  return true;
}

namespace stomp_moveit
{
StompOptimizationTask::StompOptimizationTask(moveit::core::RobotModelConstPtr robot_model_ptr, std::string group_name,
                                             const XmlRpc::XmlRpcValue& config)
  : robot_model_ptr_(robot_model_ptr), group_name_(group_name)
{
  // initializing plugin loaders
  cost_function_loader_.reset(new CostFunctionLoader("stomp_moveit", "stomp_moveit::cost_functions::"
                                                                     "StompCostFunction"));
  noise_generator_loader_.reset(new NoiseGeneratorLoader("stomp_moveit", "stomp_moveit::noise_generators::"
                                                                         "StompNoiseGenerator"));
  noisy_filter_loader_.reset(new NoisyFilterLoader("stomp_moveit", "stomp_moveit::noisy_filters::StompNoisyFilter"));
  update_filter_loader_.reset(new UpdateFilterLoader("stomp_moveit", "stomp_moveit::update_filters::"
                                                                     "StompUpdateFilter"));

  // preparing plugin init data
  PluginData plugin_data;
  plugin_data.config = config;
  plugin_data.group_name = group_name_;
  plugin_data.robot_model = robot_model_ptr_;

  // loading cost function plugins
  plugin_data.param_key = COST_FUNCTIONS_FIELD;
  plugin_data.plugin_desc = "CostFunction";
  plugin_data.critical = true;
  plugin_data.single_instance = false;
  if (!loadPlugins(plugin_data, cost_function_loader_, cost_functions_))
  {
    ROS_ERROR("StompOptimizationTask/%s failed to load '%s' plugins from yaml", group_name.c_str(),
              COST_FUNCTIONS_FIELD.c_str());
    throw std::logic_error("plugin not found");
  }

  // loading noise generators
  plugin_data.param_key = NOISE_GENERATOR_FIELD;
  plugin_data.plugin_desc = "NoiseGenerator";
  plugin_data.critical = true;
  plugin_data.single_instance = true;
  if (!loadPlugins(plugin_data, noise_generator_loader_, noise_generators_))
  {
    ROS_ERROR("StompOptimizationTask/%s failed to load '%s' plugins from yaml", group_name.c_str(),
              NOISE_GENERATOR_FIELD.c_str());
    throw std::logic_error("plugin not found");
  }

  // loading noisy filter plugins
  plugin_data.param_key = NOISY_FILTERS_FIELD;
  plugin_data.plugin_desc = "NoisyFilter";
  plugin_data.critical = false;
  plugin_data.single_instance = false;
  if (!loadPlugins(plugin_data, noisy_filter_loader_, noisy_filters_))
  {
    ROS_WARN("StompOptimizationTask/%s failed to load '%s' plugins from yaml", group_name.c_str(),
             NOISY_FILTERS_FIELD.c_str());
  }

  // loading filter plugins
  plugin_data.param_key = UPDATE_FILTERS_FIELD;
  plugin_data.plugin_desc = "UpdateFilter";
  plugin_data.critical = false;
  plugin_data.single_instance = false;
  if (!loadPlugins(plugin_data, update_filter_loader_, update_filters_))
  {
    ROS_WARN("StompOptimizationTask/%s failed to load '%s' plugins from yaml", group_name.c_str(),
             UPDATE_FILTERS_FIELD.c_str());
  }
}

StompOptimizationTask::~StompOptimizationTask()
{
  // TODO Auto-generated destructor stub
}

bool StompOptimizationTask::generateNoisyParameters(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                                    std::size_t num_timesteps, int iteration_number, int rollout_number,
                                                    Eigen::MatrixXd& parameters_noise, Eigen::MatrixXd& noise)
{
  return noise_generators_.back()->generateNoise(parameters, start_timestep, num_timesteps, iteration_number,
                                                 rollout_number, parameters_noise, noise);
}

bool StompOptimizationTask::computeNoisyCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                              std::size_t num_timesteps, int iteration_number, int rollout_number,
                                              Eigen::VectorXd& costs, bool& validity)
{
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(num_timesteps, cost_functions_.size());
  Eigen::VectorXd state_costs = Eigen::VectorXd::Zero(num_timesteps);
  validity = true;
  for (auto i = 0u; i < cost_functions_.size(); i++)
  {
    bool valid;
    auto cf = cost_functions_[i];

    if (!cf->computeCosts(parameters, start_timestep, num_timesteps, iteration_number, rollout_number, state_costs,
                          valid))
    {
      return false;
    }

    validity &= valid;

    cost_matrix.col(i) = state_costs * cf->getWeight();
  }
  costs = cost_matrix.rowwise().sum();
  return true;
}

bool StompOptimizationTask::computeCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                         std::size_t num_timesteps, int iteration_number, Eigen::VectorXd& costs,
                                         bool& validity)
{
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(num_timesteps, cost_functions_.size());
  Eigen::VectorXd state_costs = Eigen::VectorXd::Zero(num_timesteps);
  validity = true;
  for (auto i = 0u; i < cost_functions_.size(); i++)
  {
    bool valid;
    auto cf = cost_functions_[i];

    if (!cf->computeCosts(parameters, start_timestep, num_timesteps, iteration_number, cf->getOptimizedIndex(),
                          state_costs, valid))
    {
      return false;
    }

    validity &= valid;

    cost_matrix.col(i) = state_costs * cf->getWeight();
  }
  costs = cost_matrix.rowwise().sum();
  return true;
}

bool StompOptimizationTask::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const moveit_msgs::MotionPlanRequest& req,
                                                 const stomp_core::StompConfiguration& config,
                                                 moveit_msgs::MoveItErrorCodes& error_code)
{
  for (auto p : noise_generators_)
  {
    if (!p->setMotionPlanRequest(planning_scene, req, config, error_code))
    {
      ROS_ERROR("Failed to set Plan Request on noise generator %s", p->getName().c_str());
      return false;
    }
  }

  for (auto p : cost_functions_)
  {
    if (!p->setMotionPlanRequest(planning_scene, req, config, error_code))
    {
      ROS_ERROR("Failed to set Plan Request on cost function %s", p->getName().c_str());
      return false;
    }
  }

  for (auto p : noisy_filters_)
  {
    if (!p->setMotionPlanRequest(planning_scene, req, config, error_code))
    {
      ROS_ERROR("Failed to set Plan Request on noisy filter %s", p->getName().c_str());
      return false;
    }
  }

  for (auto p : update_filters_)
  {
    if (!p->setMotionPlanRequest(planning_scene, req, config, error_code))
    {
      ROS_ERROR("Failed to set Plan Request on update filter %s", p->getName().c_str());
      return false;
    }
  }

  return true;
}

bool StompOptimizationTask::filterNoisyParameters(std::size_t start_timestep, std::size_t num_timesteps,
                                                  int iteration_number, int rollout_number, Eigen::MatrixXd& parameters,
                                                  bool& filtered)
{
  filtered = false;
  bool temp;
  for (auto& f : noisy_filters_)
  {
    if (f->filter(start_timestep, num_timesteps, iteration_number, rollout_number, parameters, temp))
    {
      filtered |= temp;
    }
    else
    {
      return false;
    }
  }
  return true;
}

bool StompOptimizationTask::filterParameterUpdates(std::size_t start_timestep, std::size_t num_timesteps,
                                                   int iteration_number, const Eigen::MatrixXd& parameters,
                                                   Eigen::MatrixXd& updates)
{
  bool filtered = false;
  bool temp;
  for (auto& f : update_filters_)
  {
    if (f->filter(start_timestep, num_timesteps, iteration_number, parameters, updates, temp))
    {
      filtered |= temp;
    }
    else
    {
      return false;
    }
  }
  return true;
}

void StompOptimizationTask::postIteration(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                                          double cost, const Eigen::MatrixXd& parameters)
{
  for (auto p : noise_generators_)
  {
    p->postIteration(start_timestep, num_timesteps, iteration_number, cost, parameters);
  }

  for (auto p : cost_functions_)
  {
    p->postIteration(start_timestep, num_timesteps, iteration_number, cost, parameters);
  }

  for (auto p : noisy_filters_)
  {
    p->postIteration(start_timestep, num_timesteps, iteration_number, cost, parameters);
  }

  for (auto p : update_filters_)
  {
    p->postIteration(start_timestep, num_timesteps, iteration_number, cost, parameters);
  }
}

void StompOptimizationTask::done(bool success, int total_iterations, double final_cost,
                                 const Eigen::MatrixXd& parameters)
{
  for (auto p : noise_generators_)
  {
    p->done(success, total_iterations, final_cost, parameters);
  }

  for (auto p : cost_functions_)
  {
    p->done(success, total_iterations, final_cost, parameters);
  }

  for (auto p : noisy_filters_)
  {
    p->done(success, total_iterations, final_cost, parameters);
  }

  for (auto p : update_filters_)
  {
    p->done(success, total_iterations, final_cost, parameters);
  }
}

} /* namespace stomp_moveit */
